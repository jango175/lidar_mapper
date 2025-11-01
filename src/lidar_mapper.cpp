// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <string>
#include <memory>
#include <ctime>
#include <thread>
#include <chrono>
#include <functional>
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "ws2812b_control.hpp"

// #define SAVE_BAG       // Comment this line out to disable bag saving
// #define SHOW_TIMESTAMP // Comment this line out to disable timestamp printing
// #define SHOW_DATA      // Comment this line out to disable data printing
// #define TEST_PERFORMANCE // Comment this line out to disable performance testing
#define ENABLE_LOG     // Comment this line out to disable logging to file

#define TIMESTAMP_DIFF_THRESHOLD 25000000 // nanoseconds


class LidarMapper : public rclcpp::Node
{
public:
  LidarMapper() : Node("lidar_mapper"), led_strip_("/dev/spidev1.0", 1)
  {
    led_strip_.clear();

    // These define the callback groups
    callback_group_rc_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_scan_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_orientation_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_gps_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto rc_sub_opt = rclcpp::SubscriptionOptions();
    rc_sub_opt.callback_group = callback_group_rc_subscriber_;
    auto scan_sub_opt = rclcpp::SubscriptionOptions();
    scan_sub_opt.callback_group = callback_group_scan_subscriber_;
    auto orientation_sub_opt = rclcpp::SubscriptionOptions();
    orientation_sub_opt.callback_group = callback_group_orientation_subscriber_;
    auto gps_sub_opt = rclcpp::SubscriptionOptions();
    gps_sub_opt.callback_group = callback_group_gps_subscriber_;

    get_current_timestamp(time_format_);

#ifdef SAVE_BAG
    std::string bag_name = "lidar_bag_" + std::string(time_format_);

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bag_name);
    RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name.c_str());
#endif // SAVE_BAG

#ifdef ENABLE_LOG
    log_file_path_ = log_dir_ + "lidar_log_" + std::string(time_format_) + ".csv";

    // check if directory exists
    if (!std::filesystem::exists(log_dir_))
    {
      std::filesystem::create_directories(log_dir_);
    }
#endif // ENABLE_LOG

    rc_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/msp_rc_channels",
      10,
      std::bind(&LidarMapper::rcCallback, this, std::placeholders::_1),
      rc_sub_opt
    );

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/ldlidar_node/scan",
      10,
      std::bind(&LidarMapper::scanCallback, this, std::placeholders::_1),
      scan_sub_opt
    );

    orientation_subscriber_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/msp_orientation",
      10,
      std::bind(&LidarMapper::orientationCallback, this, std::placeholders::_1),
      orientation_sub_opt
    );

    gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/msp_gps",
      10,
      std::bind(&LidarMapper::gpsCallback, this, std::placeholders::_1),
      gps_sub_opt
    );

    // Setup timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&LidarMapper::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "LIDAR subscriber node has been started.");
  }


  ~LidarMapper()
  {
#ifdef SAVE_BAG
    if (writer_)
    {
      writer_->close();
      RCLCPP_INFO(this->get_logger(), "Bag file has been closed.");
    }
#endif // SAVE_BAG

#ifdef ENABLE_LOG
    // Ensure the log file is closed
    if (log_file_.is_open())
    {
      log_file_.close();
    }
#endif // ENABLE_LOG
  }


private:
  rclcpp::CallbackGroup::SharedPtr callback_group_rc_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_group_scan_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_group_orientation_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_group_gps_subscriber_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::UInt16MultiArray::SharedPtr rc_msg_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
  geometry_msgs::msg::QuaternionStamped::SharedPtr orientation_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr gps_msg_;

  std::string log_file_path_;
  std::ofstream log_file_;
  std::string log_dir_ = "/home/orangepi/ros2_ws/src/lidar_mapper/lidar_logs/";
  char time_format_[20];
  bool script_started = false;

  WS2812B led_strip_;


  void get_current_timestamp(char* time_format)
  {
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);
    strftime(time_format, 20, "%d-%m-%y_%H:%M:%S", &datetime);
  }


  void timer_callback()
  {
#ifdef TEST_PERFORMANCE
    auto start = std::chrono::high_resolution_clock::now();
#endif // TEST_PERFORMANCE

    if (rc_msg_)
    {
      if (rc_msg_->data[5] < 1800)
      {
        script_started = false;

        led_strip_.set_pixel(0, 255, 255, 255);
        led_strip_.show();

        RCLCPP_WARN(this->get_logger(), "Script not started...");
        return;
      }
      else if (script_started == false)
      {
        script_started = true;

        led_strip_.set_pixel(0, 255, 0, 0);
        led_strip_.show();

#ifdef ENABLE_LOG
        get_current_timestamp(time_format_);
        log_file_path_ = log_dir_ + "lidar_log_" + std::string(time_format_) + ".csv";
#endif // ENABLE_LOG
      }
    }
    else
      return;

    if (scan_msg_ && orientation_msg_ && gps_msg_)
    {
      // Process the data
      if (abs((scan_msg_->header.stamp.sec * 1000000000 +
              (long int)scan_msg_->header.stamp.nanosec) -
              (orientation_msg_->header.stamp.sec * 1000000000 +
              (long int)orientation_msg_->header.stamp.nanosec)) < TIMESTAMP_DIFF_THRESHOLD &&
          abs((scan_msg_->header.stamp.sec * 1000000000 +
              (long int)scan_msg_->header.stamp.nanosec) -
              (gps_msg_->header.stamp.sec * 1000000000 +
              (long int)gps_msg_->header.stamp.nanosec)) < TIMESTAMP_DIFF_THRESHOLD)
      {
#ifdef ENABLE_LOG
        log_file_.open(log_file_path_, std::ios::app);
        if (log_file_.is_open())
        {
          // if file is empty add header
          if (log_file_.tellp() == 0)
          {
            log_file_ << "Date Time"
                      << " Fix Latitude Longitude Altitude"
                      << " Qw Qx Qy Qz"
                      << " AngleMin AngleMax AngleIncrement RangeMin RangeMax RangesSize IntensitiesSize";

            for (size_t i = 0; i < scan_msg_->ranges.size(); i++)
            {
              log_file_ << " Range[" << i << "]";
            }
            for (size_t i = 0; i < scan_msg_->intensities.size(); i++)
            {
              log_file_ << " Intensity[" << i << "]";
            }
            log_file_ << std::endl;
          }

          // timestamp
          auto now = std::chrono::system_clock::now();
          auto duration = now.time_since_epoch();
          auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
          millis = millis % 1000;

          // date and time
          time_t t = std::chrono::system_clock::to_time_t(now);
          struct tm* tm = std::localtime(&t);

          log_file_ << std::put_time(tm, "%Y-%m-%d %H:%M:%S") << ".";
          if (millis < 100)
            log_file_ << "0";
          if (millis < 10)
            log_file_ << "0";
          log_file_ << millis << " ";

          // log data
          log_file_ << (int)gps_msg_->status.status
                    << std::fixed << std::setprecision(7)
                    << " " << gps_msg_->latitude
                    << " " << gps_msg_->longitude
                    << std::setprecision(2)
                    << " " << gps_msg_->altitude
                    << std::setprecision(15)
                    << " " << orientation_msg_->quaternion.w
                    << " " << orientation_msg_->quaternion.x
                    << " " << orientation_msg_->quaternion.y
                    << " " << orientation_msg_->quaternion.z
                    << std::setprecision(9)
                    << " " << scan_msg_->angle_min
                    << " " << scan_msg_->angle_max
                    << " " << scan_msg_->angle_increment
                    << std::setprecision(3)
                    << " " << scan_msg_->range_min
                    << " " << scan_msg_->range_max
                    << " " << scan_msg_->ranges.size()
                    << " " << scan_msg_->intensities.size();

          for (size_t i = 0; i < scan_msg_->ranges.size(); i++)
          {
            log_file_ << " " << scan_msg_->ranges[i];
          }
          for (size_t i = 0; i < scan_msg_->intensities.size(); i++)
          {
            log_file_ << " " << scan_msg_->intensities[i];
          }

          log_file_ << std::endl;
          log_file_.close();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Error opening log file: %s", log_file_path_.c_str());

          led_strip_.set_pixel(0, 0, 0, 255);
          led_strip_.show();
        }
#endif // ENABLE_LOG

        RCLCPP_INFO(this->get_logger(), "Timestamps match. Data is synchronized.");
        // RCLCPP_INFO(this->get_logger(), "\tscan timestamp:\t%u.%u",
        //   scan_msg_->header.stamp.sec, scan_msg_->header.stamp.nanosec);
        // RCLCPP_INFO(this->get_logger(), "\torientation timestamp:\t%u.%u",
        //   orientation_msg_->header.stamp.sec, orientation_msg_->header.stamp.nanosec);
        // RCLCPP_INFO(this->get_logger(), "\tgps timestamp:\t%u.%u",
        //   gps_msg_->header.stamp.sec, gps_msg_->header.stamp.nanosec);
        // RCLCPP_WARN(this->get_logger(), "\ts - o time difference:\t %f ms",
        //   abs((scan_msg_->header.stamp.sec * 1000000000 +
        //       (long int)scan_msg_->header.stamp.nanosec) -
        //       (orientation_msg_->header.stamp.sec * 1000000000 +
        //       (long int)orientation_msg_->header.stamp.nanosec)) / 1000000.0f);
        // RCLCPP_WARN(this->get_logger(), "\ts - g time difference:\t %f ms",
        //   abs((scan_msg_->header.stamp.sec * 1000000000 +
        //       (long int)scan_msg_->header.stamp.nanosec) -
        //       (gps_msg_->header.stamp.sec * 1000000000 +
        //       (long int)gps_msg_->header.stamp.nanosec)) / 1000000.0f);

        // Clear the messages after processing
        scan_msg_.reset();
        orientation_msg_.reset();
        gps_msg_.reset();
      }
      // else
      // {
      //   RCLCPP_WARN(this->get_logger(), "Timestamps do not match. Data may be out of sync.");

      //   RCLCPP_WARN(this->get_logger(), "scan timestamp: %u.%u",
      //     scan_msg_->header.stamp.sec, scan_msg_->header.stamp.nanosec);
      //   RCLCPP_WARN(this->get_logger(), "orientation timestamp: %u.%u",
      //     orientation_msg_->header.stamp.sec, orientation_msg_->header.stamp.nanosec);
      //   RCLCPP_WARN(this->get_logger(), "gps timestamp: %u.%u",
      //     gps_msg_->header.stamp.sec, gps_msg_->header.stamp.nanosec);
      // }
    }

#ifdef TEST_PERFORMANCE
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "Processing time: %ld microseconds", duration);
#endif // TEST_PERFORMANCE
  }


  void rcCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr rc_msg)
  {
    rc_msg_ = rc_msg;
  }


  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    scan_msg_ = scan_msg;

#ifdef SAVE_BAG
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    serialization.serialize_message(scan_msg.get(), serialized_msg.get());

    rclcpp::Time time_stamp = this->now();
    writer_->write(serialized_msg, "/ldlidar_node/scan", "sensor_msgs/msg/LaserScan", time_stamp);
#endif // SAVE_BAG

#ifdef SHOW_TIMESTAMP
    RCLCPP_INFO(this->get_logger(), "LiDAR scan data recorded at time: %u.%u",
      scan_msg->header.stamp.sec, scan_msg->header.stamp.nanosec);
#endif // SHOW_TIMESTAMP

#ifdef SHOW_DATA
    RCLCPP_INFO(this->get_logger(), "========== LiDAR Scan Data ==========");

    // Display header information
    std::cout << "Frame ID: " << scan_msg->header.frame_id << std::endl;
    std::cout << "Timestamp: " << scan_msg->header.stamp.sec << "." 
              << scan_msg->header.stamp.nanosec << std::endl;

    // Display scan parameters
    std::cout << "\n--- Scan Parameters ---" << std::endl;
    std::cout << "Angle Min: " << scan_msg->angle_min << " rad (" 
              << scan_msg->angle_min * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "Angle Max: " << scan_msg->angle_max << " rad (" 
              << scan_msg->angle_max * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "Angle Increment: " << scan_msg->angle_increment << " rad (" 
              << scan_msg->angle_increment * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "Range Min: " << scan_msg->range_min << " m" << std::endl;
    std::cout << "Range Max: " << scan_msg->range_max << " m" << std::endl;
    std::cout << "Number of ranges: " << scan_msg->ranges.size() << std::endl;
    std::cout << "Number of intensities: " << scan_msg->intensities.size() << std::endl;

    // Display range values
    std::cout << "\n--- Range Data (meters) ---" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    for (size_t i = 0; i < std::min(scan_msg->ranges.size(), scan_msg->intensities.size()); ++i)
    {
      float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

      std::cout << "  [" << std::setw(3) << i << "] Angle: "
                << std::setw(7) << angle * 180.0 / M_PI << "° "
                << "Range: ";

      if (std::isfinite(scan_msg->ranges[i]) && 
        scan_msg->ranges[i] >= scan_msg->range_min && 
        scan_msg->ranges[i] <= scan_msg->range_max)
      {
        std::cout << std::setw(6) << scan_msg->ranges[i] << " m";
      }
      else
      {
        std::cout << "INVALID";
      }

      std::cout << std::setw(12) << "Intensity: ";

      if (std::isfinite(scan_msg->intensities[i]) && 
        scan_msg->intensities[i] >= 0.0)
      {
        std::cout << std::setw(7) << scan_msg->intensities[i];
      }
      else
      {
        std::cout << "INVALID";
      }

      std::cout << std::endl;
    }
#endif // SHOW_DATA
  }


  void orientationCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr orientation_msg)
  {
    orientation_msg_ = orientation_msg;

#ifdef SAVE_BAG
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<geometry_msgs::msg::QuaternionStamped> serialization;
    serialization.serialize_message(orientation_msg.get(), serialized_msg.get());

    rclcpp::Time time_stamp = this->now();
    writer_->write(serialized_msg, "/msp_orientation", "geometry_msgs/msg/QuaternionStamped", time_stamp);
#endif // SAVE_BAG

#ifdef SHOW_TIMESTAMP
    RCLCPP_INFO(this->get_logger(), "Orientation data recorded at time: %u.%u",
      orientation_msg->header.stamp.sec, orientation_msg->header.stamp.nanosec);
#endif // SHOW_TIMESTAMP

#ifdef SHOW_DATA
    RCLCPP_INFO(this->get_logger(), "========== Orientation Data ==========");

    // Display header information
    std::cout << "Frame ID: " << orientation_msg->header.frame_id << std::endl;
    std::cout << "Timestamp: " << orientation_msg->header.stamp.sec << "." 
              << orientation_msg->header.stamp.nanosec << std::endl;

    // Display quaternion values
    std::cout << "\n--- Quaternion ---" << std::endl;
    std::cout << "x: " << orientation_msg->quaternion.x << std::endl;
    std::cout << "y: " << orientation_msg->quaternion.y << std::endl;
    std::cout << "z: " << orientation_msg->quaternion.z << std::endl;
    std::cout << "w: " << orientation_msg->quaternion.w << std::endl;
#endif // SHOW_DATA
  }


  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
  {
    gps_msg_ = gps_msg;

#ifdef SAVE_BAG
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serialization;
    serialization.serialize_message(gps_msg.get(), serialized_msg.get());

    rclcpp::Time time_stamp = this->now();
    writer_->write(serialized_msg, "/msp_gps", "sensor_msgs/msg/NavSatFix", time_stamp);
#endif // SAVE_BAG

#ifdef SHOW_TIMESTAMP
    RCLCPP_INFO(this->get_logger(), "GPS data recorded at time: %u.%u",
      gps_msg->header.stamp.sec, gps_msg->header.stamp.nanosec);
#endif // SHOW_TIMESTAMP

#ifdef SHOW_DATA
    RCLCPP_INFO(this->get_logger(), "========== GPS Data ==========");

    // Display header information
    std::cout << "Frame ID: " << gps_msg->header.frame_id << std::endl;
    std::cout << "Timestamp: " << gps_msg->header.stamp.sec << "." 
              << gps_msg->header.stamp.nanosec << std::endl;

    // Display GPS values
    std::cout << "\n--- GPS ---" << std::endl;
    std::cout << "Status: " << static_cast<int>(gps_msg->status.status) << std::endl;
    std::cout << "Latitude: " << gps_msg->latitude << "°" << std::endl;
    std::cout << "Longitude: " << gps_msg->longitude << "°" << std::endl;
    std::cout << "Altitude: " << gps_msg->altitude << " m" << std::endl;
#endif // SHOW_DATA
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting LIDAR mapper node...");

  rclcpp::executors::MultiThreadedExecutor executor;
  auto lidar_mapper_node = std::make_shared<LidarMapper>();
  executor.add_node(lidar_mapper_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
