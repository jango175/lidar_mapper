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
#include <iomanip>
#include <string>
#include <memory>
#include <ctime>
#include <chrono>
#include <functional>
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "mavros_msgs/msg/rc_in.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "ws2812b_control.hpp"


class LidarMapper : public rclcpp::Node
{
public:
  LidarMapper() : Node("lidar_mapper"),
                  led_strip_("/dev/spidev1.0", 1)
  {
    auto timestamp_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timestamp_param_desc.description = "Threshold for timestamp difference in approximate sync (seconds)";
    this->declare_parameter("timestamp_diff_threshold", 0.025, timestamp_param_desc);

    auto enable_bag_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    enable_bag_param_desc.description = "Enable or disable bag saving";
    this->declare_parameter("enable_bag", false, enable_bag_param_desc);

    auto enable_log_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    enable_log_param_desc.description = "Enable or disable log saving";
    this->declare_parameter("enable_log", false, enable_log_param_desc);

    auto use_ned_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    use_ned_param_desc.description = "Use NED frame for orientation data (true) or ENU frame (false)";
    this->declare_parameter("use_ned", true, use_ned_param_desc);

    led_strip_.clear();

    enable_bag_ = this->get_parameter("enable_bag").as_bool();
    enable_log_ = this->get_parameter("enable_log").as_bool();

    bool use_ned = this->get_parameter("use_ned").as_bool();
    if (use_ned)
      orientation_topic_ = "/msp_orientation_ned";
    else
      orientation_topic_ = "/msp_orientation_enu";

    get_current_timestamp(time_format_);

    if (enable_bag_)
    {
      bag_name_ = bag_dir_ + "lidar_bag_" + std::string(time_format_);

      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name_.c_str());
    }

    if (enable_log_)
    {
      log_file_path_ = log_dir_ + "lidar_log_" + std::string(time_format_) + ".csv";

      // Check if directory exists
      if (!std::filesystem::exists(log_dir_))
      {
        std::filesystem::create_directories(log_dir_);
      }
      RCLCPP_INFO(this->get_logger(), "Recording to log file: %s", log_file_path_.c_str());
    }

    auto qos = rclcpp::SensorDataQoS();

    rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
      rc_topic_,
      qos,
      std::bind(&LidarMapper::rc_callback, this, std::placeholders::_1)
    );

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      qos,
      std::bind(&LidarMapper::scan_callback, this, std::placeholders::_1)
    );

    orientation_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      orientation_topic_,
      qos,
      std::bind(&LidarMapper::orientation_callback, this, std::placeholders::_1)
    );

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_,
      qos,
      std::bind(&LidarMapper::gps_callback, this, std::placeholders::_1)
    );

    mf_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());
    mf_orientation_sub_.subscribe(this, orientation_topic_, qos.get_rmw_qos_profile());
    mf_gps_sub_.subscribe(this, gps_topic_, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
      ApproximateSyncPolicy(10),
      mf_scan_sub_,
      mf_orientation_sub_,
      mf_gps_sub_
    );

    double timestamp_diff_threshold = this->get_parameter("timestamp_diff_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp difference threshold: %f seconds", timestamp_diff_threshold);

    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(timestamp_diff_threshold));
    sync_->registerCallback(std::bind(&LidarMapper::approximate_sync_callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "LIDAR mapper node has been started!");
  }


  ~LidarMapper()
  {
    // Close the bag writer if it was opened
    if (enable_bag_ && writer_)
    {
      if (writer_opened_)
      {
        writer_->close();
        writer_opened_ = false;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Bag file already closed");
      }
    }

    // Ensure the log file is closed
    if (enable_log_ && log_file_.is_open())
    {
      log_file_.close();
    }
  }


private:
  std::string rc_topic_ = "/msp/rc_channels";
  std::string scan_topic_ = "/ldlidar_node/scan";
  std::string orientation_topic_ = "/msp_orientation_ned";
  std::string gps_topic_ = "/msp/gps";

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  mavros_msgs::msg::RCIn::SharedPtr last_rc_msg_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  geometry_msgs::msg::QuaternionStamped::SharedPtr last_orientation_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_gps_msg_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> mf_orientation_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> mf_gps_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    geometry_msgs::msg::QuaternionStamped,
    sensor_msgs::msg::NavSatFix> ApproximateSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  bool enable_bag_ = false;
  bool enable_log_ = false;
  const char* home = std::getenv("HOME");
  std::string home_dir_ = home ? std::string(home) : std::string(".");

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool writer_opened_ = false;
  std::string bag_dir_ = home_dir_ + "/ros2_ws/src/lidar_mapper/lidar_bags/";
  std::string bag_name_;

  std::string log_file_path_;
  std::ofstream log_file_;
  std::string log_dir_ = home_dir_ + "/ros2_ws/src/lidar_mapper/lidar_logs/";
  char time_format_[20];
  bool script_started_ = false;

  WS2812B led_strip_;


  void get_current_timestamp(char* time_format)
  {
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);
    strftime(time_format, 20, "%d-%m-%y_%H:%M:%S", &datetime);
  }


  void approximate_sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                                 const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& orientation_msg,
                                 const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg)
  {
    if (last_rc_msg_)
    {
      if (enable_log_ || enable_bag_)
      {
        if (last_rc_msg_->channels[5] < 1800)
        {
          return;
        }
        else if (script_started_ == false)
        {
          script_started_ = true;

          get_current_timestamp(time_format_);

          if (enable_bag_)
          {
            bag_name_ = bag_dir_ + "lidar_bag_" + std::string(time_format_);
            RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name_.c_str());

            if (writer_opened_ == false)
            {
              writer_->open(bag_name_);
              writer_opened_ = true;
            }
            else
            {
              RCLCPP_ERROR(this->get_logger(), "Failed to open bag writer");
            }
          }

          if (enable_log_)
          {
            log_file_path_ = log_dir_ + "lidar_log_" + std::string(time_format_) + ".csv";
            log_file_.open(log_file_path_, std::ios::app);
            RCLCPP_INFO(this->get_logger(), "Recording to log file: %s", log_file_path_.c_str());
          }

          led_strip_.set_pixel(0, 255, 0, 0);
          led_strip_.show();
        }
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "RC data corrupted!");
      return;
    }

    // Process the data
    if (scan_msg && orientation_msg && gps_msg)
    {
      if (enable_bag_ && writer_opened_)
      {
        auto serialized_scan_msg = std::make_shared<rclcpp::SerializedMessage>();
        auto serialized_orientation_msg = std::make_shared<rclcpp::SerializedMessage>();
        auto serialized_gps_msg = std::make_shared<rclcpp::SerializedMessage>();

        rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serialization;
        scan_serialization.serialize_message(scan_msg.get(), serialized_scan_msg.get());
        writer_->write(serialized_scan_msg, "/sync_data/" + scan_topic_,
                       "sensor_msgs/msg/LaserScan", scan_msg->header.stamp);

        rclcpp::Serialization<geometry_msgs::msg::QuaternionStamped> orientation_serialization;
        orientation_serialization.serialize_message(orientation_msg.get(), serialized_orientation_msg.get());
        writer_->write(serialized_orientation_msg, "/sync_data/" + orientation_topic_,
                       "geometry_msgs/msg/QuaternionStamped", orientation_msg->header.stamp);

        rclcpp::Serialization<sensor_msgs::msg::NavSatFix> gps_serialization;
        gps_serialization.serialize_message(gps_msg.get(), serialized_gps_msg.get());
        writer_->write(serialized_gps_msg, "/sync_data/" + gps_topic_,
                       "sensor_msgs/msg/NavSatFix", gps_msg->header.stamp);
      }

      if (enable_log_)
      {
        if (log_file_.is_open())
        {
          // If file is empty add header
          if (log_file_.tellp() == 0)
          {
            log_file_ << "Date Time"
                      << " Fix Latitude Longitude Altitude"
                      << " Qw Qx Qy Qz"
                      << " AngleMin AngleMax AngleIncrement RangeMin RangeMax RangesSize IntensitiesSize";

            for (size_t i = 0; i < scan_msg->ranges.size(); i++)
            {
              log_file_ << " Range[" << i << "]";
            }
            for (size_t i = 0; i < scan_msg->intensities.size(); i++)
            {
              log_file_ << " Intensity[" << i << "]";
            }
            log_file_ << std::endl;
          }

          // Timestamp
          auto now = std::chrono::system_clock::now();
          auto duration = now.time_since_epoch();
          auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
          millis = millis % 1000;

          // Date and time
          time_t t = std::chrono::system_clock::to_time_t(now);
          struct tm* tm = std::localtime(&t);

          log_file_ << std::put_time(tm, "%Y-%m-%d %H:%M:%S") << ".";
          if (millis < 100)
            log_file_ << "0";
          if (millis < 10)
            log_file_ << "0";
          log_file_ << millis << " ";

          // Log data
          log_file_ << (int)gps_msg->status.status
                    << std::fixed << std::setprecision(7)
                    << " " << gps_msg->latitude
                    << " " << gps_msg->longitude
                    << std::setprecision(2)
                    << " " << gps_msg->altitude
                    << std::setprecision(15)
                    << " " << orientation_msg->quaternion.w
                    << " " << orientation_msg->quaternion.x
                    << " " << orientation_msg->quaternion.y
                    << " " << orientation_msg->quaternion.z
                    << std::setprecision(9)
                    << " " << scan_msg->angle_min
                    << " " << scan_msg->angle_max
                    << " " << scan_msg->angle_increment
                    << std::setprecision(3)
                    << " " << scan_msg->range_min
                    << " " << scan_msg->range_max
                    << " " << scan_msg->ranges.size()
                    << " " << scan_msg->intensities.size();

          for (size_t i = 0; i < scan_msg->ranges.size(); i++)
          {
            log_file_ << " " << scan_msg->ranges[i];
          }
          for (size_t i = 0; i < scan_msg->intensities.size(); i++)
          {
            log_file_ << " " << scan_msg->intensities[i];
          }

          log_file_ << std::endl;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Error opening log file: %s", log_file_path_.c_str());

          led_strip_.set_pixel(0, 0, 0, 255);
          led_strip_.show();
        }
      }

      // // For debugging timestamp synchronization
      // RCLCPP_INFO(this->get_logger(), "Timestamps match. Data is synchronized.");
      // RCLCPP_INFO(this->get_logger(), "\tscan timestamp:\t%u.%u",
      //   scan_msg->header.stamp.sec, scan_msg->header.stamp.nanosec);
      // RCLCPP_INFO(this->get_logger(), "\torientation timestamp:\t%u.%u",
      //   orientation_msg->header.stamp.sec, orientation_msg->header.stamp.nanosec);
      // RCLCPP_INFO(this->get_logger(), "\tgps timestamp:\t%u.%u",
      //   gps_msg->header.stamp.sec, gps_msg->header.stamp.nanosec);
      // RCLCPP_WARN(this->get_logger(), "\ts - o time difference:\t %f ms",
      //   abs((scan_msg->header.stamp.sec * 1000000000 +
      //       (long int)scan_msg->header.stamp.nanosec) -
      //       (orientation_msg->header.stamp.sec * 1000000000 +
      //       (long int)orientation_msg->header.stamp.nanosec)) / 1000000.0f);
      // RCLCPP_WARN(this->get_logger(), "\ts - g time difference:\t %f ms",
      //   abs((scan_msg->header.stamp.sec * 1000000000 +
      //       (long int)scan_msg->header.stamp.nanosec) -
      //       (gps_msg->header.stamp.sec * 1000000000 +
      //       (long int)gps_msg->header.stamp.nanosec)) / 1000000.0f);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Data corrupted!");
      return;
    }
  }


  void rc_callback(const mavros_msgs::msg::RCIn::SharedPtr rc_msg)
  {
    last_rc_msg_ = rc_msg;

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_rc_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<mavros_msgs::msg::RCIn> rc_serialization;
      rc_serialization.serialize_message(rc_msg.get(), serialized_rc_msg.get());
      writer_->write(serialized_rc_msg, rc_topic_,
                     "mavros_msgs/msg/RCIn", rc_msg->header.stamp);
    }

    if (last_rc_msg_->channels[5] < 1800)
    {
      if (enable_bag_ && script_started_ && writer_)
      {
        if (writer_opened_)
        {
          writer_->close();
          writer_opened_ = false;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Bag file not opened");
        }
      }

      if (enable_log_ && script_started_ && log_file_.is_open())
      {
        log_file_.close();
      }

      script_started_ = false;

      led_strip_.set_pixel(0, 255, 255, 255);
      led_strip_.show();

      if (enable_bag_ || enable_log_)
      {
        RCLCPP_WARN(this->get_logger(), "Script not started yet...");
      }
    }
    else
    {
      if (script_started_ == false)
      {
        led_strip_.set_pixel(0, 100, 100, 100);
        led_strip_.show();
      }
    }
  }


  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    last_scan_msg_ = scan_msg;

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_scan_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serialization;
      scan_serialization.serialize_message(scan_msg.get(), serialized_scan_msg.get());
      writer_->write(serialized_scan_msg, "/ldlidar_node/scan",
                     "sensor_msgs/msg/LaserScan", scan_msg->header.stamp);
    }
  }


  void orientation_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr orientation_msg)
  {
    last_orientation_msg_ = orientation_msg;

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_orientation_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<geometry_msgs::msg::QuaternionStamped> orientation_serialization;
      orientation_serialization.serialize_message(orientation_msg.get(), serialized_orientation_msg.get());
      writer_->write(serialized_orientation_msg, orientation_topic_,
                     "geometry_msgs/msg/QuaternionStamped", orientation_msg->header.stamp);
    }
  }


  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
  {
    last_gps_msg_ = gps_msg;

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_gps_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<sensor_msgs::msg::NavSatFix> gps_serialization;
      gps_serialization.serialize_message(gps_msg.get(), serialized_gps_msg.get());
      writer_->write(serialized_gps_msg, gps_topic_,
                     "sensor_msgs/msg/NavSatFix", gps_msg->header.stamp);
    }
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting LIDAR mapper node...");

  rclcpp::spin(std::make_shared<LidarMapper>());

  rclcpp::shutdown();

  return 0;
}
