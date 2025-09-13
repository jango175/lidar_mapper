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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rosbag2_cpp/writer.hpp"

// #define SAVE_BAG       // Comment this line out to disable bag saving
// #define SHOW_TIMESTAMP // Comment this line out to disable timestamp printing
// #define SHOW_DATA      // Comment this line out to disable data printing


class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber() : Node("lidar_subscriber")
  {
    // These define the callback groups
    callback_group_scan_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_orientation_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_gps_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto scan_sub_opt = rclcpp::SubscriptionOptions();
    scan_sub_opt.callback_group = callback_group_scan_subscriber_;
    auto orientation_sub_opt = rclcpp::SubscriptionOptions();
    orientation_sub_opt.callback_group = callback_group_orientation_subscriber_;
    auto gps_sub_opt = rclcpp::SubscriptionOptions();
    gps_sub_opt.callback_group = callback_group_gps_subscriber_;

#ifdef SAVE_BAG
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);
    char time_format[20];
    strftime(time_format, 20, "%m-%d-%y:%H:%M:%S", &datetime);
    std::string bag_name = "lidar_bag_" + std::string(time_format);

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bag_name);
    RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name.c_str());
#endif // SAVE_BAG

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/ldlidar_node/scan",
      10,
      std::bind(&LidarSubscriber::scanCallback, this, std::placeholders::_1),
      scan_sub_opt
    );

    orientation_subscriber_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/msp_orientation",
      10,
      std::bind(&LidarSubscriber::orientationCallback, this, std::placeholders::_1),
      orientation_sub_opt
    );

    gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/msp_gps",
      10,
      std::bind(&LidarSubscriber::gpsCallback, this, std::placeholders::_1),
      gps_sub_opt
    );

    // Setup timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250), std::bind(&LidarSubscriber::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "LIDAR subscriber node has been started.");
  }


private:
  rclcpp::CallbackGroup::SharedPtr callback_group_scan_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_group_orientation_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_group_gps_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
  geometry_msgs::msg::QuaternionStamped::SharedPtr orientation_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr gps_msg_;


  void timer_callback()
  {
    if (scan_msg_)
    {
      RCLCPP_INFO(this->get_logger(), "Latest LiDAR scan received at time: %u.%u",
        scan_msg_->header.stamp.sec, scan_msg_->header.stamp.nanosec);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No LiDAR scan data received yet.");
    }

    if (orientation_msg_)
    {
      RCLCPP_INFO(this->get_logger(), "Latest orientation received at time: %u.%u",
        orientation_msg_->header.stamp.sec, orientation_msg_->header.stamp.nanosec);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No orientation data received yet.");
    }

    if (gps_msg_)
    {
      RCLCPP_INFO(this->get_logger(), "Latest GPS data received at time: %u.%u",
        gps_msg_->header.stamp.sec, gps_msg_->header.stamp.nanosec);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No GPS data received yet.");
    }
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

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting LIDAR subscriber node...");

  rclcpp::executors::MultiThreadedExecutor executor;
  auto sub_node = std::make_shared<LidarSubscriber>();
  executor.add_node(sub_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
