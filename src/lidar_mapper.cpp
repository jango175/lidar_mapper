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

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "mavros_msgs/msg/rc_in.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

    led_strip_.clear();

    enable_bag_ = this->get_parameter("enable_bag").as_bool();

    get_current_timestamp(time_format_);

    if (enable_bag_)
    {
      bag_name_ = bag_dir_ + "lidar_bag_" + std::string(time_format_);

      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name_.c_str());
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

    orientation_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      orientation_topic_,
      qos,
      std::bind(&LidarMapper::orientation_callback, this, std::placeholders::_1)
    );

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_,
      qos,
      std::bind(&LidarMapper::gps_callback, this, std::placeholders::_1)
    );

    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      position_topic_,
      qos,
      std::bind(&LidarMapper::position_callback, this, std::placeholders::_1)
    );

    mf_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());
    mf_orientation_sub_.subscribe(this, orientation_topic_, qos.get_rmw_qos_profile());
    mf_gps_sub_.subscribe(this, gps_topic_, qos.get_rmw_qos_profile());
    mf_position_sub_.subscribe(this, position_topic_, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
      ApproximateSyncPolicy(10),
      mf_scan_sub_,
      mf_orientation_sub_,
      mf_gps_sub_,
      mf_position_sub_
    );

    double timestamp_diff_threshold = this->get_parameter("timestamp_diff_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp difference threshold: %f seconds", timestamp_diff_threshold);

    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(timestamp_diff_threshold));
    sync_->registerCallback(std::bind(&LidarMapper::approximate_sync_callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3,
                                      std::placeholders::_4));

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
  }


private:
  std::string rc_topic_ = "/mavros/rc/in";
  std::string scan_topic_ = "/ldlidar_node/scan";
  std::string orientation_topic_ = "/mavros/imu/data";
  std::string gps_topic_ = "/mavros/global_position/global";
  std::string position_topic_ = "mavros/local_position/pose";

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;

  mavros_msgs::msg::RCIn::SharedPtr last_rc_msg_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  sensor_msgs::msg::Imu::SharedPtr last_orientation_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_gps_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_position_msg_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> mf_orientation_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> mf_gps_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> mf_position_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    sensor_msgs::msg::Imu,
    sensor_msgs::msg::NavSatFix,
    geometry_msgs::msg::PoseStamped> ApproximateSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  bool enable_bag_ = false;
  const char* home = std::getenv("HOME");
  std::string home_dir_ = home ? std::string(home) : std::string(".");

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool writer_opened_ = false;
  std::string bag_dir_ = home_dir_ + "/ros2_ws/src/lidar_mapper/lidar_bags/";
  std::string bag_name_;

  char time_format_[20];
  bool script_started_ = false;
  const long unsigned int script_start_channel_ = 7;
  int script_start_state_ = 0;

  WS2812B led_strip_;


  void get_current_timestamp(char* time_format)
  {
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);
    strftime(time_format, 20, "%d-%m-%y_%H:%M:%S", &datetime);
  }


  void approximate_sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                                 const sensor_msgs::msg::Imu::ConstSharedPtr& orientation_msg,
                                 const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                                 const geometry_msgs::msg::PoseStamped::ConstSharedPtr& position_msg)
  {
    if (last_rc_msg_ && last_rc_msg_->channels.size() > script_start_channel_)
    {
      if (enable_bag_)
      {
        if (script_start_state_ != 2)
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
    if (scan_msg && orientation_msg && gps_msg && position_msg)
    {
      if (enable_bag_ && writer_opened_)
      {
        auto serialized_scan_msg = std::make_shared<rclcpp::SerializedMessage>();
        auto serialized_orientation_msg = std::make_shared<rclcpp::SerializedMessage>();
        auto serialized_gps_msg = std::make_shared<rclcpp::SerializedMessage>();
        auto serialized_position_msg = std::make_shared<rclcpp::SerializedMessage>();

        rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serialization;
        scan_serialization.serialize_message(scan_msg.get(), serialized_scan_msg.get());
        writer_->write(serialized_scan_msg, "/sync_data" + scan_topic_,
                       "sensor_msgs/msg/LaserScan", scan_msg->header.stamp);

        rclcpp::Serialization<sensor_msgs::msg::Imu> orientation_serialization;
        orientation_serialization.serialize_message(orientation_msg.get(), serialized_orientation_msg.get());
        writer_->write(serialized_orientation_msg, "/sync_data" + orientation_topic_,
                       "sensor_msgs/msg/Imu", orientation_msg->header.stamp);

        rclcpp::Serialization<sensor_msgs::msg::NavSatFix> gps_serialization;
        gps_serialization.serialize_message(gps_msg.get(), serialized_gps_msg.get());
        writer_->write(serialized_gps_msg, "/sync_data" + gps_topic_,
                       "sensor_msgs/msg/NavSatFix", gps_msg->header.stamp);

        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> position_serialization;
        position_serialization.serialize_message(position_msg.get(), serialized_position_msg.get());
        writer_->write(serialized_position_msg, "/sync_data" + position_topic_,
                       "geometry_msgs/msg/PoseStamped", position_msg->header.stamp);
      }
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

    if (last_rc_msg_->channels.size() <= script_start_channel_)
    {
      return;
    }
    else
    {
      if (last_rc_msg_->channels[script_start_channel_] > 1700)
      {
        switch (script_start_state_)
        {
          case 0:
            script_start_state_ = 1;
            break;
          case 2:
            script_start_state_ = 3;
            break;
          default:
            break;
        }
      }
      else
      {
        switch (script_start_state_)
        {
          case 1:
            script_start_state_ = 2;
            break;
          case 3:
            script_start_state_ = 0;
            break;
          default:
            break;
        }
      }
    }

    if (script_start_state_ != 2)
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

      script_started_ = false;

      led_strip_.set_pixel(0, 255, 255, 255);
      led_strip_.show();

      if (enable_bag_)
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
      writer_->write(serialized_scan_msg, scan_topic_,
                     "sensor_msgs/msg/LaserScan", scan_msg->header.stamp);
    }
  }


  void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr orientation_msg)
  {
    last_orientation_msg_ = orientation_msg;

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_orientation_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<sensor_msgs::msg::Imu> orientation_serialization;
      orientation_serialization.serialize_message(orientation_msg.get(), serialized_orientation_msg.get());
      writer_->write(serialized_orientation_msg, orientation_topic_,
                     "sensor_msgs/msg/Imu", orientation_msg->header.stamp);
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


  void position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr position_msg)
  {
    last_position_msg_ = position_msg;

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_position_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<geometry_msgs::msg::PoseStamped> position_serialization;
      position_serialization.serialize_message(position_msg.get(), serialized_position_msg.get());
      writer_->write(serialized_position_msg, position_topic_,
                     "geometry_msgs/msg/PoseStamped", position_msg->header.stamp);
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
