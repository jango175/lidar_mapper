/**
 * @file lidar_mapper.cpp
 * @author jango175
 * @brief LIDAR mapper ROS2 node
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <string>
#include <ctime>
#include <boost/qvm/quat.hpp>
#include <boost/qvm/quat_operations.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <unistd.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ws2812b_control.hpp"


// LidarMapper node class
class LidarMapper : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Lidar Mapper object
   * 
   */
  LidarMapper() : Node("lidar_mapper")
  {
    // parameters
    auto lidar_mount_angle_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_angle_param_desc.description = "LIDAR mount angle in degrees";
    this->declare_parameter("lidar_mount_angle_deg", 30.0, lidar_mount_angle_param_desc);

    auto mf_timeout_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    mf_timeout_param_desc.description = "Timeout for message filter tf buffer (seconds)";
    this->declare_parameter("mf_timeout", 0.25, mf_timeout_param_desc);

    auto timestamp_tolerance_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timestamp_tolerance_param_desc.description = "Timestamp tolerance for laser interpolation (seconds)";
    this->declare_parameter("timestamp_tolerance", 0.11, timestamp_tolerance_param_desc);

    auto enable_bag_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    enable_bag_param_desc.description = "Enable or disable bag saving";
    this->declare_parameter("enable_bag", false, enable_bag_param_desc);

    double lidar_mount_angle_deg = this->get_parameter("lidar_mount_angle_deg").as_double();

    enable_bag_ = this->get_parameter("enable_bag").as_bool();
    if (enable_bag_)
    {
      get_current_timestamp(time_format_);
      bag_name_ = bag_dir_ + "lidar_bag_" + std::string(time_format_);

      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name_.c_str());
    }

    double mf_timeout = this->get_parameter("mf_timeout").as_double();
    RCLCPP_INFO(this->get_logger(), "Using message filter timeout: %f seconds", mf_timeout);

    double timestamp_tolerance = this->get_parameter("timestamp_tolerance").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp tolerance: %f seconds", timestamp_tolerance);

    if (timestamp_tolerance >= mf_timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "mf_timeout is smaller than timestamp_tolerance! Exiting...");
      return;
    }

    led_strip_.clear();

    auto qos = rclcpp::SensorDataQoS();

    // tf
    boost::qvm::quat<double> q_x = boost::qvm::rotx_quat(0.0);
    boost::qvm::quat<double> q_y = boost::qvm::roty_quat(lidar_mount_angle_deg * M_PI / 180.0);
    boost::qvm::quat<double> q_z = boost::qvm::rotz_quat(0.0);
    boost::qvm::quat<double> q_lidar = q_z * q_y * q_x;
    boost::qvm::normalize(q_lidar);

    drone_lidar_tf_.header.frame_id = drone_link_;
    drone_lidar_tf_.child_frame_id = lidar_link_;
    drone_lidar_tf_.transform.translation.x = lidar_offset_x_;
    drone_lidar_tf_.transform.translation.y = lidar_offset_y_;
    drone_lidar_tf_.transform.translation.z = lidar_offset_z_;
    drone_lidar_tf_.transform.rotation.w = q_lidar.a[0];
    drone_lidar_tf_.transform.rotation.x = q_lidar.a[1];
    drone_lidar_tf_.transform.rotation.y = q_lidar.a[2];
    drone_lidar_tf_.transform.rotation.z = q_lidar.a[3];

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface()
    );
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // subscribers
    mf_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());

    mf_tf2_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      mf_scan_sub_, *tf_buffer_, drone_link_, 10,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      tf2::durationFromSec(mf_timeout)
    );
    mf_tf2_->setTolerance(rclcpp::Duration::from_seconds(timestamp_tolerance));
    mf_tf2_->registerCallback(&LidarMapper::sync_scan_callback, this);

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

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_,
      qos,
      std::bind(&LidarMapper::pose_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "LIDAR mapper node has been started!");
  }


  /**
   * @brief Destroy the Lidar Mapper object
   * 
   */
  ~LidarMapper()
  {
    // close the bag writer if it was opened
    if (enable_bag_ && writer_)
    {
      if (writer_opened_)
      {
        writer_->close();
        sync();
        writer_opened_ = false;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Bag file already closed");
      }
    }
  }


private:
  const std::string rc_topic_ = "/mavros/rc/in";
  const std::string scan_topic_ = "/ldlidar_node/scan";
  const std::string orientation_topic_ = "/mavros/imu/data";
  const std::string gps_topic_ = "/mavros/global_position/global";
  const std::string pose_topic_ = "/mavros/local_position/pose";
  const std::string sync_slice_point_cloud_topic_ = "/sync_slice_point_cloud";

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  const std::string world_link_ = "map";
  const std::string drone_link_ = "base_link";
  const std::string lidar_link_ = "ldlidar_link";

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> mf_tf2_;

  mavros_msgs::msg::RCIn::SharedPtr last_rc_msg_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  sensor_msgs::msg::Imu::SharedPtr last_orientation_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_gps_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_;

  laser_geometry::LaserProjection laser_projector_;
  geometry_msgs::msg::TransformStamped drone_lidar_tf_;
  const double lidar_offset_x_ = 0.088;
  const double lidar_offset_y_ = 0.0;
  const double lidar_offset_z_ = 0.088;

  bool enable_bag_ = false;
  const char* home_ = std::getenv("HOME");
  const std::string home_dir_ = home_ ? std::string(home_) : std::string(".");

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool writer_opened_ = false;
  const std::string bag_dir_ = home_dir_ + "/ros2_ws/src/lidar_mapper/lidar_bags/";
  std::string bag_name_;

  char time_format_[20];
  const long unsigned int script_start_channel_ = 7;
  int script_start_state_ = 0;

  WS2812B led_strip_{"/dev/spidev1.0", 1};


  /**
   * @brief Get current timestamp (%y-%m-%d_%H:%M:%S format)
   * 
   * @param time_format Pointer to the timestamp string
   */
  void get_current_timestamp(char* time_format)
  {
    time_t timestamp = time(NULL);
    struct tm datetime = *localtime(&timestamp);
    strftime(time_format, 20, "%y-%m-%d_%H:%M:%S", &datetime);
  }


  /**
   * @brief Callback for deskewed scan data
   * 
   * @param scan_msg Laser scan message pointer
   */
  void sync_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    if (script_start_state_ != 2)
      return;

    rclcpp::Duration scan_duration = rclcpp::Duration::from_seconds(scan_msg->ranges.size() * scan_msg->time_increment);
    rclcpp::Time end_of_scan = rclcpp::Time(scan_msg->header.stamp) + scan_duration;
    if (!tf_buffer_->canTransform(drone_link_,
                                  scan_msg->header.frame_id,
                                  end_of_scan,
                                  rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF data to cover the entire scan duration...");
      return;
    }

    sensor_msgs::msg::PointCloud2 sync_slice_point_cloud_msg;
    laser_projector_.transformLaserScanToPointCloud(drone_link_, *scan_msg, sync_slice_point_cloud_msg, *tf_buffer_);

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_point_cloud_msg = std::make_shared<rclcpp::SerializedMessage>();
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> point_cloud_serialization;
      point_cloud_serialization.serialize_message(&sync_slice_point_cloud_msg, serialized_point_cloud_msg.get());
      writer_->write(serialized_point_cloud_msg, sync_slice_point_cloud_topic_,
                     "sensor_msgs/msg/PointCloud2", sync_slice_point_cloud_msg.header.stamp);
    }
  }


  /**
   * @brief Callback for RC channels data
   * 
   * @param rc_msg RC channels message pointer
   */
  void rc_callback(const mavros_msgs::msg::RCIn::SharedPtr rc_msg)
  {
    last_rc_msg_ = rc_msg;

    if (last_rc_msg_->channels.size() <= script_start_channel_)
    {
      RCLCPP_ERROR(this->get_logger(), "Not enough channels to process");
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
      if (enable_bag_)
      {
        if (writer_ && writer_opened_)
        {
          writer_->close();
          sync();
          writer_opened_ = false;

          last_gps_msg_ = nullptr;
          last_orientation_msg_ = nullptr;
          last_pose_msg_ = nullptr;
          last_rc_msg_ = nullptr;
          last_scan_msg_ = nullptr;
        }
      }

      led_strip_.set_pixel(0, 255, 255, 255);
      led_strip_.show();

      RCLCPP_WARN(this->get_logger(), "Script not started yet...");
    }
    else
    {
      led_strip_.set_pixel(0, 255, 0, 0);

      if (enable_bag_)
      {
        if (last_scan_msg_ != nullptr && last_pose_msg_ != nullptr)
        {
          if (writer_ && writer_opened_ == false)
          {
            get_current_timestamp(time_format_);
            bag_name_ = bag_dir_ + "lidar_bag_" + std::string(time_format_);
            RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name_.c_str());

            writer_->open(bag_name_);
            writer_opened_ = true;
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Needed data not available yet");
          led_strip_.set_pixel(0, 100, 100, 100);
        }
      }

      led_strip_.show();
    }

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_rc_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<mavros_msgs::msg::RCIn> rc_serialization;
      rc_serialization.serialize_message(rc_msg.get(), serialized_rc_msg.get());
      writer_->write(serialized_rc_msg, rc_topic_,
                     "mavros_msgs/msg/RCIn", rc_msg->header.stamp);
    }
  }


  /**
   * @brief Callback for raw scan data
   * 
   * @param scan_msg Laser scan message pointer
   */
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


  /**
   * @brief Callback for orientation data
   * 
   * @param orientation_msg Orientation message pointer
   */
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


  /**
   * @brief Callback for GPS data
   * 
   * @param gps_msg GPS message pointer
   */
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


  /**
   * @brief Callback for local position data
   * 
   * @param pose_msg Local position message pointer
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    last_pose_msg_ = pose_msg;

    geometry_msgs::msg::TransformStamped world_drone_tf;
    world_drone_tf.header.stamp = pose_msg->header.stamp;
    world_drone_tf.header.frame_id = world_link_;
    world_drone_tf.child_frame_id = drone_link_;
    world_drone_tf.transform.translation.x = pose_msg->pose.position.x;
    world_drone_tf.transform.translation.y = pose_msg->pose.position.y;
    world_drone_tf.transform.translation.z = pose_msg->pose.position.z;
    world_drone_tf.transform.rotation.w = pose_msg->pose.orientation.w;
    world_drone_tf.transform.rotation.x = pose_msg->pose.orientation.x;
    world_drone_tf.transform.rotation.y = pose_msg->pose.orientation.y;
    world_drone_tf.transform.rotation.z = pose_msg->pose.orientation.z;

    // update just timestamp
    drone_lidar_tf_.header.stamp = pose_msg->header.stamp;

    tf_broadcaster_->sendTransform(world_drone_tf);
    tf_broadcaster_->sendTransform(drone_lidar_tf_);

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_pose_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serialization;
      pose_serialization.serialize_message(pose_msg.get(), serialized_pose_msg.get());
      writer_->write(serialized_pose_msg, pose_topic_,
                     "geometry_msgs/msg/PoseStamped", pose_msg->header.stamp);
    }
  }
};


/**
 * @brief Main function
 * 
 * @param argc Argument count
 * @param argv Argument vector
 * 
 * @return Exit status
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting LIDAR mapper node...");

  rclcpp::spin(std::make_shared<LidarMapper>());

  rclcpp::shutdown();

  return 0;
}
