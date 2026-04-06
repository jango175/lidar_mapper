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
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    auto lidar_mount_roll_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_roll_param_desc.description = "LIDAR mount roll angle in degrees";
    this->declare_parameter("lidar_mount_roll_deg", 0.0, lidar_mount_roll_param_desc);

    auto lidar_mount_pitch_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_pitch_param_desc.description = "LIDAR mount pitch angle in degrees";
    this->declare_parameter("lidar_mount_pitch_deg", 30.0, lidar_mount_pitch_param_desc);

    auto lidar_mount_yaw_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_yaw_param_desc.description = "LIDAR mount yaw angle in degrees";
    this->declare_parameter("lidar_mount_yaw_deg", 0.0, lidar_mount_yaw_param_desc);

    auto lidar_mount_offset_x_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_offset_x_param_desc.description = "LIDAR mount offset in X axis in meters";
    this->declare_parameter("lidar_mount_offset_x", 0.088, lidar_mount_offset_x_param_desc);

    auto lidar_mount_offset_y_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_offset_y_param_desc.description = "LIDAR mount offset in Y axis in meters";
    this->declare_parameter("lidar_mount_offset_y", 0.0, lidar_mount_offset_y_param_desc);

    auto lidar_mount_offset_z_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_offset_z_param_desc.description = "LIDAR mount offset in Z axis in meters";
    this->declare_parameter("lidar_mount_offset_z", 0.088, lidar_mount_offset_z_param_desc);

    auto mf_timeout_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    mf_timeout_param_desc.description = "Timeout for message filter tf buffer in seconds";
    this->declare_parameter("mf_timeout", 0.25, mf_timeout_param_desc);

    auto timestamp_tolerance_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timestamp_tolerance_param_desc.description = "Timestamp tolerance for laser interpolation in seconds";
    this->declare_parameter("timestamp_tolerance", 0.11, timestamp_tolerance_param_desc);

    auto sor_mean_k_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    sor_mean_k_param_desc.description = "Number of neighbors to analyze for SOR filter";
    this->declare_parameter("sor_mean_k", 50, sor_mean_k_param_desc);

    auto sor_std_dev_mult_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    sor_std_dev_mult_param_desc.description = "Standard deviation multiplier for SOR filter";
    this->declare_parameter("sor_std_dev_mult", 1.0, sor_std_dev_mult_param_desc);

    auto enable_bag_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    enable_bag_param_desc.description = "Enable or disable bag saving";
    this->declare_parameter("enable_bag", false, enable_bag_param_desc);

    const double lidar_mount_roll_deg = this->get_parameter("lidar_mount_roll_deg").as_double();
    const double lidar_mount_pitch_deg = this->get_parameter("lidar_mount_pitch_deg").as_double();
    const double lidar_mount_yaw_deg = this->get_parameter("lidar_mount_yaw_deg").as_double();
    const double lidar_mount_offset_x = this->get_parameter("lidar_mount_offset_x").as_double();
    const double lidar_mount_offset_y = this->get_parameter("lidar_mount_offset_y").as_double();
    const double lidar_mount_offset_z = this->get_parameter("lidar_mount_offset_z").as_double();
    RCLCPP_INFO(this->get_logger(), "LIDAR mounting roll: %f degrees", lidar_mount_roll_deg);
    RCLCPP_INFO(this->get_logger(), "LIDAR mounting pitch: %f degrees", lidar_mount_pitch_deg);
    RCLCPP_INFO(this->get_logger(), "LIDAR mounting yaw: %f degrees", lidar_mount_yaw_deg);
    RCLCPP_INFO(this->get_logger(), "LIDAR mounting offset X: %f meters", lidar_mount_offset_x);
    RCLCPP_INFO(this->get_logger(), "LIDAR mounting offset Y: %f meters", lidar_mount_offset_y);
    RCLCPP_INFO(this->get_logger(), "LIDAR mounting offset Z: %f meters", lidar_mount_offset_z);

    const double mf_timeout = this->get_parameter("mf_timeout").as_double();
    const double timestamp_tolerance = this->get_parameter("timestamp_tolerance").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp tolerance: %f seconds", timestamp_tolerance);
    RCLCPP_INFO(this->get_logger(), "Using message filter timeout: %f seconds", mf_timeout);

    if (timestamp_tolerance >= mf_timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "mf_timeout is smaller than timestamp_tolerance! Exiting...");
      return;
    }

    sor_mean_k_ = this->get_parameter("sor_mean_k").as_int();
    sor_std_dev_mult_ = this->get_parameter("sor_std_dev_mult").as_double();
    RCLCPP_INFO(this->get_logger(), "Using SOR filter mean k parameter: %d", sor_mean_k_);
    RCLCPP_INFO(this->get_logger(), "Using SOR filter std dev mult parameter: %f", sor_std_dev_mult_);

    if (sor_mean_k_ < 0 || sor_std_dev_mult_ < 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid SOR filter parameters! Exiting...");
      return;
    }

    enable_bag_ = this->get_parameter("enable_bag").as_bool();
    if (enable_bag_)
    {
      get_current_timestamp(time_format_);
      bag_name_ = bag_dir_ + "lidar_bag_" + std::string(time_format_);

      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      RCLCPP_INFO(this->get_logger(), "Recording to bag file: %s", bag_name_.c_str());
    }

    led_strip_.clear();

    auto qos = rclcpp::SensorDataQoS();


    // publishers
    sync_slice_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(sync_slice_point_cloud_topic_, qos);

    // tf
    boost::qvm::quat<double> q_x = boost::qvm::rotx_quat(lidar_mount_roll_deg * M_PI / 180.0);
    boost::qvm::quat<double> q_y = boost::qvm::roty_quat(lidar_mount_pitch_deg * M_PI / 180.0);
    boost::qvm::quat<double> q_z = boost::qvm::rotz_quat(lidar_mount_yaw_deg * M_PI / 180.0);
    boost::qvm::quat<double> q_lidar = q_z * q_y * q_x;
    boost::qvm::normalize(q_lidar);

    drone_lidar_tf_.header.frame_id = drone_link_;
    drone_lidar_tf_.child_frame_id = lidar_link_;
    drone_lidar_tf_.transform.translation.x = lidar_mount_offset_x;
    drone_lidar_tf_.transform.translation.y = lidar_mount_offset_y;
    drone_lidar_tf_.transform.translation.z = lidar_mount_offset_z;
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
    mf_slice_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());

    mf_slice_scan_tf2_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      mf_slice_scan_sub_, *tf_buffer_, drone_link_, 10,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      tf2::durationFromSec(mf_timeout)
    );
    mf_slice_scan_tf2_->setTolerance(rclcpp::Duration::from_seconds(timestamp_tolerance));
    mf_slice_scan_tf2_->registerCallback(&LidarMapper::sync_slice_scan_callback, this);

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

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      qos,
      std::bind(&LidarMapper::odom_callback, this, std::placeholders::_1)
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
  const std::string odom_topic_ = "mavros/local_position/odom";
  const std::string pose_topic_ = "/mavros/local_position/pose";
  const std::string sync_slice_point_cloud_topic_ = "/sync_slice_point_cloud";

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  const std::string world_link_ = "map";
  const std::string drone_link_ = "base_link";
  const std::string lidar_link_ = "ldlidar_link";

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_slice_point_cloud_pub_;

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_slice_scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> mf_slice_scan_tf2_;

  mavros_msgs::msg::RCIn::SharedPtr last_rc_msg_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  sensor_msgs::msg::Imu::SharedPtr last_orientation_msg_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_;

  laser_geometry::LaserProjection laser_projector_;
  geometry_msgs::msg::TransformStamped drone_lidar_tf_;

  int sor_mean_k_ = 50;
  double sor_std_dev_mult_ = 1.0;

  bool enable_bag_ = false;
  const char* home_ = std::getenv("HOME");
  const std::string home_dir_ = home_ ? std::string(home_) : std::string(".");

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool writer_opened_ = false;
  const std::string bag_dir_ = home_dir_ + "/ros2_ws/src/lidar_mapper/lidar_bags/";
  std::string bag_name_;

  char time_format_[20];
  const size_t script_start_channel_ = 7;
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
   * @param slice_scan_msg Laser slice scan message pointer
   */
  void sync_slice_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr slice_scan_msg)
  {
    if (script_start_state_ != 2)
      return;

    rclcpp::Duration scan_duration = rclcpp::Duration::from_seconds(slice_scan_msg->ranges.size() * slice_scan_msg->time_increment);
    rclcpp::Time end_of_scan = rclcpp::Time(slice_scan_msg->header.stamp) + scan_duration;
    if (!tf_buffer_->canTransform(drone_link_,
                                  slice_scan_msg->header.frame_id,
                                  end_of_scan,
                                  rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF data to cover the entire scan duration...");
      return;
    }

    sensor_msgs::msg::PointCloud2 sync_slice_point_cloud_msg;
    laser_projector_.transformLaserScanToPointCloud(drone_link_, *slice_scan_msg, sync_slice_point_cloud_msg, *tf_buffer_);

    // point cloud filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr sync_pcl_slice(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(sync_slice_point_cloud_msg, *sync_pcl_slice);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_sync_pcl_slice(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(sync_pcl_slice);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_std_dev_mult_);
    sor.filter(*filtered_sync_pcl_slice);

    sensor_msgs::msg::PointCloud2 filtered_sync_slice_point_cloud_msg;
    pcl::toROSMsg(*filtered_sync_pcl_slice, filtered_sync_slice_point_cloud_msg);
    filtered_sync_slice_point_cloud_msg.header = sync_slice_point_cloud_msg.header;

    sync_slice_point_cloud_pub_->publish(filtered_sync_slice_point_cloud_msg);

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_point_cloud_msg = std::make_shared<rclcpp::SerializedMessage>();
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> point_cloud_serialization;
      point_cloud_serialization.serialize_message(&filtered_sync_slice_point_cloud_msg, serialized_point_cloud_msg.get());
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

          last_odom_msg_ = nullptr;
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
   * @brief Callback for odometry data
   * 
   * @param odom_msg Odometry message pointer
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    last_odom_msg_ = odom_msg;

    if (enable_bag_ && writer_opened_)
    {
      auto serialized_odom_msg = std::make_shared<rclcpp::SerializedMessage>();

      rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
      odom_serialization.serialize_message(odom_msg.get(), serialized_odom_msg.get());
      writer_->write(serialized_odom_msg, odom_topic_,
                     "nav_msgs/msg/Odometry", odom_msg->header.stamp);
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
