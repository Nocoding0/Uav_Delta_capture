#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class UavBridgeNode : public rclcpp::Node
{
public:
  UavBridgeNode()
  : Node("uav_bridge_node"),
    serial_port_(declare_parameter<std::string>("serial_port", "/dev/ttySTM0")),
    baudrate_(declare_parameter<int>("baudrate", 921600)),
    target_frame_(declare_parameter<std::string>("target_frame", "delta_base_link")),
    camera_frame_(declare_parameter<std::string>("camera_frame", "camera_optical_frame")),
    local_pose_topic_(declare_parameter<std::string>("local_pose_topic", "/mavros/local_position/pose")),
    imu_topic_(declare_parameter<std::string>("imu_topic", "/mavros/imu/data")),
    vision_offset_topic_(declare_parameter<std::string>("vision_offset_topic", "vision/target_offset")),
    target_point_topic_(declare_parameter<std::string>("target_point_topic", "target_point")),
    local_attitude_topic_(declare_parameter<std::string>("local_attitude_topic", "uav_bridge/local_attitude")),
    imu_attitude_topic_(declare_parameter<std::string>("imu_attitude_topic", "uav_bridge/imu_attitude")),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto best_effort = rclcpp::SensorDataQoS();

    local_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      local_pose_topic_, best_effort,
      std::bind(&UavBridgeNode::localPoseCallback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, best_effort,
      std::bind(&UavBridgeNode::imuCallback, this, std::placeholders::_1));

    vision_offset_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      vision_offset_topic_, 20,
      std::bind(&UavBridgeNode::visionOffsetCallback, this, std::placeholders::_1));

    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(target_point_topic_, 20);
    local_attitude_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(local_attitude_topic_, 20);
    imu_attitude_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(imu_attitude_topic_, 50);

    diag_timer_ = create_wall_timer(1000ms, std::bind(&UavBridgeNode::diagTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "uav_bridge started serial_port=%s baudrate=%d target_frame=%s camera_frame=%s local_pose_topic=%s imu_topic=%s",
      serial_port_.c_str(), baudrate_, target_frame_.c_str(), camera_frame_.c_str(),
      local_pose_topic_.c_str(), imu_topic_.c_str());
  }

private:
  geometry_msgs::msg::Vector3Stamped makeAttitudeMsg(
    const geometry_msgs::msg::Quaternion & orientation,
    const rclcpp::Time & stamp,
    const std::string & frame_id) const
  {
    tf2::Quaternion q(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w);
    q.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Vector3Stamped attitude;
    attitude.header.stamp = stamp;
    attitude.header.frame_id = frame_id;
    attitude.vector.x = roll;
    attitude.vector.y = pitch;
    attitude.vector.z = yaw;
    return attitude;
  }

  void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_local_pose_ = *msg;
    has_local_pose_ = true;
    latest_local_attitude_ = makeAttitudeMsg(
      msg->pose.orientation,
      msg->header.stamp,
      msg->header.frame_id);
    local_attitude_pub_->publish(latest_local_attitude_);

    RCLCPP_DEBUG(
      get_logger(),
      "%s: position=[%.3f, %.3f, %.3f] attitude_rpy=[%.3f, %.3f, %.3f]",
      local_pose_topic_.c_str(),
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      latest_local_attitude_.vector.x, latest_local_attitude_.vector.y,
      latest_local_attitude_.vector.z);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_attitude_ = makeAttitudeMsg(
      msg->orientation,
      msg->header.stamp,
      msg->header.frame_id);
    imu_attitude_pub_->publish(latest_imu_attitude_);
    has_imu_ = true;
  }

  void visionOffsetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped in_camera;
    in_camera.header = msg->header;
    in_camera.point = msg->point;
    if (in_camera.header.frame_id.empty()) {
      in_camera.header.frame_id = camera_frame_;
    }

    try {
      auto in_arm = tf_buffer_.transform(in_camera, target_frame_, tf2::durationFromSec(0.05));
      target_pub_->publish(in_arm);
      RCLCPP_DEBUG(
        get_logger(),
        "%s in %s: [%.3f, %.3f, %.3f]",
        target_point_topic_.c_str(), target_frame_.c_str(), in_arm.point.x, in_arm.point.y,
        in_arm.point.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF transform failed: %s", ex.what());
    }
  }

  void diagTimerCallback()
  {
    if (!has_local_pose_) {
      RCLCPP_INFO(get_logger(), "waiting %s ...", local_pose_topic_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "FCU pose ok: altitude=%.3f local_rpy(rad)=[%.3f, %.3f, %.3f] imu=%s",
      latest_local_pose_.pose.position.z,
      latest_local_attitude_.vector.x,
      latest_local_attitude_.vector.y,
      latest_local_attitude_.vector.z,
      has_imu_ ? "OK" : "WAIT");
  }

  std::string serial_port_;
  int baudrate_;
  std::string target_frame_;
  std::string camera_frame_;
  std::string local_pose_topic_;
  std::string imu_topic_;
  std::string vision_offset_topic_;
  std::string target_point_topic_;
  std::string local_attitude_topic_;
  std::string imu_attitude_topic_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vision_offset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr local_attitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_attitude_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseStamped latest_local_pose_;
  geometry_msgs::msg::Vector3Stamped latest_local_attitude_;
  geometry_msgs::msg::Vector3Stamped latest_imu_attitude_;
  bool has_local_pose_{false};
  bool has_imu_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UavBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
