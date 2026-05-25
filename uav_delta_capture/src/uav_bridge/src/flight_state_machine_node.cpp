#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class FlightStateMachineNode : public rclcpp::Node
{
public:
  FlightStateMachineNode()
  : Node("flight_state_machine_node"),
    status_topic_(declare_parameter<std::string>("status_topic", "fcu_link/status")),
    state_topic_(declare_parameter<std::string>("state_topic", "uav_bridge/flight_state")),
    publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 5.0))
  {
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);

    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, 10,
      std::bind(&FlightStateMachineNode::statusCallback, this, std::placeholders::_1));

    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&FlightStateMachineNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "flight_state_machine started: status_topic=%s state_topic=%s",
      status_topic_.c_str(), state_topic_.c_str());
  }

private:
  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_link_status_ = msg->data;
  }

  void timerCallback()
  {
    if (state_ == "INIT") {
      state_ = "WAIT_FCU";
    }

    if (last_link_status_ == "OK") {
      state_ = "TRACKING";
    } else if (last_link_status_ == "LOST") {
      state_ = "FAILSAFE";
    } else if (last_link_status_ == "WAIT_FCU" && state_ != "FAILSAFE") {
      state_ = "WAIT_FCU";
    }

    if (state_ != last_published_state_) {
      RCLCPP_INFO(get_logger(), "flight state -> %s", state_.c_str());
      last_published_state_ = state_;
    }

    std_msgs::msg::String msg;
    msg.data = state_;
    state_pub_->publish(msg);
  }

  std::string status_topic_;
  std::string state_topic_;
  double publish_rate_hz_;

  std::string last_link_status_{"WAIT_FCU"};
  std::string state_{"INIT"};
  std::string last_published_state_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlightStateMachineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
