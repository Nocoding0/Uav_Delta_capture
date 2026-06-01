#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "uav_delta_msgs/msg/fcu_state.hpp"
#include "uav_delta_msgs/msg/uwb_status.hpp"
#include "uav_delta_msgs/srv/flight_command.hpp"

using namespace std::chrono_literals;

class MissionSequencerNode : public rclcpp::Node
{
public:
  enum class Phase {
    PRE_CHECK,
    ARMING,
    TAKEOFF,
    WAIT_ALTITUDE,
    NAVIGATING,
    HOVER_REACHED,
    LANDING,
    DONE,
    FAILSAFE
  };

  MissionSequencerNode()
  : Node("mission_sequencer_node"),
    fcu_state_topic_(declare_parameter<std::string>("fcu_state_topic", "fcu_state")),
    uwb_status_topic_(declare_parameter<std::string>("uwb_status_topic", "uwb/status")),
    target_reached_topic_(declare_parameter<std::string>("target_reached_topic", "uwb/target_reached")),
    fcu_link_status_topic_(declare_parameter<std::string>("fcu_link_status_topic", "fcu_link/status")),
    flight_command_service_(declare_parameter<std::string>("flight_command_service", "flight_command")),
    mission_state_topic_(declare_parameter<std::string>("mission_state_topic", "mission/state")),
    mission_event_topic_(declare_parameter<std::string>("mission_event_topic", "mission/event")),
    takeoff_altitude_(declare_parameter<double>("takeoff_altitude", 1.5)),
    altitude_tolerance_(declare_parameter<double>("altitude_tolerance", 0.2)),
    pre_check_rate_hz_(declare_parameter<double>("pre_check_rate_hz", 2.0)),
    low_battery_pct_(declare_parameter<double>("low_battery_pct", 20.0)),
    uwb_signal_loss_timeout_sec_(declare_parameter<double>("uwb_signal_loss_timeout_sec", 3.0)),
    auto_land_on_failsafe_(declare_parameter<bool>("auto_land_on_failsafe", true))
  {
    takeoff_altitude_ = std::max(0.5, takeoff_altitude_);
    altitude_tolerance_ = std::max(0.05, altitude_tolerance_);
    pre_check_rate_hz_ = std::max(0.5, pre_check_rate_hz_);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // FCU state subscription
    fcu_sub_ = create_subscription<uav_delta_msgs::msg::FcuState>(
      fcu_state_topic_, 10,
      [this](const uav_delta_msgs::msg::FcuState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_fcu_ = msg;
        has_fcu_ = true;
      });

    // UWB status subscription
    uwb_sub_ = create_subscription<uav_delta_msgs::msg::UwbStatus>(
      uwb_status_topic_, 10,
      [this](const uav_delta_msgs::msg::UwbStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_uwb_ = msg;
        has_uwb_ = true;
        last_uwb_time_ = now();
      });

    // UWB target reached subscription
    reached_sub_ = create_subscription<std_msgs::msg::Bool>(
      target_reached_topic_, 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        target_reached_ = msg->data;
      });

    // FCU link status subscription
    link_sub_ = create_subscription<std_msgs::msg::String>(
      fcu_link_status_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        fcu_link_status_ = msg->data;
      });

    // Flight command service client
    cmd_client_ = create_client<uav_delta_msgs::srv::FlightCommand>(
      flight_command_service_, rmw_qos_profile_default, cb_group_);

    // Publishers
    state_pub_ = create_publisher<std_msgs::msg::String>(mission_state_topic_, 10);
    event_pub_ = create_publisher<std_msgs::msg::String>(mission_event_topic_, 10);

    // Main tick timer
    const auto period = std::chrono::duration<double>(1.0 / pre_check_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MissionSequencerNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "mission_sequencer started: takeoff=%.1fm alt_tol=%.2f low_bat=%.0f%%",
      takeoff_altitude_, altitude_tolerance_, low_battery_pct_);
  }

private:
  // ── State machine tick ─────────────────────────────────────────────

  void tick()
  {
    switch (phase_) {
      case Phase::PRE_CHECK:     tickPreCheck(); break;
      case Phase::ARMING:        tickArming(); break;
      case Phase::TAKEOFF:       tickTakeoff(); break;
      case Phase::WAIT_ALTITUDE: tickWaitAltitude(); break;
      case Phase::NAVIGATING:    tickNavigating(); break;
      case Phase::HOVER_REACHED: tickHoverReached(); break;
      case Phase::LANDING:       tickLanding(); break;
      case Phase::DONE:          break;
      case Phase::FAILSAFE:      tickFailsafe(); break;
    }

    publishState();
  }

  // ── Phase handlers ─────────────────────────────────────────────────

  void tickPreCheck()
  {
    std::lock_guard<std::mutex> lk(state_mtx_);

    if (!has_fcu_ || !last_fcu_->connected) {
      return;  // still waiting
    }
    if (!last_fcu_->estimator_ok) {
      return;  // EKF not ready
    }
    if (!has_uwb_ || !last_uwb_->signal_valid) {
      return;  // UWB not ready
    }

    RCLCPP_INFO(get_logger(), "Pre-check passed: FCU ok, EKF ok, UWB signal valid");
    publishEvent("pre_check_passed");
    transitionTo(Phase::ARMING);
  }

  void tickArming()
  {
    if (arming_in_progress_) {
      return;  // waiting for service response
    }

    arming_in_progress_ = true;
    callFlightCommand(
      uav_delta_msgs::srv::FlightCommand::Request::CMD_ARM, 0.0f,
      [this](bool success, const std::string & msg) {
        arming_in_progress_ = false;
        if (success) {
          RCLCPP_INFO(get_logger(), "Arm successful");
          publishEvent("armed");
          transitionTo(Phase::TAKEOFF);
        } else {
          RCLCPP_WARN(get_logger(), "Arm failed: %s, retrying...", msg.c_str());
          // will retry on next tick
        }
      });
  }

  void tickTakeoff()
  {
    if (takeoff_in_progress_) {
      return;
    }

    takeoff_in_progress_ = true;
    callFlightCommand(
      uav_delta_msgs::srv::FlightCommand::Request::CMD_TAKEOFF,
      static_cast<float>(takeoff_altitude_),
      [this](bool success, const std::string & msg) {
        takeoff_in_progress_ = false;
        if (success) {
          RCLCPP_INFO(get_logger(), "Takeoff command accepted (%.1fm)", takeoff_altitude_);
          publishEvent("takeoff_accepted");
          transitionTo(Phase::WAIT_ALTITUDE);
        } else {
          RCLCPP_ERROR(get_logger(), "Takeoff failed: %s", msg.c_str());
          transitionTo(Phase::FAILSAFE);
        }
      });
  }

  void tickWaitAltitude()
  {
    if (!checkFcuAndLink()) {
      return;
    }

    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!has_fcu_) {
      return;
    }

    float alt = last_fcu_->local_z;
    if (std::fabs(alt - takeoff_altitude_) <= altitude_tolerance_) {
      RCLCPP_INFO(get_logger(), "Altitude reached: %.2fm (target %.2fm)", alt, takeoff_altitude_);
      publishEvent("altitude_reached");
      transitionTo(Phase::NAVIGATING);
    }
  }

  void tickNavigating()
  {
    if (!checkFcuAndLink()) {
      return;
    }

    // Check UWB signal
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (has_uwb_) {
        double age = (now() - last_uwb_time_).seconds();
        if (age > uwb_signal_loss_timeout_sec_) {
          RCLCPP_ERROR(get_logger(), "UWB signal lost (%.1fs > %.1fs)", age, uwb_signal_loss_timeout_sec_);
          publishEvent("uwb_signal_lost");
          transitionTo(Phase::FAILSAFE);
          return;
        }
      }
    }

    if (target_reached_) {
      RCLCPP_INFO(get_logger(), "Target reached — hovering");
      publishEvent("target_reached");
      transitionTo(Phase::HOVER_REACHED);
    }
  }

  void tickHoverReached()
  {
    if (!checkFcuAndLink()) {
      return;
    }

    // Hover reached — publish event once, then wait.
    // Downstream (grasp system) should call a service or publish an event
    // to signal completion. For now, we just hold position.
    // TODO: add grasp trigger logic here (call SetArmStatus service)
  }

  void tickLanding()
  {
    if (landing_in_progress_) {
      // Wait for disarm
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (has_fcu_ && !last_fcu_->armed) {
        RCLCPP_INFO(get_logger(), "Landing complete, disarmed");
        publishEvent("mission_complete");
        transitionTo(Phase::DONE);
      }
      return;
    }

    landing_in_progress_ = true;
    callFlightCommand(
      uav_delta_msgs::srv::FlightCommand::Request::CMD_MODE_LAND, 0.0f,
      [this](bool success, const std::string & msg) {
        if (success) {
          RCLCPP_INFO(get_logger(), "LAND mode set");
          publishEvent("landing");
        } else {
          RCLCPP_ERROR(get_logger(), "LAND failed: %s", msg.c_str());
          // Try again
          landing_in_progress_ = false;
        }
      });
  }

  void tickFailsafe()
  {
    if (!auto_land_on_failsafe_) {
      return;
    }

    if (failsafe_land_sent_) {
      // Wait for disarm
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (has_fcu_ && !last_fcu_->armed) {
        RCLCPP_INFO(get_logger(), "Failsafe landing complete, disarmed");
        publishEvent("failsafe_landed");
        transitionTo(Phase::DONE);
      }
      return;
    }

    failsafe_land_sent_ = true;
    callFlightCommand(
      uav_delta_msgs::srv::FlightCommand::Request::CMD_MODE_LAND, 0.0f,
      [this](bool success, const std::string & msg) {
        RCLCPP_WARN(get_logger(), "Failsafe LAND: %s", success ? "accepted" : msg.c_str());
      });
  }

  // ── Safety checks ──────────────────────────────────────────────────

  bool checkFcuAndLink()
  {
    // Check link status
    if (fcu_link_status_ == "LOST") {
      RCLCPP_ERROR(get_logger(), "FCU link LOST");
      publishEvent("fcu_link_lost");
      transitionTo(Phase::FAILSAFE);
      return false;
    }

    // Check battery
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (has_fcu_ && last_fcu_->remaining > 0.0f &&
        last_fcu_->remaining < low_battery_pct_ / 100.0f)
    {
      RCLCPP_ERROR(get_logger(), "Battery low: %.0f%%", last_fcu_->remaining * 100.0f);
      publishEvent("low_battery");
      transitionTo(Phase::FAILSAFE);
      return false;
    }

    return true;
  }

  // ── Helpers ────────────────────────────────────────────────────────

  void transitionTo(Phase new_phase)
  {
    RCLCPP_INFO(get_logger(), "Phase: %s -> %s", phaseName(phase_), phaseName(new_phase));
    phase_ = new_phase;
  }

  void publishState()
  {
    std_msgs::msg::String msg;
    msg.data = phaseName(phase_);
    state_pub_->publish(msg);
  }

  void publishEvent(const std::string & event)
  {
    std_msgs::msg::String msg;
    msg.data = event;
    event_pub_->publish(msg);
  }

  void callFlightCommand(
    int command, float param,
    std::function<void(bool, const std::string &)> callback)
  {
    if (!cmd_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "FlightCommand service not ready");
      if (callback) {
        callback(false, "service not ready");
      }
      return;
    }

    auto req = std::make_shared<uav_delta_msgs::srv::FlightCommand::Request>();
    req->command = command;
    req->param = param;

    cmd_client_->async_send_request(req,
      [this, callback](rclcpp::Client<uav_delta_msgs::srv::FlightCommand>::SharedFuture future) {
        auto result = future.get();
        if (callback) {
          callback(result->success, result->message);
        }
      });
  }

  static const char * phaseName(Phase p)
  {
    switch (p) {
      case Phase::PRE_CHECK:     return "PRE_CHECK";
      case Phase::ARMING:        return "ARMING";
      case Phase::TAKEOFF:       return "TAKEOFF";
      case Phase::WAIT_ALTITUDE: return "WAIT_ALTITUDE";
      case Phase::NAVIGATING:    return "NAVIGATING";
      case Phase::HOVER_REACHED: return "HOVER_REACHED";
      case Phase::LANDING:       return "LANDING";
      case Phase::DONE:          return "DONE";
      case Phase::FAILSAFE:      return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  // Parameters
  std::string fcu_state_topic_;
  std::string uwb_status_topic_;
  std::string target_reached_topic_;
  std::string fcu_link_status_topic_;
  std::string flight_command_service_;
  std::string mission_state_topic_;
  std::string mission_event_topic_;
  double takeoff_altitude_;
  double altitude_tolerance_;
  double pre_check_rate_hz_;
  double low_battery_pct_;
  double uwb_signal_loss_timeout_sec_;
  bool auto_land_on_failsafe_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Subscribers
  rclcpp::Subscription<uav_delta_msgs::msg::FcuState>::SharedPtr fcu_sub_;
  rclcpp::Subscription<uav_delta_msgs::msg::UwbStatus>::SharedPtr uwb_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr link_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;

  // Service client
  rclcpp::Client<uav_delta_msgs::srv::FlightCommand>::SharedPtr cmd_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  Phase phase_{Phase::PRE_CHECK};
  std::mutex state_mtx_;
  uav_delta_msgs::msg::FcuState::SharedPtr last_fcu_;
  uav_delta_msgs::msg::UwbStatus::SharedPtr last_uwb_;
  bool has_fcu_{false};
  bool has_uwb_{false};
  rclcpp::Time last_uwb_time_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> target_reached_{false};
  std::string fcu_link_status_{"INIT"};

  // Async operation flags
  bool arming_in_progress_{false};
  bool takeoff_in_progress_{false};
  bool landing_in_progress_{false};
  bool failsafe_land_sent_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionSequencerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
