#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "uav_delta_msgs/msg/fcu_state.hpp"
#include "uav_delta_msgs/msg/uwb_status.hpp"

using namespace std::chrono_literals;

class UwbNavigatorNode : public rclcpp::Node
{
public:
  UwbNavigatorNode()
  : Node("uwb_navigator_node"),
    serial_port_(declare_parameter<std::string>("serial_port", "/dev/ttyACM1")),
    serial_baud_(declare_parameter<int>("serial_baud", 115200)),
    uwb_frame_id_(declare_parameter<std::string>("uwb_frame_id", "uwb")),
    cmd_vel_topic_(declare_parameter<std::string>("cmd_vel_topic", "cmd_vel")),
    fcu_state_topic_(declare_parameter<std::string>("fcu_state_topic", "fcu_state")),
    uwb_status_topic_(declare_parameter<std::string>("uwb_status_topic", "uwb/status")),
    target_reached_topic_(declare_parameter<std::string>("target_reached_topic", "uwb/target_reached")),
    target_x_(declare_parameter<double>("target_x", 0.0)),
    target_y_(declare_parameter<double>("target_y", 0.0)),
    target_z_(declare_parameter<double>("target_z", 1.0)),
    kp_xy_(declare_parameter<double>("kp_xy", 0.8)),
    kp_z_(declare_parameter<double>("kp_z", 0.6)),
    max_vel_xy_(declare_parameter<double>("max_vel_xy", 0.8)),
    max_vel_z_(declare_parameter<double>("max_vel_z", 0.4)),
    deadband_radius_(declare_parameter<double>("deadband_radius", 0.10)),
    max_accel_xy_(declare_parameter<double>("max_accel_xy", 0.5)),
    max_accel_z_(declare_parameter<double>("max_accel_z", 0.3)),
    control_rate_hz_(declare_parameter<double>("control_rate_hz", 20.0)),
    signal_loss_timeout_sec_(declare_parameter<double>("signal_loss_timeout_sec", 0.5)),
    max_range_xy_(declare_parameter<double>("max_range_xy", 15.0)),
    max_range_z_(declare_parameter<double>("max_range_z", 5.0)),
    min_z_(declare_parameter<double>("min_z", 0.3)),
    status_rate_hz_(declare_parameter<double>("status_rate_hz", 10.0))
  {
    // Clamp parameters
    kp_xy_ = std::max(0.01, kp_xy_);
    kp_z_ = std::max(0.01, kp_z_);
    max_vel_xy_ = std::max(0.1, max_vel_xy_);
    max_vel_z_ = std::max(0.1, max_vel_z_);
    deadband_radius_ = std::max(0.0, deadband_radius_);
    control_rate_hz_ = std::max(1.0, control_rate_hz_);
    signal_loss_timeout_sec_ = std::max(0.05, signal_loss_timeout_sec_);

    // Publishers
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 20);
    status_pub_ = create_publisher<uav_delta_msgs::msg::UwbStatus>(uwb_status_topic_, 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>(target_reached_topic_, 1);

    // FCU state subscription
    fcu_sub_ = create_subscription<uav_delta_msgs::msg::FcuState>(
      fcu_state_topic_, 10,
      [this](const uav_delta_msgs::msg::FcuState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(fcu_mtx_);
        last_fcu_ = msg;
        has_fcu_ = true;
      });

    // Open serial port
    serial_fd_ = openSerial(serial_port_, serial_baud_);
    if (serial_fd_ < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open serial port %s — running in IDLE (no UWB data)",
        serial_port_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial port %s opened (fd=%d)", serial_port_.c_str(), serial_fd_);
      io_thread_ = std::thread(&UwbNavigatorNode::ioLoop, this);
    }

    // Control timer (20Hz)
    const auto ctrl_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    ctrl_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ctrl_period),
      std::bind(&UwbNavigatorNode::controlCallback, this));

    // Status timer (10Hz)
    const auto status_period = std::chrono::duration<double>(1.0 / status_rate_hz_);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
      std::bind(&UwbNavigatorNode::statusCallback, this));

    last_pos_time_ = now();
    last_vel_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "uwb_navigator started: target=(%.2f,%.2f,%.2f) kp_xy=%.2f kp_z=%.2f deadband=%.2f",
      target_x_, target_y_, target_z_, kp_xy_, kp_z_, deadband_radius_);
  }

  ~UwbNavigatorNode() override
  {
    // Publish zero velocity before shutdown
    if (vel_pub_->get_subscription_count() > 0) {
      auto zero = geometry_msgs::msg::TwistStamped();
      zero.header.stamp = now();
      zero.header.frame_id = uwb_frame_id_;
      vel_pub_->publish(zero);
    }

    running_ = false;
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }
  }

private:
  // ── Serial I/O thread ────────────────────────────────────────────────

  void ioLoop()
  {
    char buf[256];
    std::string line;

    while (running_) {
      int n = ::read(serial_fd_, buf, sizeof(buf) - 1);
      if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
          continue;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read error: %s", strerror(errno));
        std::this_thread::sleep_for(10ms);
        continue;
      }

      for (int i = 0; i < n; i++) {
        if (buf[i] == '\n') {
          parseLine(line);
          line.clear();
        } else if (buf[i] != '\r') {
          line += buf[i];
        }
      }
    }
  }

  void parseLine(const std::string & line)
  {
    // DWM1001 POS format: "POS,x,y,z" (mm)
    if (line.substr(0, 4) != "POS,") {
      return;
    }

    float x_mm, y_mm, z_mm;
    if (sscanf(line.c_str() + 4, "%f,%f,%f", &x_mm, &y_mm, &z_mm) != 3) {
      return;
    }

    UwbPosition pos;
    pos.x = x_mm * 0.001f;
    pos.y = y_mm * 0.001f;
    pos.z = z_mm * 0.001f;
    pos.time = now();

    std::lock_guard<std::mutex> lk(pos_mtx_);
    latest_pos_ = pos;
    has_pos_ = true;
  }

  // ── Control callback (20Hz) ──────────────────────────────────────────

  void controlCallback()
  {
    // Get FCU state
    bool fcu_ok = false;
    {
      std::lock_guard<std::mutex> lk(fcu_mtx_);
      if (has_fcu_ && last_fcu_->connected && last_fcu_->armed && last_fcu_->mode == "GUIDED") {
        fcu_ok = true;
      }
    }

    // Get UWB position
    UwbPosition pos;
    bool pos_valid = false;
    {
      std::lock_guard<std::mutex> lk(pos_mtx_);
      if (has_pos_) {
        pos = latest_pos_;
        double age = (now() - pos.time).seconds();
        if (age <= signal_loss_timeout_sec_) {
          // Range check
          float horiz_dist = std::sqrt(pos.x * pos.x + pos.y * pos.y);
          if (horiz_dist <= max_range_xy_ && pos.z >= min_z_ && pos.z <= max_range_z_) {
            pos_valid = true;
          }
        }
      }
    }

    // Update nav state
    if (!pos_valid) {
      nav_state_ = has_pos_ ? "SIGNAL_LOST" : "NO_SIGNAL";
      prev_reached_ = false;
    } else if (!fcu_ok) {
      nav_state_ = "FCU_NOT_READY";
      prev_reached_ = false;
    } else {
      // Compute error in UWB frame
      float dx = target_x_ - pos.x;
      float dy = target_y_ - pos.y;
      float dz = target_z_ - pos.z;
      float horiz_err = std::sqrt(dx * dx + dy * dy);
      float dist = std::sqrt(horiz_err * horiz_err + dz * dz);

      if (dist <= deadband_radius_) {
        nav_state_ = "HOVER_REACHED";
        if (!prev_reached_) {
          auto msg = std_msgs::msg::Bool();
          msg.data = true;
          reached_pub_->publish(msg);
          RCLCPP_INFO(get_logger(), "Target reached (%.2fm away)", dist);
        }
        prev_reached_ = true;
        publishZeroVel();
      } else {
        nav_state_ = "NAVIGATING";
        if (prev_reached_) {
          auto msg = std_msgs::msg::Bool();
          msg.data = false;
          reached_pub_->publish(msg);
          prev_reached_ = false;
        }

        // P-controller
        float vx = kp_xy_ * dx;
        float vy = kp_xy_ * dy;
        float vz = kp_z_ * dz;

        // Clamp to max velocity
        float h_speed = std::sqrt(vx * vx + vy * vy);
        if (h_speed > max_vel_xy_) {
          float scale = max_vel_xy_ / h_speed;
          vx *= scale;
          vy *= scale;
        }
        vz = clamp(vz, -max_vel_z_, max_vel_z_);

        // Rate limiting
        auto now_time = now();
        double dt = (now_time - last_vel_time_).seconds();
        last_vel_time_ = now_time;
        if (dt > 0.0 && dt < 1.0) {
          float max_dv_xy = max_accel_xy_ * dt;
          float max_dv_z = max_accel_z_ * dt;
          vx = rateLimit(prev_vx_, vx, max_dv_xy);
          vy = rateLimit(prev_vy_, vy, max_dv_xy);
          vz = rateLimit(prev_vz_, vz, max_dv_z);
        }

        // Publish velocity
        auto vel_msg = geometry_msgs::msg::TwistStamped();
        vel_msg.header.stamp = now_time;
        vel_msg.header.frame_id = uwb_frame_id_;
        vel_msg.twist.linear.x = vx;
        vel_msg.twist.linear.y = vy;
        vel_msg.twist.linear.z = vz;
        vel_pub_->publish(vel_msg);

        prev_vx_ = vx;
        prev_vy_ = vy;
        prev_vz_ = vz;
      }

      last_distance_ = dist;
    }

    // Not navigating → zero velocity
    if (nav_state_ != "NAVIGATING") {
      publishZeroVel();
      prev_vx_ = 0.0f;
      prev_vy_ = 0.0f;
      prev_vz_ = 0.0f;
    }
  }

  // ── Status callback (10Hz) ───────────────────────────────────────────

  void statusCallback()
  {
    UwbPosition pos;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lk(pos_mtx_);
      if (has_pos_) {
        pos = latest_pos_;
        double age = (now() - pos.time).seconds();
        valid = (age <= signal_loss_timeout_sec_);
      }
    }

    auto msg = uav_delta_msgs::msg::UwbStatus();
    msg.header.stamp = now();
    msg.header.frame_id = uwb_frame_id_;
    msg.x = pos.x;
    msg.y = pos.y;
    msg.z = pos.z;
    msg.anchor_count = 0;  // DWM1001 POS doesn't report this
    msg.quality = valid ? 1.0f : 0.0f;
    msg.signal_valid = valid;
    msg.age_sec = valid ? static_cast<float>((now() - pos.time).seconds()) : -1.0f;
    msg.nav_state = nav_state_;
    msg.distance_to_target = last_distance_;

    status_pub_->publish(msg);
  }

  // ── Helpers ──────────────────────────────────────────────────────────

  void publishZeroVel()
  {
    auto vel = geometry_msgs::msg::TwistStamped();
    vel.header.stamp = now();
    vel.header.frame_id = uwb_frame_id_;
    vel_pub_->publish(vel);
  }

  static float clamp(float v, float lo, float hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  static float rateLimit(float prev, float target, float max_delta)
  {
    float delta = target - prev;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    return prev + delta;
  }

  int openSerial(const std::string & port, int baud)
  {
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      return -1;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
      ::close(fd);
      return -1;
    }

    // Input baud
    speed_t b = B115200;
    switch (baud) {
      case 9600: b = B9600; break;
      case 38400: b = B38400; break;
      case 57600: b = B57600; break;
      case 115200: b = B115200; break;
      default: b = B115200; break;
    }
    cfsetispeed(&tty, b);
    cfsetospeed(&tty, b);

    // 8N1, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= (CLOCAL | CREAD);

    // Raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Blocking read, return after 1 char or 100ms timeout
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      ::close(fd);
      return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
  }

  struct UwbPosition {
    float x{0}, y{0}, z{0};
    rclcpp::Time time{0, 0, RCL_ROS_TIME};
  };

  // Parameters
  std::string serial_port_;
  int serial_baud_;
  std::string uwb_frame_id_;
  std::string cmd_vel_topic_;
  std::string fcu_state_topic_;
  std::string uwb_status_topic_;
  std::string target_reached_topic_;
  double target_x_, target_y_, target_z_;
  double kp_xy_, kp_z_;
  double max_vel_xy_, max_vel_z_;
  double deadband_radius_;
  double max_accel_xy_, max_accel_z_;
  double control_rate_hz_;
  double signal_loss_timeout_sec_;
  double max_range_xy_, max_range_z_;
  double min_z_;
  double status_rate_hz_;

  // Serial
  int serial_fd_{-1};
  std::thread io_thread_;
  std::atomic<bool> running_{true};

  // Position data (protected by pos_mtx_)
  std::mutex pos_mtx_;
  UwbPosition latest_pos_;
  bool has_pos_{false};

  // FCU state (protected by fcu_mtx_)
  std::mutex fcu_mtx_;
  uav_delta_msgs::msg::FcuState::SharedPtr last_fcu_;
  bool has_fcu_{false};

  // Publishers / subscribers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<uav_delta_msgs::msg::UwbStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::Subscription<uav_delta_msgs::msg::FcuState>::SharedPtr fcu_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Control state
  std::string nav_state_{"IDLE"};
  bool prev_reached_{false};
  float prev_vx_{0}, prev_vy_{0}, prev_vz_{0};
  rclcpp::Time last_pos_time_;
  rclcpp::Time last_vel_time_;
  float last_distance_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UwbNavigatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
