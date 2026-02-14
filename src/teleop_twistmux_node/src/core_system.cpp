#include <cstdint>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace
{
static double clamp1(const double v) noexcept
{
  if (v > 1.0) { return 1.0; }
  if (v < -1.0) { return -1.0; }
  return v;
}

static bool get_button(const sensor_msgs::msg::Joy & msg, const int index) noexcept
{
  if ((index < 0) || (static_cast<size_t>(index) >= msg.buttons.size())) { return false; }
  return (msg.buttons[static_cast<size_t>(index)] != 0);
}

static double get_axis(const sensor_msgs::msg::Joy & msg, const int index) noexcept
{
  if ((index < 0) || (static_cast<size_t>(index) >= msg.axes.size())) { return 0.0; }
  return static_cast<double>(msg.axes[static_cast<size_t>(index)]);
}
}  // namespace

class CoreSystemTeleop final : public rclcpp::Node
{
public:
  explicit CoreSystemTeleop(const rclcpp::NodeOptions & options)
  : rclcpp::Node("core_system", options)
  {
    // Topics
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // Rates / timeouts
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    joy_timeout_s_   = declare_parameter<double>("joy_timeout_s", 0.25);

    // Mapping
    axis_linear_x_   = declare_parameter<int>("axis_linear_x", 1);
    axis_angular_z_  = declare_parameter<int>("axis_angular_z", 0);
    deadman_button_  = declare_parameter<int>("deadman_button", 5);  // e.g., RB

    // Speed presets
    speed_buttons_        = declare_parameter<std::vector<int64_t>>("speed_buttons", std::vector<int64_t>{0, 1, 2, 3, 4, 5});
    linear_scales_        = declare_parameter<std::vector<double>>("linear_scales", std::vector<double>{0.10, 0.20, 0.35, 0.50, 0.75, 1.00});
    angular_scales_       = declare_parameter<std::vector<double>>("angular_scales", std::vector<double>{0.25, 0.45, 0.65, 0.80, 1.00, 1.00});
    default_speed_index_  = declare_parameter<int>("default_speed_index", 0);

    // Validate preset vectors
    const size_t n = speed_buttons_.size();
    if ((n == 0U) || (linear_scales_.size() != n) || (angular_scales_.size() != n))
    {
      RCLCPP_FATAL(get_logger(),
        "Invalid preset configuration: speed_buttons(%zu), linear_scales(%zu), angular_scales(%zu). Must all match and be non-zero.",
        speed_buttons_.size(), linear_scales_.size(), angular_scales_.size());
      throw std::runtime_error("Invalid preset configuration");
    }
    if (default_speed_index_ < 0) { default_speed_index_ = 0; }
    if (static_cast<size_t>(default_speed_index_) >= n) { default_speed_index_ = static_cast<int>(n - 1U); }

    // Clamp scales to sane bounds (you wanted max 1.0)
    for (size_t i = 0U; i < n; ++i)
    {
      if (linear_scales_[i] < 0.0)  { linear_scales_[i] = 0.0; }
      if (linear_scales_[i] > 1.0)  { linear_scales_[i] = 1.0; }
      if (angular_scales_[i] < 0.0) { angular_scales_[i] = 0.0; }
      if (angular_scales_[i] > 1.0) { angular_scales_[i] = 1.0; }
    }

    // Pub/Sub
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg)
      {
        last_joy_msg_ = *msg;
        last_joy_time_ = now();
      });

    // Publish timer
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / ((publish_rate_hz_ > 1e-3) ? publish_rate_hz_ : 20.0)));
    timer_ = create_wall_timer(period, [this]() { this->on_timer(); });

    RCLCPP_INFO(get_logger(), "core_system teleop started: joy='%s' -> cmd_vel='%s' @ %.2f Hz",
      joy_topic_.c_str(), cmd_vel_topic_.c_str(), publish_rate_hz_);
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::Twist out{};
    const rclcpp::Time t_now = now();

    // If we haven't received /joy recently, output zero (fail-safe)
    const double age_s = (t_now - last_joy_time_).seconds();
    if (age_s > joy_timeout_s_)
    {
      cmd_pub_->publish(out);
      return;
    }

    // Deadman
    if (!get_button(last_joy_msg_, deadman_button_))
    {
      cmd_pub_->publish(out);
      return;
    }

    // Select preset: first pressed wins by order in speed_buttons_
    // (You can order speed_buttons_ by priority if you want.)
    int idx = default_speed_index_;
    for (size_t i = 0U; i < speed_buttons_.size(); ++i)
    {
      if (get_button(last_joy_msg_, static_cast<int>(speed_buttons_[i])))
      {
        idx = static_cast<int>(i);
        break;
      }
    }

    const double lin_in = clamp1(get_axis(last_joy_msg_, axis_linear_x_));
    const double ang_in = clamp1(get_axis(last_joy_msg_, axis_angular_z_));

    out.linear.x  = lin_in * linear_scales_[static_cast<size_t>(idx)];
    out.angular.z = ang_in * angular_scales_[static_cast<size_t>(idx)];

    // Everything else explicitly zero (rover-safe)
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;

    cmd_pub_->publish(out);
  }

  // Parameters
  std::string joy_topic_;
  std::string cmd_vel_topic_;
  double publish_rate_hz_{20.0};
  double joy_timeout_s_{0.25};

  int axis_linear_x_{1};
  int axis_angular_z_{0};
  int deadman_button_{5};

  std::vector<int64_t> speed_buttons_;
  std::vector<double> linear_scales_;
  std::vector<double> angular_scales_;
  int default_speed_index_{0};

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  sensor_msgs::msg::Joy last_joy_msg_{};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoreSystemTeleop>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}

