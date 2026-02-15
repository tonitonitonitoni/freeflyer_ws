#pragma once

#include <controller_interface/controller_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mutex>
#include <string>

namespace attitude_controller
{

class AttitudeController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  static double wrap_to_pi(double a);
  static double clamp(double v, double lo, double hi);

  std::mutex mtx_;
  bool odom_valid_{false};
  double rw_cmd_{0.0};  // -1, 0, +1
  int rw_dir_{0};       // -1, 0, +1
  bool attitude_ready_{false};
  double yaw_error_{0.0};
  double yaw_rate_{0.0};
  double desired_yaw_rate_{0.0};
  double desired_yaw_cmd_{0.0};
  bool desired_yaw_inited_{false};
  double last_guidance_t_{-1.0};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_t_{-1.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_err_pub_;

  bool ifaces_ready_{false};
  bool use_odom_twist_{true};
  bool enable_attitude_control_{true};

  double r0_{1.5};
  double v_tan_des_{0.1};
  double k_yaw_{5.0};
  double k_yaw_rate_{5.0};
  double yaw_deadband_{0.02};
  double yaw_on_threshold_{0.45};
  double yaw_off_threshold_{0.20};
  double yaw_rate_limit_{0.35};
  double yaw_rate_error_on_threshold_{0.015};
  double yaw_rate_error_off_threshold_{0.005};
  double yaw_surface_gain_{0.8};
  double yaw_surface_on_threshold_{0.02};
  double yaw_surface_off_threshold_{0.004};
  double yaw_ref_rate_limit_{0.15};
  bool use_geometric_yaw_rate_{true};
  double yaw_min_speed_for_control_{0.05};
  double attitude_enable_radial_tol_{0.12};
  double attitude_enable_vtan_tol_{0.12};
  double Tmax_{0.015};

  double target_x_{0.0};
  double target_y_{0.0};
};

}  // namespace attitude_controller
