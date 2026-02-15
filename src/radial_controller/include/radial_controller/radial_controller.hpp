#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mutex>

namespace radial_controller
{

class RadialController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

private:
  struct OdomState
  {
    bool valid{false};
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    double yaw{0.0};
  };

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

  double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  std::mutex odom_mtx_;
  OdomState odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr radial_error_pub_;
  std_msgs::msg::Float64 radial_error_msg_;

  // Parameters
  double r0_{1.2};
  double k_r_{10.0};
  double k_dr_{20.0};
  double k_t_{8.0};
  double target_x_{0.0};
  double target_y_{0.0};
  double orbit_omega_{0.6};
  double orbit_direction_{1.0};
  double Fmax_{4.0};
  double vehicle_mass_{15.0};
  double centripetal_gain_{1.0};
  double tangential_force_limit_{2.0};
  double tangential_suppress_error_{0.15};
  double tangential_outward_gate_error_{0.08};
  double tangential_outward_gate_vrad_{0.03};
  double radial_deadband_{0.03};
  double radial_velocity_deadband_{0.05};

  bool ifaces_ready_{false};
};

} // namespace radial_controller
