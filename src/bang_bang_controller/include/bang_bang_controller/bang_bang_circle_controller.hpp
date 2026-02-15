#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mutex>
#include <string>

namespace bang_bang_controller
{

struct OdomState
{
  bool valid{false};
  rclcpp::Time stamp;

  // Pose
  double x{0.0}, y{0.0}, yaw{0.0};

  // Twist (world frame)
  double vx{0.0}, vy{0.0};

  // Yaw rate (body/world Z axis is same in planar)
  double yaw_rate{0.0};

  // Whether odom twist was interpreted as body-frame and rotated to world
  bool twist_was_body{false};
};

class BangBangCircleController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // --- ROS I/O ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex odom_mtx_;
  OdomState odom_;

  // Optional debug publishers (match your Python topics)
  bool enable_debug_{true};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_radial_err_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_yaw_err_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fx_w_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fy_w_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;

  // --- Parameters (mirrors Python) ---
  // Orbit guidance
  double r0_{1.5};
  double v_tan_des_{0.1};
  double k_r_{5.0};
  double k_dr_{2.7};
  double k_ir_{0.35};
  double radial_i_limit_{0.25};
  double radial_i_zone_{0.25};
  double radial_i_leak_{0.25};
  double k_t_{1.0};
  double k_cent_{1.0};
  bool enable_tangential_control_{true};
  bool enable_centripetal_ff_{true};
  bool enable_radial_integral_{true};
  bool use_odom_twist_{true};
  double mass_ {15.0};

  // Attitude control
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
  
  bool desired_yaw_unwrap_inited_{false};
  double desired_yaw_unwrapped_{0.0};

  // Actuator limits
  double Fmax_{2.0};   // thruster_force
  double Tmax_{0.015}; // reaction_torque (kept for parity; rw is bang-bang)
  bool enable_attitude_control_{true};

  // Target
  double target_x_{0.0};
  double target_y_{0.0};

  // --- Controller internal state ---
  double fx_cmd_body_{0.0};
  double fy_cmd_body_{0.0};
  double tau_cmd_{0.0};  // computed for parity/debug; RW uses bang-bang
  double radial_error_int_{0.0};

  // Yaw command shaping
  bool desired_yaw_inited_{false};
  double desired_yaw_cmd_{0.0};
  double desired_yaw_rate_{0.0};


  // RW bang-bang state
  int rw_dir_{0}; // -1: cw, +1: ccw, 0: off
  bool attitude_ready_{false};

  // Safety / freshness
  double odom_timeout_{0.2}; // seconds; if odom stale -> neutral
  bool require_fresh_odom_{true};

  // --- Helpers ---
  static double wrap_to_pi(double a);
  static double clamp(double v, double lo, double hi);
  static double hypot2(double a, double b);

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Split responsibilities
  void compute_guidance_and_attitude(const OdomState & s, double dt);
  void compute_rw_switching(); // uses yaw_error, desired_yaw_rate, yaw_rate, etc.
  void write_actuators();      // writes command interfaces

  // cached errors for RW logic
  double yaw_error_{0.0};
  double yaw_rate_{0.0};

  // Command interface indices
  // 0=fwd 1=rev 2=lft 3=rgt 4=rw_dir
  bool ifaces_ready_{false};
};

}  // namespace bang_bang_controller

