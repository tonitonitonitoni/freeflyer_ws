#include "radial_controller/radial_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <cmath>

namespace radial_controller
{

// -------------------------
controller_interface::CallbackReturn
RadialController::on_init()
{
  auto_declare<double>("r0", 1.2);
  auto_declare<double>("k_r", 10.0);
  auto_declare<double>("k_dr", 20.0);
  auto_declare<double>("k_t", 8.0);
  auto_declare<double>("target_x", 0.0);
  auto_declare<double>("target_y", 0.0);
  auto_declare<double>("orbit_omega", 0.6);
  auto_declare<double>("orbit_direction", 1.0);
  auto_declare<double>("thruster_force", 4.0);
  auto_declare<double>("vehicle_mass", 15.0);
  auto_declare<double>("centripetal_gain", 1.0);
  auto_declare<double>("tangential_force_limit", 2.0);
  auto_declare<double>("tangential_suppress_error", 0.15);
  auto_declare<double>("tangential_outward_gate_error", 0.08);
  auto_declare<double>("tangential_outward_gate_vrad", 0.03);
  auto_declare<double>("radial_deadband", 0.03);
  auto_declare<double>("radial_velocity_deadband", 0.05);

  return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------
controller_interface::CallbackReturn
RadialController::on_configure(const rclcpp_lifecycle::State &)
{
  auto n = get_node();

  r0_ = n->get_parameter("r0").as_double();
  k_r_ = n->get_parameter("k_r").as_double();
  k_dr_ = n->get_parameter("k_dr").as_double();
  k_t_ = n->get_parameter("k_t").as_double();
  target_x_ = n->get_parameter("target_x").as_double();
  target_y_ = n->get_parameter("target_y").as_double();
  orbit_omega_ = n->get_parameter("orbit_omega").as_double();
  orbit_direction_ = n->get_parameter("orbit_direction").as_double();
  Fmax_ = n->get_parameter("thruster_force").as_double();
  vehicle_mass_ = std::max(0.0, n->get_parameter("vehicle_mass").as_double());
  centripetal_gain_ = std::max(0.0, n->get_parameter("centripetal_gain").as_double());
  tangential_force_limit_ = n->get_parameter("tangential_force_limit").as_double();
  tangential_suppress_error_ = std::max(1e-3, n->get_parameter("tangential_suppress_error").as_double());
  tangential_outward_gate_error_ = std::max(0.0, n->get_parameter("tangential_outward_gate_error").as_double());
  tangential_outward_gate_vrad_ = std::max(0.0, n->get_parameter("tangential_outward_gate_vrad").as_double());
  radial_deadband_ = n->get_parameter("radial_deadband").as_double();
  radial_velocity_deadband_ = n->get_parameter("radial_velocity_deadband").as_double();
  tangential_force_limit_ = std::max(0.0, tangential_force_limit_);

  odom_sub_ = n->create_subscription<nav_msgs::msg::Odometry>(
    "/freeflyer/odometry/filtered",
    10,
    std::bind(&RadialController::odom_cb, this, std::placeholders::_1));
  radial_error_pub_ = n->create_publisher<std_msgs::msg::Float64>(
    "/orbit/radial_error", 10);

  return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------
controller_interface::CallbackReturn
RadialController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 4)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Expected 4 command interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (auto &ci : command_interfaces_)
    ci.set_value(0.0);

  ifaces_ready_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------
controller_interface::CallbackReturn
RadialController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto &ci : command_interfaces_)
    ci.set_value(0.0);

  ifaces_ready_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------
controller_interface::InterfaceConfiguration
RadialController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      "thruster_fwd/command",
      "thruster_rev/command",
      "thruster_lft/command",
      "thruster_rgt/command"
    }
  };
}

controller_interface::InterfaceConfiguration
RadialController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

// -------------------------
void RadialController::odom_cb(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(odom_mtx_);

  odom_.valid = true;
  odom_.x = msg->pose.pose.position.x;
  odom_.y = msg->pose.pose.position.y;
  odom_.vx = msg->twist.twist.linear.x;
  odom_.vy = msg->twist.twist.linear.y;
  const auto & q = msg->pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  odom_.yaw = std::atan2(siny_cosp, cosy_cosp);
}

// -------------------------
controller_interface::return_type
RadialController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!ifaces_ready_)
    return controller_interface::return_type::OK;

  OdomState s;
  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    s = odom_;
  }

  if (!s.valid)
    return controller_interface::return_type::OK;

  // Geometry
  const double rx = s.x - target_x_;
  const double ry = s.y - target_y_;
  const double r  = std::hypot(rx, ry);

  if (r < 1e-3)
    return controller_interface::return_type::OK;

  const double r_hat_x = rx / r;
  const double r_hat_y = ry / r;
  const double t_hat_x = -r_hat_y;
  const double t_hat_y = r_hat_x;

  const double cy = std::cos(s.yaw);
  const double sy = std::sin(s.yaw);
  const double vx_w = cy * s.vx - sy * s.vy;
  const double vy_w = sy * s.vx + cy * s.vy;
  const double v_rad = vx_w * r_hat_x + vy_w * r_hat_y;
  const double radial_error = r - r0_;
  radial_error_msg_.data = static_cast<float>(radial_error);
  radial_error_pub_->publish(radial_error_msg_);

  const double dir = orbit_direction_ >= 0.0 ? 1.0 : -1.0;
  const double v_t = vx_w * t_hat_x + vy_w * t_hat_y;
  const double v_t_des = dir * orbit_omega_ * r0_;

  const double f_r_ff = -centripetal_gain_ * vehicle_mass_ * (v_t * v_t) / std::max(r, 1e-3);
  double f_r = f_r_ff;
  const bool outside_orbit = radial_error > 0.0;
  const bool apply_pd = outside_orbit ||
    std::abs(radial_error) > radial_deadband_ ||
    std::abs(v_rad) > radial_velocity_deadband_;
  if (apply_pd) {
    f_r += -k_r_ * radial_error - k_dr_ * v_rad;
  }

  double f_t = clamp(k_t_ * (v_t_des - v_t), -tangential_force_limit_, tangential_force_limit_);
  if (radial_error > 0.0) {
    const double scale = clamp(1.0 - radial_error / tangential_suppress_error_, 0.0, 1.0);
    f_t *= scale;
    if (radial_error > tangential_outward_gate_error_ && v_rad > tangential_outward_gate_vrad_) {
      f_t = 0.0;
    }
  }

  // Convert to world
  const double fx_w = f_r * r_hat_x + f_t * t_hat_x;
  const double fy_w = f_r * r_hat_y + f_t * t_hat_y;

  const double fx_b = cy * fx_w + sy * fy_w;
  const double fy_b = -sy * fx_w + cy * fy_w;

  double fx = clamp(fx_b, -Fmax_, Fmax_);
  double fy = clamp(fy_b, -Fmax_, Fmax_);

  const double fwd = std::max(0.0,  fx) / Fmax_;
  const double rev = std::max(0.0, -fx) / Fmax_;
  const double lft = std::max(0.0,  fy) / Fmax_;
  const double rgt = std::max(0.0, -fy) / Fmax_;

  command_interfaces_[0].set_value(fwd);
  command_interfaces_[1].set_value(rev);
  command_interfaces_[2].set_value(lft);
  command_interfaces_[3].set_value(rgt);

  return controller_interface::return_type::OK;
}

} // namespace radial_controller

PLUGINLIB_EXPORT_CLASS(
  radial_controller::RadialController,
  controller_interface::ControllerInterface)
