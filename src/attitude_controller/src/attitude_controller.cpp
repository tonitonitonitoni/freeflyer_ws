#include "attitude_controller/attitude_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

namespace attitude_controller
{

double AttitudeController::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double AttitudeController::wrap_to_pi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

controller_interface::CallbackReturn AttitudeController::on_init()
{
  auto_declare<double>("r0", 1.5);
  auto_declare<double>("v_tan_des", 0.1);

  auto_declare<double>("k_yaw", 5.0);
  auto_declare<double>("k_yaw_rate", 5.0);
  auto_declare<double>("yaw_deadband", 0.02);
  auto_declare<double>("yaw_on_threshold", 0.45);
  auto_declare<double>("yaw_off_threshold", 0.20);
  auto_declare<double>("yaw_rate_limit", 0.35);
  auto_declare<double>("yaw_rate_error_on_threshold", 0.015);
  auto_declare<double>("yaw_rate_error_off_threshold", 0.005);
  auto_declare<double>("yaw_surface_gain", 0.8);
  auto_declare<double>("yaw_surface_on_threshold", 0.02);
  auto_declare<double>("yaw_surface_off_threshold", 0.004);
  auto_declare<double>("yaw_ref_rate_limit", 0.15);
  auto_declare<bool>("use_geometric_yaw_rate", true);
  auto_declare<double>("yaw_min_speed_for_control", 0.05);
  auto_declare<double>("attitude_enable_radial_tol", 0.12);
  auto_declare<double>("attitude_enable_vtan_tol", 0.12);
  auto_declare<double>("reaction_torque", 0.015);
  auto_declare<bool>("enable_attitude_control", true);

  auto_declare<double>("target_x", 0.0);
  auto_declare<double>("target_y", 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AttitudeController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      "rw_dir/command"
    }
  };
}

controller_interface::InterfaceConfiguration
AttitudeController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn
AttitudeController::on_configure(const rclcpp_lifecycle::State &)
{
  auto n = get_node();

  r0_ = n->get_parameter("r0").as_double();
  v_tan_des_ = n->get_parameter("v_tan_des").as_double();
  use_odom_twist_ = true;

  k_yaw_ = n->get_parameter("k_yaw").as_double();
  k_yaw_rate_ = n->get_parameter("k_yaw_rate").as_double();
  yaw_deadband_ = std::max(0.0, n->get_parameter("yaw_deadband").as_double());
  yaw_on_threshold_ = std::abs(n->get_parameter("yaw_on_threshold").as_double());
  yaw_off_threshold_ = std::abs(n->get_parameter("yaw_off_threshold").as_double());
  yaw_rate_limit_ = std::abs(n->get_parameter("yaw_rate_limit").as_double());
  yaw_rate_error_on_threshold_ = std::abs(n->get_parameter("yaw_rate_error_on_threshold").as_double());
  yaw_rate_error_off_threshold_ = std::abs(n->get_parameter("yaw_rate_error_off_threshold").as_double());
  yaw_surface_gain_ = n->get_parameter("yaw_surface_gain").as_double();
  yaw_surface_on_threshold_ = std::abs(n->get_parameter("yaw_surface_on_threshold").as_double());
  yaw_surface_off_threshold_ = std::abs(n->get_parameter("yaw_surface_off_threshold").as_double());
  yaw_ref_rate_limit_ = std::abs(n->get_parameter("yaw_ref_rate_limit").as_double());
  use_geometric_yaw_rate_ = n->get_parameter("use_geometric_yaw_rate").as_bool();
  yaw_min_speed_for_control_ = std::max(0.0, n->get_parameter("yaw_min_speed_for_control").as_double());
  attitude_enable_radial_tol_ = std::max(0.0, n->get_parameter("attitude_enable_radial_tol").as_double());
  attitude_enable_vtan_tol_ = std::max(0.0, n->get_parameter("attitude_enable_vtan_tol").as_double());
  Tmax_ = std::abs(n->get_parameter("reaction_torque").as_double());
  enable_attitude_control_ = n->get_parameter("enable_attitude_control").as_bool();

  if (yaw_off_threshold_ > yaw_on_threshold_) {
    yaw_off_threshold_ = yaw_on_threshold_;
  }

  target_x_ = n->get_parameter("target_x").as_double();
  target_y_ = n->get_parameter("target_y").as_double();

  odom_sub_ = n->create_subscription<nav_msgs::msg::Odometry>(
    "/freeflyer/odometry/filtered", 10,
    std::bind(&AttitudeController::odom_cb, this, std::placeholders::_1));

  yaw_err_pub_ = n->create_publisher<std_msgs::msg::Float64>("/orbit/yaw_error", 10);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    odom_valid_ = false;
    rw_cmd_ = 0.0;
    rw_dir_ = 0;
    attitude_ready_ = false;
    yaw_error_ = 0.0;
    yaw_rate_ = 0.0;
    desired_yaw_rate_ = 0.0;
    desired_yaw_cmd_ = 0.0;
    desired_yaw_inited_ = false;
    last_guidance_t_ = -1.0;
    last_x_ = 0.0;
    last_y_ = 0.0;
    last_t_ = -1.0;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AttitudeController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 1) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 1 command interface (rw_dir), got %zu", command_interfaces_.size());
    ifaces_ready_ = false;
    return controller_interface::CallbackReturn::ERROR;
  }

  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  ifaces_ready_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AttitudeController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (ifaces_ready_) {
    for (auto & ci : command_interfaces_) {
      ci.set_value(0.0);
    }
  }
  ifaces_ready_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

void AttitudeController::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  const double t = static_cast<double>(msg->header.stamp.sec) +
    1e-9 * static_cast<double>(msg->header.stamp.nanosec);

  const auto & q = msg->pose.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  double vx = 0.0;
  double vy = 0.0;

  if (use_odom_twist_) {
    const double vx_m = msg->twist.twist.linear.x;
    const double vy_m = msg->twist.twist.linear.y;
    const std::string child = msg->child_frame_id;
    const bool twist_in_body =
      child.empty() || (child.size() >= 8 && child.rfind("base_link") == child.size() - 8);
    if (twist_in_body) {
      vx = c * vx_m - s * vy_m;
      vy = s * vx_m + c * vy_m;
    } else {
      vx = vx_m;
      vy = vy_m;
    }
  } else {
    std::lock_guard<std::mutex> lk(mtx_);
    if (last_t_ < 0.0) {
      last_x_ = x;
      last_y_ = y;
      last_t_ = t;
      return;
    }
    const double dt = t - last_t_;
    if (dt > 1e-4) {
      vx = (x - last_x_) / dt;
      vy = (y - last_y_) / dt;
    }
    last_x_ = x;
    last_y_ = y;
    last_t_ = t;
  }

  const double yaw_rate = msg->twist.twist.angular.z;

  const double rx = x - target_x_;
  const double ry = y - target_y_;
  const double r = std::hypot(rx, ry);
  if (r < 1e-3) {
    return;
  }

  const double r_hat_x = rx / r;
  const double r_hat_y = ry / r;
  const double t_hat_x = -r_hat_y;
  const double t_hat_y = r_hat_x;

  const double radial_error = r - r0_;
  const double v_tan = vx * t_hat_x + vy * t_hat_y;
  const double speed = std::hypot(vx, vy);
  const bool attitude_ready =
    std::abs(radial_error) <= attitude_enable_radial_tol_ &&
    std::abs(speed - std::abs(v_tan_des_)) <= attitude_enable_vtan_tol_ &&
    speed >= yaw_min_speed_for_control_;

  double desired_yaw_cmd = yaw;
  double desired_yaw_rate = 0.0;
  double yaw_error = 0.0;
  int rw_dir = 0;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!desired_yaw_inited_) {
      desired_yaw_cmd_ = yaw;
      desired_yaw_inited_ = true;
    }
    desired_yaw_cmd = desired_yaw_cmd_;
    rw_dir = rw_dir_;

    const double dt_guidance =
      (last_guidance_t_ >= 0.0) ? std::max(0.0, t - last_guidance_t_) : 0.0;
    last_guidance_t_ = t;

    const double desired_yaw = std::atan2(-r_hat_y, -r_hat_x);
    if (dt_guidance > 0.0 && speed >= yaw_min_speed_for_control_) {
      double yaw_step = wrap_to_pi(desired_yaw - desired_yaw_cmd_);
      const double max_step = yaw_ref_rate_limit_ * dt_guidance;
      if (std::abs(yaw_step) > max_step) {
        yaw_step = std::copysign(max_step, yaw_step);
      }
      desired_yaw_cmd_ = wrap_to_pi(desired_yaw_cmd_ + yaw_step);
      if (use_geometric_yaw_rate_) {
        const double yr_des = v_tan / std::max(r, 1e-3);
        desired_yaw_rate_ = clamp(yr_des, -yaw_ref_rate_limit_, yaw_ref_rate_limit_);
      } else {
        desired_yaw_rate_ = yaw_step / dt_guidance;
      }
    } else {
      desired_yaw_rate_ = 0.0;
    }

    desired_yaw_cmd = desired_yaw_cmd_;
    desired_yaw_rate = desired_yaw_rate_;
    yaw_error = wrap_to_pi(desired_yaw_cmd - yaw);
    yaw_error_ = yaw_error;
    yaw_rate_ = yaw_rate;
    attitude_ready_ = attitude_ready;
  }

  std_msgs::msg::Float64 yaw_err_msg;
  yaw_err_msg.data = yaw_error;
  yaw_err_pub_->publish(yaw_err_msg);

  const double yaw_rate_error = desired_yaw_rate - yaw_rate;
  double tau = k_yaw_ * yaw_error + k_yaw_rate_ * yaw_rate_error;
  tau = clamp(tau, -Tmax_, Tmax_);
  if (std::abs(yaw_error) < yaw_deadband_) {
    tau = 0.0;
  }

  if (!enable_attitude_control_ || !attitude_ready) {
    rw_dir = 0;
  } else {
    const double surf = yaw_rate_error + yaw_surface_gain_ * yaw_error;
    if (rw_dir == 0) {
      if (surf > yaw_surface_on_threshold_) rw_dir = +1;
      else if (surf < -yaw_surface_on_threshold_) rw_dir = -1;
      else if (yaw_error > yaw_on_threshold_) rw_dir = +1;
      else if (yaw_error < -yaw_on_threshold_) rw_dir = -1;
    } else if (rw_dir > 0) {
      if (surf < yaw_surface_off_threshold_ ||
          yaw_rate_error < yaw_rate_error_off_threshold_ ||
          yaw_rate > yaw_rate_limit_ ||
          yaw_error < -yaw_off_threshold_) {
        rw_dir = 0;
      }
    } else {
      if (surf > -yaw_surface_off_threshold_ ||
          yaw_rate_error > -yaw_rate_error_off_threshold_ ||
          yaw_rate < -yaw_rate_limit_ ||
          yaw_error > yaw_off_threshold_) {
        rw_dir = 0;
      }
    }
  }

  std::lock_guard<std::mutex> lk(mtx_);
  rw_dir_ = rw_dir;
  rw_cmd_ = static_cast<double>(rw_dir_);
  (void)tau;
  odom_valid_ = true;
}

controller_interface::return_type
AttitudeController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!ifaces_ready_) {
    return controller_interface::return_type::OK;
  }

  double rw_cmd = 0.0;
  bool odom_valid = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    rw_cmd = rw_cmd_;
    odom_valid = odom_valid_;
  }

  if (!odom_valid) {
    command_interfaces_[0].set_value(0.0);
    return controller_interface::return_type::OK;
  }

  command_interfaces_[0].set_value(rw_cmd);

  return controller_interface::return_type::OK;
}

}  // namespace attitude_controller

PLUGINLIB_EXPORT_CLASS(
  attitude_controller::AttitudeController,
  controller_interface::ControllerInterface)
