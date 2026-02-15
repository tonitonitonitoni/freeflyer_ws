#include "bang_bang_controller/bang_bang_circle_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <algorithm>
namespace bang_bang_controller
{

// ---------- small math helpers ----------
double BangBangCircleController::wrap_to_pi(double a)
{
  // numerically nice: atan2(sin,cos)
  return std::atan2(std::sin(a), std::cos(a));
}

double BangBangCircleController::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double BangBangCircleController::hypot2(double a, double b)
{
  return std::sqrt(a*a + b*b);
}

// ---------- lifecycle ----------
controller_interface::CallbackReturn BangBangCircleController::on_init()
{
  // Orbit guidance params
  auto_declare<double>("r0", 1.5);
  auto_declare<double>("v_tan_des", 0.1);
  auto_declare<double>("k_r", 5.0);
  auto_declare<double>("k_dr", 2.7);
  auto_declare<double>("k_ir", 0.35);
  auto_declare<double>("radial_i_limit", 0.25);
  auto_declare<double>("radial_i_zone", 0.25);
  auto_declare<double>("radial_i_leak", 0.25);
  auto_declare<double>("k_t", 1.0);
  auto_declare<double>("k_cent", 1.0);
  auto_declare<bool>("enable_tangential_control", true);
  auto_declare<bool>("enable_centripetal_ff", true);
  auto_declare<bool>("enable_radial_integral", true);
  auto_declare<bool>("use_odom_twist", true);

  // Attitude control params
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

  // Actuator limits
  auto_declare<double>("thruster_force", 4.0);
  auto_declare<double>("reaction_torque", 0.05);
  auto_declare<bool>("enable_attitude_control", true);

  // Target
  auto_declare<double>("target_x", 0.0);
  auto_declare<double>("target_y", 0.0);

  // Debug / safety
  auto_declare<bool>("enable_debug", true);
  auto_declare<double>("odom_timeout", 0.2);
  auto_declare<bool>("require_fresh_odom", true);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BangBangCircleController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      "thruster_fwd/command",
      "thruster_rev/command",
      "thruster_lft/command",
      "thruster_rgt/command",
      "rw_dir/command"
    }
  };
}

controller_interface::InterfaceConfiguration
BangBangCircleController::state_interface_configuration() const
{
  // We ingest state from Odometry (EKF), not from state interfaces.
  return { controller_interface::interface_configuration_type::NONE };
}

controller_interface::CallbackReturn
BangBangCircleController::on_configure(const rclcpp_lifecycle::State &)
{
  // Load parameters (mirror Python’s init)
  auto n = get_node();

  r0_ = n->get_parameter("r0").as_double();
  v_tan_des_ = n->get_parameter("v_tan_des").as_double();
  k_r_ = n->get_parameter("k_r").as_double();
  k_dr_ = n->get_parameter("k_dr").as_double();
  k_ir_ = n->get_parameter("k_ir").as_double();

  radial_i_limit_ = std::max(0.0, n->get_parameter("radial_i_limit").as_double());
  radial_i_zone_  = std::max(0.0, n->get_parameter("radial_i_zone").as_double());
  radial_i_leak_  = std::max(0.0, n->get_parameter("radial_i_leak").as_double());

  k_t_ = n->get_parameter("k_t").as_double();
  k_cent_ = n->get_parameter("k_cent").as_double();

  enable_tangential_control_ = n->get_parameter("enable_tangential_control").as_bool();
  enable_centripetal_ff_     = n->get_parameter("enable_centripetal_ff").as_bool();
  enable_radial_integral_    = n->get_parameter("enable_radial_integral").as_bool();
  use_odom_twist_            = n->get_parameter("use_odom_twist").as_bool();

  k_yaw_ = n->get_parameter("k_yaw").as_double();
  k_yaw_rate_ = n->get_parameter("k_yaw_rate").as_double();

  yaw_deadband_ = n->get_parameter("yaw_deadband").as_double();
  yaw_on_threshold_ = std::abs(n->get_parameter("yaw_on_threshold").as_double());
  yaw_off_threshold_ = std::abs(n->get_parameter("yaw_off_threshold").as_double());
  yaw_rate_limit_ = std::abs(n->get_parameter("yaw_rate_limit").as_double());
  yaw_rate_error_on_threshold_  = std::abs(n->get_parameter("yaw_rate_error_on_threshold").as_double());
  yaw_rate_error_off_threshold_ = std::abs(n->get_parameter("yaw_rate_error_off_threshold").as_double());
  yaw_surface_gain_ = n->get_parameter("yaw_surface_gain").as_double();
  yaw_surface_on_threshold_ = std::abs(n->get_parameter("yaw_surface_on_threshold").as_double());
  yaw_surface_off_threshold_ = std::abs(n->get_parameter("yaw_surface_off_threshold").as_double());
  yaw_ref_rate_limit_ = std::abs(n->get_parameter("yaw_ref_rate_limit").as_double());
  use_geometric_yaw_rate_ = n->get_parameter("use_geometric_yaw_rate").as_bool();

  yaw_min_speed_for_control_ = std::max(0.0, n->get_parameter("yaw_min_speed_for_control").as_double());
  attitude_enable_radial_tol_ = std::max(0.0, n->get_parameter("attitude_enable_radial_tol").as_double());
  attitude_enable_vtan_tol_   = std::max(0.0, n->get_parameter("attitude_enable_vtan_tol").as_double());

  if (yaw_off_threshold_ > yaw_on_threshold_)
    yaw_off_threshold_ = yaw_on_threshold_;

  Fmax_ = n->get_parameter("thruster_force").as_double();
  Tmax_ = n->get_parameter("reaction_torque").as_double();
  enable_attitude_control_ = n->get_parameter("enable_attitude_control").as_bool();

  target_x_ = n->get_parameter("target_x").as_double();
  target_y_ = n->get_parameter("target_y").as_double();

  enable_debug_ = n->get_parameter("enable_debug").as_bool();
  odom_timeout_ = std::max(0.0, n->get_parameter("odom_timeout").as_double());
  require_fresh_odom_ = n->get_parameter("require_fresh_odom").as_bool();

  // Subscribers
  odom_sub_ = n->create_subscription<nav_msgs::msg::Odometry>(
    "/freeflyer/odometry/filtered", 10,
    std::bind(&BangBangCircleController::odom_cb, this, std::placeholders::_1));

  // Always publish core tracking errors for plotting/monitoring.
  pub_radial_err_ = n->create_publisher<std_msgs::msg::Float64>("/orbit/radial_error", 10);
  pub_yaw_err_    = n->create_publisher<std_msgs::msg::Float64>("/orbit/yaw_error", 10);

  // Optional extended debug publishers.
  if (enable_debug_) {
    pub_fx_w_       = n->create_publisher<std_msgs::msg::Float64>("/debug/fx_w", 10);
    pub_fy_w_       = n->create_publisher<std_msgs::msg::Float64>("/debug/fy_w", 10);
    pub_fr_         = n->create_publisher<std_msgs::msg::Float64>("/debug/f_r", 10);
  }

  // Reset internal state
  fx_cmd_body_ = 0.0;
  fy_cmd_body_ = 0.0;
  tau_cmd_ = 0.0;
  radial_error_int_ = 0.0;

  desired_yaw_inited_ = false;
  desired_yaw_cmd_ = 0.0;
  desired_yaw_rate_ = 0.0;

  rw_dir_ = 0;
  attitude_ready_ = false;
  yaw_error_ = 0.0;
  yaw_rate_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BangBangCircleController::on_activate(const rclcpp_lifecycle::State &)
{
  // command_interfaces_ is provided by base class; ensure expected size
  if (command_interfaces_.size() != 5) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Expected 5 command interfaces, got %zu", command_interfaces_.size());
    ifaces_ready_ = false;
    return controller_interface::CallbackReturn::ERROR;
  }

  // Neutral commands on activate
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  ifaces_ready_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BangBangCircleController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (ifaces_ready_) {
    for (auto & ci : command_interfaces_) {
      ci.set_value(0.0);
    }
  }
  ifaces_ready_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------- odom ingest (parity with Python) ----------
void BangBangCircleController::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  OdomState s;
  s.valid = true;
  s.stamp = msg->header.stamp;

  s.x = msg->pose.pose.position.x;
  s.y = msg->pose.pose.position.y;

  const auto & q = msg->pose.pose.orientation;
  s.yaw = std::atan2(
    2.0 * (q.w*q.z + q.x*q.y),
    1.0 - 2.0 * (q.y*q.y + q.z*q.z));

  const double c = std::cos(s.yaw);
  const double sn = std::sin(s.yaw);

  // Python logic: if use_odom_twist -> interpret twist possibly in body frame
  double vx_m = msg->twist.twist.linear.x;
  double vy_m = msg->twist.twist.linear.y;

  if (use_odom_twist_) {
    std::string child = msg->child_frame_id;
    // Heuristic: if empty or endswith base_link => twist in body frame
    bool twist_in_body = child.empty() || (child.size() >= 8 && child.rfind("base_link") == child.size() - 8);
    s.twist_was_body = twist_in_body;

    if (twist_in_body) {
      // body -> world
      s.vx = c * vx_m - sn * vy_m;
      s.vy = sn * vx_m + c * vy_m;
    } else {
      s.vx = vx_m;
      s.vy = vy_m;
    }
  } else {
    // If you ever want finite-difference here, do it inside update() with stored last pose.
    // For parity with Python behavior when use_odom_twist=false, you'd implement FD.
    // Most setups should keep use_odom_twist=true with EKF.
    s.vx = vx_m;
    s.vy = vy_m;
  }

  s.yaw_rate = msg->twist.twist.angular.z;

  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    odom_ = s;
  }
}

void BangBangCircleController::compute_guidance_and_attitude(
  const OdomState & s, double dt)
{
  // ----------------------------
  // Defaults
  // ----------------------------
  fx_cmd_body_ = 0.0;
  fy_cmd_body_ = 0.0;
  tau_cmd_ = 0.0;
  desired_yaw_rate_ = 0.0;

  // ----------------------------
  // Orbit geometry
  // ----------------------------
  const double rx = s.x - target_x_;
  const double ry = s.y - target_y_;
  const double r  = std::hypot(rx, ry);

  if (r < 1e-3) {
    attitude_ready_ = false;
    return;
  }

  const double radial_error = r - r0_;

  if (pub_radial_err_) {
    std_msgs::msg::Float64 m;
    m.data = radial_error;
    pub_radial_err_->publish(m);
  }

  const double r_hat_x = rx / r;
  const double r_hat_y = ry / r;
  const double t_hat_x = -r_hat_y;
  const double t_hat_y =  r_hat_x;

  const double v_rad = s.vx * r_hat_x + s.vy * r_hat_y;
  const double v_tan = s.vx * t_hat_x + s.vy * t_hat_y;
  const double speed = std::hypot(s.vx, s.vy);

  attitude_ready_ =
      (std::abs(radial_error) <= attitude_enable_radial_tol_) &&
      (std::abs(speed - std::abs(v_tan_des_)) <= attitude_enable_vtan_tol_) &&
      (speed >= yaw_min_speed_for_control_);

  // ----------------------------
  // Radial integral (leaky)
  // ----------------------------
  if (enable_radial_integral_ && dt > 0.0) {
    const double leak_scale = std::max(0.0, 1.0 - radial_i_leak_ * dt);
    radial_error_int_ *= leak_scale;

    if (std::abs(radial_error) <= radial_i_zone_) {
      radial_error_int_ += radial_error * dt;
    }

    radial_error_int_ =
        clamp(radial_error_int_, -radial_i_limit_, radial_i_limit_);
  }

  // ----------------------------
  // Radial force
  // ----------------------------
  double f_r = -k_r_ * radial_error
               -k_dr_ * v_rad;

  if (enable_radial_integral_) {
    f_r += -k_ir_ * radial_error_int_;
  }

  // Proper centripetal physics: F = m v^2 / r
  if (enable_centripetal_ff_) {
    f_r += -k_cent_ * mass_ *
           (v_tan * v_tan / std::max(r, 1e-3));
  }

  // ----------------------------
  // Tangential force
  // ----------------------------
  const double f_t =
      enable_tangential_control_
      ? k_t_ * (v_tan_des_ - v_tan)
      : 0.0;

  const double fx_w = f_r * r_hat_x + f_t * t_hat_x;
  const double fy_w = f_r * r_hat_y + f_t * t_hat_y;

  if (enable_debug_) {
    if (pub_fr_) {
      std_msgs::msg::Float64 m; m.data = f_r;
      pub_fr_->publish(m);
    }
    if (pub_fx_w_) {
      std_msgs::msg::Float64 m; m.data = fx_w;
      pub_fx_w_->publish(m);
    }
    if (pub_fy_w_) {
      std_msgs::msg::Float64 m; m.data = fy_w;
      pub_fy_w_->publish(m);
    }
  }

  // ----------------------------
  // Rotate world -> body
  // ----------------------------
  const double c  = std::cos(s.yaw);
  const double sn = std::sin(s.yaw);

  fx_cmd_body_ =  c * fx_w + sn * fy_w;
  fy_cmd_body_ = -sn * fx_w + c * fy_w;

  // ============================================================
  //           Continuous Desired Yaw (NO 2π JUMPS)
  // ============================================================

  const double desired_yaw_wrapped =
      std::atan2(-r_hat_y, -r_hat_x);

  if (!desired_yaw_unwrap_inited_) {
    desired_yaw_unwrapped_ = desired_yaw_wrapped;
    desired_yaw_unwrap_inited_ = true;
  } else {
    const double delta =
        wrap_to_pi(desired_yaw_wrapped - desired_yaw_unwrapped_);
    desired_yaw_unwrapped_ += delta;
  }

  if (!desired_yaw_inited_) {
    desired_yaw_cmd_ = s.yaw;
    desired_yaw_inited_ = true;
  }

  // ----------------------------
  // Yaw shaping
  // ----------------------------
  desired_yaw_rate_ = 0.0;

  if (dt > 0.0 && speed >= yaw_min_speed_for_control_) {

    const double desired_yaw_target_wrapped =
        wrap_to_pi(desired_yaw_unwrapped_);

    double yaw_step =
        wrap_to_pi(desired_yaw_target_wrapped - desired_yaw_cmd_);

    const double max_step = yaw_ref_rate_limit_ * dt;

    if (std::abs(yaw_step) > max_step) {
      yaw_step = std::copysign(max_step, yaw_step);
    }

    desired_yaw_cmd_ =
        wrap_to_pi(desired_yaw_cmd_ + yaw_step);

    if (use_geometric_yaw_rate_) {
      const double yr_des =
          v_tan / std::max(r, 1e-3);

      desired_yaw_rate_ =
          clamp(yr_des,
                -yaw_ref_rate_limit_,
                 yaw_ref_rate_limit_);
    } else {
      desired_yaw_rate_ = yaw_step / dt;
    }
  }

  // ----------------------------
  // Yaw error
  // ----------------------------
  yaw_error_ =
      wrap_to_pi(desired_yaw_cmd_ - s.yaw);

  yaw_rate_ = s.yaw_rate;

  if (pub_yaw_err_) {
    std_msgs::msg::Float64 m;
    m.data = yaw_error_;
    pub_yaw_err_->publish(m);
  }

  // ----------------------------
  // Continuous torque (optional)
  // ----------------------------
  if (enable_attitude_control_) {
    const double yaw_rate_error =
        desired_yaw_rate_ - s.yaw_rate;

    double tau =
        k_yaw_ * yaw_error_
      + k_yaw_rate_ * yaw_rate_error;

    tau = clamp(tau, -Tmax_, Tmax_);

    if (std::abs(yaw_error_) < yaw_deadband_) {
      tau = 0.0;
    }

    tau_cmd_ = tau;
  } else {
    tau_cmd_ = 0.0;
  }
}


// RW bang-bang control with hysteresis + limits (ported from Python)
void BangBangCircleController::compute_rw_switching()
{
  if (!enable_attitude_control_ || !attitude_ready_) {
    rw_dir_ = 0;
    return;
  }

  const double yerr = yaw_error_;
  const double yr = yaw_rate_;
  const double yre = desired_yaw_rate_ - yr;

  // Sliding surface
  const double s = yre + yaw_surface_gain_ * yerr;

  if (rw_dir_ == 0) {
    if (s > yaw_surface_on_threshold_) {
      rw_dir_ = +1;
    } else if (s < -yaw_surface_on_threshold_) {
      rw_dir_ = -1;
    } else if (yerr > yaw_on_threshold_) {
      rw_dir_ = +1;
    } else if (yerr < -yaw_on_threshold_) {
      rw_dir_ = -1;
    }
  } else if (rw_dir_ > 0) {
    if (
      s < yaw_surface_off_threshold_ ||
      yre < yaw_rate_error_off_threshold_ ||
      yr > yaw_rate_limit_ ||
      yerr < -yaw_off_threshold_
    ) {
      rw_dir_ = 0;
    }
  } else { // rw_dir_ < 0
    if (
      s > -yaw_surface_off_threshold_ ||
      yre > -yaw_rate_error_off_threshold_ ||
      yr < -yaw_rate_limit_ ||
      yerr > yaw_off_threshold_
    ) {
      rw_dir_ = 0;
    }
  }
}

void BangBangCircleController::write_actuators()
{
  if (!ifaces_ready_) return;

  // Clamp desired body forces
  const double fx = clamp(fx_cmd_body_, -Fmax_, Fmax_);
  const double fy = clamp(fy_cmd_body_, -Fmax_, Fmax_);

  // Split into one-sided normalized thrusters:
  // +X => fwd, -X => rev, +Y => lft, -Y => rgt
  const double fwd = std::max(0.0,  fx) / Fmax_;
  const double rev = std::max(0.0, -fx) / Fmax_;
  const double lft = std::max(0.0,  fy) / Fmax_;
  const double rgt = std::max(0.0, -fy) / Fmax_;

  command_interfaces_[0].set_value(fwd);
  command_interfaces_[1].set_value(rev);
  command_interfaces_[2].set_value(lft);
  command_interfaces_[3].set_value(rgt);
  command_interfaces_[4].set_value(static_cast<double>(rw_dir_)); // -1,0,+1
}

controller_interface::return_type
BangBangCircleController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Grab latest odom snapshot
  OdomState s;
  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    s = odom_;
  }

  if (!s.valid) {
    // No state yet -> neutral
    fx_cmd_body_ = fy_cmd_body_ = 0.0;
    rw_dir_ = 0;
    write_actuators();
    return controller_interface::return_type::OK;
  }

  // Freshness check
  if (require_fresh_odom_) {
    const double age = (time - s.stamp).seconds();
    if (age > odom_timeout_) {
      fx_cmd_body_ = fy_cmd_body_ = 0.0;
      rw_dir_ = 0;
      write_actuators();
      return controller_interface::return_type::OK;
    }
  }

  // Use controller period as dt for integrators & yaw shaping (deterministic)
  const double dt = std::max(0.0, period.seconds());

  compute_guidance_and_attitude(s, dt);
  compute_rw_switching();
  write_actuators();

  return controller_interface::return_type::OK;
}

} // namespace bang_bang_controller

PLUGINLIB_EXPORT_CLASS(
  bang_bang_controller::BangBangCircleController,
  controller_interface::ControllerInterface)
