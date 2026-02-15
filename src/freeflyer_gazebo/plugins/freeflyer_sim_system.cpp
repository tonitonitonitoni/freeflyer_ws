#include "freeflyer_gazebo/freeflyer_sim_system.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <algorithm>

namespace freeflyer_sim_system
{

double FreeflyerSimSystem::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

bool FreeflyerSimSystem::params_from_hardware_info_(const hardware_interface::HardwareInfo & info)
{
  // Parameters come from <ros2_control><hardware><param name="...">...</param>
  auto get = [&](const std::string & key, double & out) -> bool {
    auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end()) return false;
    out = std::stod(it->second);
    return true;
  };
  auto get_s = [&](const std::string & key, std::string & out) -> bool {
    auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end()) return false;
    out = it->second;
    return true;
  };

  get_s("body_link", body_link_name_);
  get("pwm_neutral", pwm_neutral_);
  get("pwm_max", pwm_max_);
  get("pwm_deadband", pwm_deadband_);
  get("gamma", gamma_);
  get("spool_tau", spool_tau_);
  get("F_max", F_max_);
  get("rw_tau_max", rw_tau_max_);

  // sanity clamps
  if (pwm_max_ <= pwm_neutral_) pwm_max_ = pwm_neutral_ + 1.0;
  pwm_deadband_ = std::max(0.0, pwm_deadband_);
  gamma_ = std::max(0.1, gamma_);
  spool_tau_ = std::max(0.0, spool_tau_);
  F_max_ = std::max(0.0, F_max_);
  rw_tau_max_ = std::max(0.0, rw_tau_max_);

  return true;
}

bool FreeflyerSimSystem::params_from_sdf_(sdf::ElementPtr /*sdf*/)
{
  // Not required for this plugin; kept for future extension.
  return true;
}

double FreeflyerSimSystem::thrust_from_u(double u) const
{
  // Interpret u as desired FORCE fraction
  u = clamp(u, 0.0, 1.0);

  // Linearized: force equals commanded fraction
  return F_max_ * u;
}




double FreeflyerSimSystem::spool(double F_current, double F_target, double dt) const
{
  if (spool_tau_ <= 0.0 || dt <= 0.0) return F_target;

  // First-order lag: F_next = a*F + (1-a)*F_target
  const double a = std::exp(-dt / spool_tau_);
  return a * F_current + (1.0 - a) * F_target;
}

FreeflyerSimSystem::CallbackReturn
FreeflyerSimSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (gazebo_ros2_control::GazeboSystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Ensure expected joints exist
  if (system_info.joints.size() != 5) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FreeflyerSimSystem"),
      "Expected 5 joints (4 thrusters + rw_dir). Got %zu", system_info.joints.size());
    return CallbackReturn::ERROR;
  }

  // Initialize commands to zero
  u_cmd_ = {0.0, 0.0, 0.0, 0.0};
  rw_dir_cmd_ = 0.0;
  F_cmd_ = {0.0, 0.0, 0.0, 0.0};

  // Load params from URDF ros2_control hardware params
  params_from_hardware_info_(system_info);

  return CallbackReturn::SUCCESS;
}

bool FreeflyerSimSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  nh_ = model_nh;
  model_ = parent_model;

  params_from_hardware_info_(hardware_info);
  params_from_sdf_(sdf);

  body_link_ = model_->GetLink(body_link_name_);
  if (!body_link_) {
    RCLCPP_ERROR(
      nh_->get_logger(),
      "FreeflyerSimSystem: body_link '%s' not found in model.",
      body_link_name_.c_str());
    return false;
  }

  RCLCPP_INFO(
    nh_->get_logger(),
    "FreeflyerSimSystem: applying thruster forces to link '%s' (F_max=%.3fN, gamma=%.2f, deadband=%.1fus, tau=%.3fs)",
    body_link_name_.c_str(), F_max_, gamma_, pwm_deadband_, spool_tau_);

  return true;
}

std::vector<hardware_interface::StateInterface>
FreeflyerSimSystem::export_state_interfaces()
{
  // No state interfaces required for now (EKF provides state via topics)
  return {};
}

std::vector<hardware_interface::CommandInterface>
FreeflyerSimSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joint order in URDF should match:
  // thruster_fwd, thruster_rev, thruster_lft, thruster_rgt, rw_dir
  // Each exposes "command"
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(info_.joints[0].name, "command", &u_cmd_[FWD]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(info_.joints[1].name, "command", &u_cmd_[REV]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(info_.joints[2].name, "command", &u_cmd_[LFT]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(info_.joints[3].name, "command", &u_cmd_[RGT]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(info_.joints[4].name, "command", &rw_dir_cmd_));

  return command_interfaces;
}

FreeflyerSimSystem::CallbackReturn
FreeflyerSimSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // reset commands
  u_cmd_ = {0.0, 0.0, 0.0, 0.0};
  rw_dir_cmd_ = 0.0;
  F_cmd_ = {0.0, 0.0, 0.0, 0.0};
  return CallbackReturn::SUCCESS;
}

FreeflyerSimSystem::CallbackReturn
FreeflyerSimSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // neutralize
  u_cmd_ = {0.0, 0.0, 0.0, 0.0};
  rw_dir_cmd_ = 0.0;
  F_cmd_ = {0.0, 0.0, 0.0, 0.0};
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
FreeflyerSimSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Nothing to read (state comes from EKF topic pipeline)
  return hardware_interface::return_type::OK;
}

void FreeflyerSimSystem::apply_wrenches_(double dt)
{
  if (!body_link_) return;

  // Convert normalized command -> thrust target -> spooled thrust
  std::array<double, N_THR> F_target;
  for (size_t i = 0; i < N_THR; ++i) {
    F_target[i] = thrust_from_u(u_cmd_[i]);
    F_cmd_[i] = spool(F_cmd_[i], F_target[i], dt);
  }

  // Body-frame forces:
  // FWD: +X, REV: -X, LFT: +Y, RGT: -Y
  const double Fx =  (F_cmd_[FWD] - F_cmd_[REV]);
  const double Fy =  (F_cmd_[LFT] - F_cmd_[RGT]);

  // Apply in body frame
  body_link_->AddRelativeForce(ignition::math::Vector3d(Fx, Fy, 0.0));

  // Reaction wheel: bang-bang yaw torque about body Z
  double rw = rw_dir_cmd_;
  // Allow controller to write -1/0/1, but clamp just in case
  if (rw > 0.5) rw = 1.0;
  else if (rw < -0.5) rw = -1.0;
  else rw = 0.0;

  const double tau_z = rw_tau_max_ * rw;
  body_link_->AddRelativeTorque(ignition::math::Vector3d(0.0, 0.0, tau_z));
}

hardware_interface::return_type
FreeflyerSimSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = std::max(0.0, period.seconds());

  // Clamp thruster commands to [0,1] to protect sim
  for (auto & u : u_cmd_) {
    u = clamp(u, 0.0, 1.0);
  }

  // Apply the nonlinear wrench model
  apply_wrenches_(dt);

  return hardware_interface::return_type::OK;
}

}  // namespace freeflyer_sim_system

PLUGINLIB_EXPORT_CLASS(
  freeflyer_sim_system::FreeflyerSimSystem,
  gazebo_ros2_control::GazeboSystemInterface)
