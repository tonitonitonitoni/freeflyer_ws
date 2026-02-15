#pragma once

#include <gazebo_ros2_control/gazebo_system_interface.hpp>

#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <string>
#include <vector>
#include <array>

namespace freeflyer_sim_system
{

class FreeflyerSimSystem : public gazebo_ros2_control::GazeboSystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

private:
  // --- Gazebo handles ---
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr body_link_;
  rclcpp::Node::SharedPtr nh_;

  // --- Logical channels ---
  enum ThrusterIdx { FWD=0, REV=1, LFT=2, RGT=3 };
  static constexpr size_t N_THR = 4;

  // Commands coming from ros2_control (0..1), and rw_dir (-1,0,1)
  std::array<double, N_THR> u_cmd_{0.0, 0.0, 0.0, 0.0};
  double rw_dir_cmd_{0.0};

  // Internal filtered thrust commands (Newtons)
  std::array<double, N_THR> F_cmd_{0.0, 0.0, 0.0, 0.0};

  // --- Parameters for PWM->thrust model ---
  double pwm_neutral_{1000.0};
  double pwm_max_{1500.0};
  double pwm_deadband_{40.0};
  double gamma_{2.0};
  double spool_tau_{0.10};
  double F_max_{2.0};       // N at pwm_max
  double rw_tau_max_{0.05}; // Nm bang-bang torque

  std::string body_link_name_{"base_link"};

  // --- Helpers ---
  static double clamp(double v, double lo, double hi);
  double thrust_from_u(double u) const;
  double spool(double F_current, double F_target, double dt) const;

  bool params_from_hardware_info_(const hardware_interface::HardwareInfo & info);
  bool params_from_sdf_(sdf::ElementPtr sdf);

  void apply_wrenches_(double dt);
};

}  // namespace freeflyer_sim_system
