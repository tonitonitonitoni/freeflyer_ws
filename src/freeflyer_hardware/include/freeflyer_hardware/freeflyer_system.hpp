#pragma once

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace freeflyer_hardware
{

class FreeflyerSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FreeflyerSystem);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Command storage
  double cmd_fwd_{0.0};
  double cmd_rev_{0.0};
  double cmd_lft_{0.0};
  double cmd_rgt_{0.0};
  double cmd_rw_{0.0};

  // Serial parameters
  std::string port_;
  int baudrate_{115200};

  int neutral_pwm_{1000};
  int max_pwm_{1500};

  double battery_voltage_ = 0.0;
  double packet_counter_ = 0.0;
  double watchdog_active_ = 0.0;
  double rw_state_ = 0.0;

  char rx_buffer_[256];
  size_t rx_index_ = 0;


  // Serial handle (placeholder — implement however you like)
  int serial_fd_{-1};

  bool open_serial();
  void close_serial();
  void send_neutral();
  void send_command();

  // Write watchdog
  rclcpp::Time last_write_time_{0, 0, RCL_SYSTEM_TIME};
  double watchdog_timeout_{0.5};
  rclcpp::Time last_telemetry_time_;
  
};

}  // namespace freeflyer_hardware
