#include "freeflyer_hardware/freeflyer_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdio>
#include <algorithm>

namespace freeflyer_hardware
{

hardware_interface::CallbackReturn
FreeflyerSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  port_ = info.hardware_parameters.at("port");
  baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));
  neutral_pwm_ = std::stoi(info.hardware_parameters.at("neutral_pwm"));
  max_pwm_ = std::stoi(info.hardware_parameters.at("max_pwm"));
  watchdog_timeout_ = 0.5;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FreeflyerSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  interfaces.emplace_back("battery_voltage", "state", &battery_voltage_);
  interfaces.emplace_back("packet_counter", "state", &packet_counter_);
  interfaces.emplace_back("watchdog", "state", &watchdog_active_);
  interfaces.emplace_back("rw_state", "state", &rw_state_);

  return interfaces;
}


std::vector<hardware_interface::CommandInterface>
FreeflyerSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(5);
  interfaces.emplace_back("thruster_fwd", "command", &cmd_fwd_);
  interfaces.emplace_back("thruster_rev", "command", &cmd_rev_);
  interfaces.emplace_back("thruster_lft", "command", &cmd_lft_);
  interfaces.emplace_back("thruster_rgt", "command", &cmd_rgt_);
  interfaces.emplace_back("rw_dir", "command", &cmd_rw_);
  return interfaces;
}

hardware_interface::CallbackReturn
FreeflyerSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_serial())
    return hardware_interface::CallbackReturn::ERROR;

  send_neutral();
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  auto now = clock.now();
  last_write_time_ = now;
  last_telemetry_time_ = now;


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FreeflyerSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_neutral();
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
FreeflyerSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
    return hardware_interface::return_type::OK;

  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  const auto now = clock.now();

  char c;
  ssize_t n;

  // Non-blocking read
  while ((n = ::read(serial_fd_, &c, 1)) > 0) {

    if (c == '\n') {
      rx_buffer_[rx_index_] = '\0';

      if (std::strncmp(rx_buffer_, "$TEL,", 5) == 0) {

        uint32_t pkt;
        uint32_t last_rx;
        uint16_t pwm_fwd, pwm_rev, pwm_lft, pwm_rgt;
        int rw;
        int wd;
        uint16_t vbat;

        int parsed = std::sscanf(
          rx_buffer_,
          "$TEL,%u,%u,%hu,%hu,%hu,%hu,%d,%d,%hu",
          &pkt,
          &last_rx,
          &pwm_fwd,
          &pwm_rev,
          &pwm_lft,
          &pwm_rgt,
          &rw,
          &wd,
          &vbat
        );

        if (parsed == 9) {
          packet_counter_  = static_cast<double>(pkt);
          watchdog_active_ = static_cast<double>(wd);
          rw_state_        = static_cast<double>(rw);
          battery_voltage_ = static_cast<double>(vbat) / 1000.0;

          // Update telemetry timestamp ONLY on valid packet
          last_telemetry_time_ = now;
        }
      }

      rx_index_ = 0;
    }
    else {
      if (rx_index_ < sizeof(rx_buffer_) - 1)
        rx_buffer_[rx_index_++] = c;
    }
  }

  // Telemetry timeout detection
  if (last_telemetry_time_.nanoseconds() > 0) {
    if ((now - last_telemetry_time_).seconds() > 1.0) {
      watchdog_active_ = 1.0;  // communication stale
    }
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type
FreeflyerSystem::write(const rclcpp::Time & time, const rclcpp::Duration &)
{
  // Watchdog
  if ((time - last_write_time_).seconds() > watchdog_timeout_) {
    send_neutral();
    return hardware_interface::return_type::OK;
  }

  auto clamp01 = [](double v) {
    return std::max(0.0, std::min(1.0, v));
  };

  cmd_fwd_ = clamp01(cmd_fwd_);
  cmd_rev_ = clamp01(cmd_rev_);
  cmd_lft_ = clamp01(cmd_lft_);
  cmd_rgt_ = clamp01(cmd_rgt_);

  int rw = (cmd_rw_ > 0.5) ? 1 : (cmd_rw_ < -0.5 ? -1 : 0);

  char buf[128];
  std::snprintf(
    buf, sizeof(buf),
    "$CMD,%.3f,%.3f,%.3f,%.3f,%d\n",
    cmd_fwd_, cmd_rev_, cmd_lft_, cmd_rgt_, rw);

  ::write(serial_fd_, buf, std::strlen(buf));

  last_write_time_ = time;
  return hardware_interface::return_type::OK;
}

bool FreeflyerSystem::open_serial()
{
  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  if (serial_fd_ < 0)
    return false;
  fcntl(serial_fd_, F_SETFL, O_NONBLOCK);

  termios tty{};
  tcgetattr(serial_fd_, &tty);

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag = 0;

  tcsetattr(serial_fd_, TCSANOW, &tty);
  return true;
}

void FreeflyerSystem::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void FreeflyerSystem::send_neutral()
{
  if (serial_fd_ < 0) return;

  char buf[128];
  std::snprintf(buf, sizeof(buf),
    "$CMD,0.0,0.0,0.0,0.0,0\n");
  ::write(serial_fd_, buf, std::strlen(buf));
}

}  // namespace freeflyer_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  freeflyer_hardware::FreeflyerSystem,
  hardware_interface::SystemInterface)
