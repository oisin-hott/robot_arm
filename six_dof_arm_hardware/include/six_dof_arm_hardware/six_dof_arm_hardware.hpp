#pragma once

// ── Includes ──
#include <vector>
#include <string>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <cmath>

//Serial Port Includes
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

namespace six_dof_arm_hardware
{

class SixDofArmHardware : public hardware_interface::SystemInterface
{
public:
    // ── Lifecycle ──
hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    // ── Interface registration ──
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ── Per-cycle read/write ──
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    bool sync_packet();
    uint8_t checksum(uint8_t buffer[30]);

private:

    std::vector<double> hw_commands_positions_;    // position commands IN
    std::vector<double> hw_states_positions_;      // position states OUT
    std::vector<double> hw_states_velocities_;     // velocity states OUT - Here for ROS2 compliance, will always read 0
    std::vector<double> steps_per_rad_;            // conversion factors
    std::vector<double> hw_steps_;
    bool teensy_connected;
    bool synced;
    int serial_port;
    static constexpr uint8_t START_BYTE = 0xAA;

};



}  // namespace six_dof_arm_hardware