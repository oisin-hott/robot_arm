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

private:

    std::vector<double> hw_commands_positions_;    // position commands IN
    std::vector<double> hw_states_positions_;      // position states OUT
    std::vector<double> hw_states_velocities_;     // velocity states OUT
    std::vector<double> steps_per_rad_;            // conversion factors
    std::vector<double> hw_steps_;

};



}  // namespace six_dof_arm_hardware