#include "six_dof_arm_hardware/six_dof_arm_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace six_dof_arm_hardware
{
    hardware_interface::CallbackReturn SixDofArmHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // rest stays the same — info_ still populated by base class
        hw_commands_positions_.resize(info_.joints.size(), 0.0);
        hw_states_positions_.resize(info_.joints.size(), 0.0);
        hw_states_velocities_.resize(info_.joints.size(), 0.0);
        steps_per_rad_.resize(info_.joints.size(), 0.0);
        hw_steps_.resize(info_.joints.size(), 0.0);
        double steps_per_rev = 200.0;
        double microstepping = 8.0;
        std::vector<double> joint_multiplier = {1.0, 2.0, 3.0, 2.0, 5.0, 8.0, 2.0};  // per-joint gear ratios

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            steps_per_rad_[i] = (steps_per_rev * joint_multiplier[i] * microstepping) / (2.0 * 3.14);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SixDofArmHardware::on_activate(
        const rclcpp_lifecycle::State & )
    {
        // Set all states and commands to a safe starting value (0.0)
        for (size_t i = 0; i < hw_states_positions_.size(); i++)  //TODO: Use Encoder positions rather than 0 positions
        {
            hw_states_positions_[i] = 0.0;
            hw_states_velocities_[i] = 0.0;
            hw_commands_positions_[i] = 0.0;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

     hardware_interface::CallbackReturn SixDofArmHardware::on_deactivate(
      const rclcpp_lifecycle::State & )
    {
     return hardware_interface::CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface>
    SixDofArmHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            // Register this joint's POSITION state
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name,                    // joint name
                    hardware_interface::HW_IF_POSITION,      // "position"
                    &hw_states_positions_[i]));               // pointer to the data

            // Register this joint's VELOCITY state
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_VELOCITY,
                    &hw_states_velocities_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SixDofArmHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &hw_commands_positions_[i]));    // ← command vector, not state
        }

        return command_interfaces;
    }

    hardware_interface::return_type 
    SixDofArmHardware::read(const rclcpp::Time &, const rclcpp::Duration & )
    {
           
            for (size_t i = 0; i<info_.joints.size(); i++)
            {
                hw_states_positions_[i] = hw_steps_[i]/steps_per_rad_[i];
                hw_states_velocities_[i] = 0.0;
            }
        return hardware_interface::return_type::OK;
        
    }
    hardware_interface::return_type 
    SixDofArmHardware::write(const rclcpp::Time &, const rclcpp::Duration & )
    {
            for (size_t i = 0; i<info_.joints.size(); i++)
            {
                hw_steps_[i] = hw_commands_positions_[i]*steps_per_rad_[i];
            }
        return hardware_interface::return_type::OK;
        
    }

}

PLUGINLIB_EXPORT_CLASS(
    six_dof_arm_hardware::SixDofArmHardware,
    hardware_interface::SystemInterface)