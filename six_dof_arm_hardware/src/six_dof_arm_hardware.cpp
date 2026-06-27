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
        teensy_connected = true;
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
        
        if (teensy_connected)
        {
            //Setting up serial port. Using tutorial from mbedded ninja, use for reference
            serial_port = open("/dev/ttyUSB0", O_RDWR);

            // Check for errors
            if (serial_port < 0) {
                printf("Error %i from open: %s\n", errno, strerror(errno));
                return 1;
            }

            struct termios tty;

            if(tcgetattr(serial_port, &tty) != 0) {
                printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            }

            struct termios {
                tcflag_t c_iflag;    /* input mode flags */
                tcflag_t c_oflag;    /* output mode flags */
                tcflag_t c_cflag;    /* control mode flags */
                tcflag_t c_lflag;    /* local mode flags */
                cc_t c_line;      /* line discipline */
                cc_t c_cc[NCCS];    /* control characters */
                };
            
            //Set Control Modes (c_cflags)
            tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
            tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used
            tty.c_cflag &= ~CSIZE; // Clear all the size bits
            tty.c_cflag |= CS8; // 8 bits per byte
            tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control !!Come back to this!!

            //Settings Local Modes (c_lflags)
            tty.c_lflag &= ~ICANON;
            tty.c_lflag &= ~ECHO; // Disable echo
            tty.c_lflag &= ~ECHOE; // Disable erasure
            tty.c_lflag &= ~ECHONL; // Disable new-line echo
            tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

            //Settings Input Modes
            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
            tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
            
            //Settings Output Modes
            tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

            //VMIN and VTIME
            tty.c_cc[VTIME] = 0;    // Wait for 30bytes to be recieved, each of our commands should be 30 bytes
            tty.c_cc[VMIN] = 30;

            //Baud Rate TODO Read further into for speciffic design choice
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);

            //Saving Terminos
            if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            }

            if (!sync_packet()) { //sync the packets to start with start Byte and be correct length
                RCLCPP_ERROR(rclcpp::get_logger("SixDofArmHardware"), "Failed to sync with Teensy, aborting activation");
                return hardware_interface::CallbackReturn::ERROR;
            }


        }


        return hardware_interface::CallbackReturn::SUCCESS;
    }

     hardware_interface::CallbackReturn SixDofArmHardware::on_deactivate(
      const rclcpp_lifecycle::State & )
    {
        if (teensy_connected){
            close(serial_port);
        }
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
    SixDofArmHardware::read(const rclcpp::Time &, const rclcpp::Duration & ) //reasding joint angles from teensey
    {
        if (!teensy_connected) {
            return hardware_interface::return_type::OK;
        }

        if (teensy_connected) {
            uint8_t buffer[30];
            int n = ::read(serial_port, buffer, 30);

            if (n != 30) { //check length of data packet
                RCLCPP_WARN(rclcpp::get_logger("SixDofArmHardware"), "Incomplete packet, discarding");
                return hardware_interface::return_type::OK;
            }

            if (buffer[0] != START_BYTE) { //check if start byte is correct
                RCLCPP_WARN(rclcpp::get_logger("SixDofArmHardware"), "Bad start byte, discarding packet");
                return hardware_interface::return_type::OK;
            }

            if (buffer[29] != checksum(buffer)) { //check the checksum byte
                RCLCPP_WARN(rclcpp::get_logger("SixDofArmHardware"), "Bad checksum, discarding packet");
                return hardware_interface::return_type::OK;
            }

            for (size_t i = 0; i < info_.joints.size(); i++)
            {
                float position;
                memcpy(&position, &buffer[1 + i*4], 4);
                hw_states_positions_[i] = position;
            }
        }

        return hardware_interface::return_type::OK;
        
    }

    hardware_interface::return_type 
    SixDofArmHardware::write(const rclcpp::Time &, const rclcpp::Duration & ) //writing joint angles to teensy
    {
        if (!teensy_connected) {
            return hardware_interface::return_type::OK;
        }

        if (teensy_connected){

            uint8_t buffer[30];
            buffer[0] = START_BYTE;
            for (size_t i = 0; i<info_.joints.size(); i++)
            {
                float position;
                position = hw_commands_positions_[i];
                memcpy(&buffer[1+i*4], &position, 4);
            }
            
            buffer[29] = checksum(buffer);
            ::write(serial_port, buffer, 30);
        }
        return hardware_interface::return_type::OK;
        
    }

    uint8_t SixDofArmHardware::checksum(uint8_t buffer[30])
    {
        uint8_t checksum_value = 0;
        for (size_t i = 0; i < 29; i++) {
            checksum_value += buffer[i];
        }
        return checksum_value;
    }

    bool SixDofArmHardware::sync_packet(){

        uint8_t start_byte_check = 0x00;

        auto start_time = std::chrono::steady_clock::now();

        while(start_byte_check != START_BYTE)
        {
            ::read(serial_port, &start_byte_check, 1);
            
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > std::chrono::seconds(5))
            {
                RCLCPP_ERROR(rclcpp::get_logger("SixDofArmHardware"), "Sync timeout, no Teensy response");
                return false;
            }
        }

            uint8_t buffer[30];
            memcpy(&buffer[0], &start_byte_check, 1);      // copy start byte into position 0
            int n = ::read(serial_port, &buffer[1], 29);   // read remaining 29 bytes starting at index 1

            if (n != 29) { //check length of data packet
                RCLCPP_WARN(rclcpp::get_logger("SixDofArmHardware"), "Incomplete packet, discarding");
                return false;
            }

            if (buffer[29] != checksum(buffer)) { //check the checksum byte
                RCLCPP_WARN(rclcpp::get_logger("SixDofArmHardware"), "Bad checksum, discarding packet");
                return false;            }

        
        RCLCPP_INFO(rclcpp::get_logger("SixDofArmHardware"), "Synced to Teensy packet stream!");
        return true;
    }

}

PLUGINLIB_EXPORT_CLASS(
    six_dof_arm_hardware::SixDofArmHardware,
    hardware_interface::SystemInterface)