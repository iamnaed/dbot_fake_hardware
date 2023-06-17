#include <chrono>
#include <memory>
#include <vector>

#include "dbot_fake_hardware/dbot_fake_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dbot_fake_hardware
{
    // Initialize
    hardware_interface::CallbackReturn DbotFakeHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize storage variables
        state_positions_.resize(info_.joints.size(), 0.0);
        state_velocities_.resize(info_.joints.size(), 0.0);
        state_accelerations_.resize(info_.joints.size(), 0.0);
        cmd_positions_.resize(info_.joints.size(), 0.0);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Establish communication
    hardware_interface::CallbackReturn DbotFakeHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Establishing communications to motor controllers...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Communication opened successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Disconnect communications
    hardware_interface::CallbackReturn DbotFakeHardware::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Disconnecting communications to motor controllers...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Communication disconected successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Engage actuators
    hardware_interface::CallbackReturn DbotFakeHardware::on_activate(const rclcpp_lifecycle::State& previous_state) 
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Engaging motors...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Motors engaged successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Disengage actuators
    hardware_interface::CallbackReturn DbotFakeHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state) 
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Disengaging motors...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Motors disengaged successfully. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Cleanup
    hardware_interface::CallbackReturn DbotFakeHardware::on_shutdown(const rclcpp_lifecycle::State& previous_state) 
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Shutting down.. please wait...");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Shut down successfull. . .");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Export state interfaces
    std::vector<hardware_interface::StateInterface> DbotFakeHardware::export_state_interfaces() 
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            // State Position
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_positions_[i]));

            // State Velocity
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocities_[i]));

            // State Acceleration
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &state_accelerations_[i]));
        }
        return state_interfaces;
    }

    // Export command interfaces
    std::vector<hardware_interface::CommandInterface> DbotFakeHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            // Position command interface
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_positions_[i]));
        }
        return command_interfaces;
    }

    // Read data
    hardware_interface::return_type DbotFakeHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        (void)time;
        // Positions and velocities
        auto len = info_.joints.size();
        for (auto i = 0u; i < len; i++)
        {
            // Previous position
            double prev_pos = state_positions_[i];
            double prev_vel = state_velocities_[i];
            double deltaSeconds = period.seconds();

            // Set Position
            state_positions_[i] = cmd_positions_[i];

            // Set Velocity
            double ds = state_positions_[i] - prev_pos;
            state_velocities_[i] = ds / deltaSeconds;

            // Set Acceleration
            double dv = state_velocities_[i] - prev_vel;
            state_accelerations_[i] = dv / deltaSeconds;
        }
        return hardware_interface::return_type::OK;
    }

    // Write data
    hardware_interface::return_type DbotFakeHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        (void)time;
        (void)period;
        //RCLCPP_INFO(rclcpp::get_logger("DbotFakeHardware"), "Sending command. . .Time: %.1fs", period.seconds());
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dbot_fake_hardware::DbotFakeHardware, hardware_interface::SystemInterface)