#ifndef DBOT_FAKE_HARDWARE__DBOT_FAKE_HARDWARE_HPP_
#define DBOT_FAKE_HARDWARE__DBOT_FAKE_HARDWARE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

namespace dbot_fake_hardware
{

    class DbotFakeHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DbotFakeHardware);

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

        DBOT_FAKE_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        DBOT_FAKE_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        DBOT_FAKE_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Parameters for Dbot Fake Hardware Simulation
        double time_to_start_sec_;
        double time_to_close_sec_;

        // Store commands
        std::vector<double> cmd_positions_;
        std::vector<double> state_positions_;
        std::vector<double> state_velocities_;

        // Storage
        std::vector<double> state_positions_previous_;
    };
}


#endif  // DBOT_FAKE_HARDWARE__DBOT_FAKE_HARDWARE_HPP_