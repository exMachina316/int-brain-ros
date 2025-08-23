#ifndef int_brain_hardware__ACTUATOR_SYSTEM_HPP_
#define int_brain_hardware__ACTUATOR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "int_brain_hardware/arduino_comms.hpp"
#include "int_brain_hardware/motor.hpp"
#include "int_brain_hardware/battery.hpp"
#include "int_brain_hardware/imu.hpp"

#include "int_brain_messages.h"

namespace int_brain_hardware
{
  class IntBrainHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      std::string device_addr = "";
      int timeout_ms = 0;
      bool is_feedforward_ = false;

      // IMU settings
      bool imu_usage = false;
      bool imu_sensor_fusion = false;
      uint32_t imu_sensor_fusion_frequency = 0;

      // Encoder settings
      bool encoder_usage = false;
      bool encoder_velocity_calculation = false;
      uint32_t encoder_velocity_calculation_frequency = 0;
      uint32_t encoder_cpr = 0;

      // Motor current measurement settings
      bool motor_current_meas_on_off = false;
      uint32_t motor_current_meas_frequency = 0;

      // Battery voltage measurement settings
      bool battery_voltage_meas_on_off = false;
      uint32_t battery_voltage_meas_rate = 0;

      // Motor closed loop control settings
      MotorControllerMode_TypeDef motor_control_mode = PID_FEED_FORWARD;
      uint32_t motor_controller_frequency = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IntBrainHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    ArduinoComms comms_;
    Config cfg_;
    Motor motors[4];
    Imu imu_;
    Battery battery_;
  };

} // namespace int_brain_hardware

#endif // int_brain_hardware__ACTUATOR_SYSTEM_HPP_
