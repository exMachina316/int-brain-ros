#include "int_brain_hardware/actuator_system.hpp"
#include "int_brain_hardware/battery.hpp"
#include "int_brain_hardware/imu.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace int_brain_hardware
{
  hardware_interface::CallbackReturn IntBrainHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    cfg_.device_addr = info_.hardware_parameters["device_addr"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.is_feedforward_ = info_.hardware_parameters["is_feedforward"] == "true" ? true : false;

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      const auto command_interface = joint.command_interfaces[0];

      RCLCPP_INFO(
          rclcpp::get_logger("IntBrainHardware"),
          "Joint '%s' has command interface '%s' found.",
          joint.name.c_str(), command_interface.name.c_str());

      if (command_interface.name != hardware_interface::HW_IF_VELOCITY)
      // {
      //   for (auto &motor : motors)
      //   {
      //     if (motor.name_ == joint.name)
      //     {
      //       double min_effort = std::stod(command_interface.parameters.at("min"));
      //       double max_effort = std::stod(command_interface.parameters.at("max"));
      //       motor.setup(joint.name, min_effort, max_effort);

      //       RCLCPP_INFO(
      //           rclcpp::get_logger("IntBrainHardware"),
      //           "Joint '%s' effort interface has min: %f and max: %f",
      //           joint.name.c_str(), min_effort, max_effort);
      //     }
      //   }
      // }
      // else
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IntBrainHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            command_interface.name.c_str(), hardware_interface::HW_IF_VELOCITY);

        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> IntBrainHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Add motor state interfaces
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors[i].pos_));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors[i].vel_));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_CURRENT, &motors[i].current_));
    }

    // Add IMU state interfaces
    for (auto &sensor : info_.sensors)
    {
      if (sensor.name == "imu_sensor")
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "orientation.x", &imu_.orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "orientation.y", &imu_.orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "orientation.z", &imu_.orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "orientation.w", &imu_.orientation_[3]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "angular_velocity.x", &imu_.angular_velocity_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "angular_velocity.y", &imu_.angular_velocity_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "angular_velocity.z", &imu_.angular_velocity_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "linear_acceleration.x", &imu_.linear_acceleration_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "linear_acceleration.y", &imu_.linear_acceleration_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "linear_acceleration.z", &imu_.linear_acceleration_[2]));
      }
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "battery_voltage", "battery_voltage", &battery_.voltage_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> IntBrainHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Add motor command interfaces
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors[i].rpm_desired_));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn IntBrainHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device_addr, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn IntBrainHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Cleaning up ...please wait...");

    motors[0].reset_state();
    motors[1].reset_state();
    motors[2].reset_state();
    motors[3].reset_state();

    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn IntBrainHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn IntBrainHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Deactivating ...please wait...");

    // Stop motors
    if (comms_.connected())
    {
      comms_.disconnect();
    }

    RCLCPP_INFO(rclcpp::get_logger("IntBrainHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type IntBrainHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // Read IMU data
    std::vector<float> imu_data;
    if (comms_.req_data(REQUEST_IMU_RAW, imu_data) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IntBrainHardware"), "Failed to read IMU data");
      return hardware_interface::return_type::ERROR;
    }
    imu_.linear_acceleration_[0] = imu_data[0];
    imu_.linear_acceleration_[1] = imu_data[1];
    imu_.linear_acceleration_[2] = imu_data[2];
    imu_.angular_velocity_[0] = imu_data[3];
    imu_.angular_velocity_[1] = imu_data[4];
    imu_.angular_velocity_[2] = imu_data[5];

    imu_data.clear();
    if (comms_.req_data(REQUEST_IMU_PROCESSED, imu_data) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IntBrainHardware"), "Failed to read IMU data");
      return hardware_interface::return_type::ERROR;
    }
    imu_.orientation_[0] = imu_data[0];
    imu_.orientation_[1] = imu_data[1];
    imu_.orientation_[2] = imu_data[2];
    imu_.orientation_[3] = imu_data[3];

    // Read encoder positions
    std::vector<int64_t> encoder_positions;
    if (comms_.req_data(REQUEST_ENCODER_POSITIONS, encoder_positions) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IntBrainHardware"), "Failed to read encoder positions");
      return hardware_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      if (cfg_.is_feedforward_)
      {
        motors[i].pos_ += motors[i].vel_ * period.seconds();
      }
      else
      {
        motors[i].pos_ = encoder_positions[i];
      }
    }

    // Read encoder velocities
    std::vector<float> encoder_velocities;
    if (comms_.req_data(REQUEST_ENCODER_VELOCITIES, encoder_velocities) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IntBrainHardware"), "Failed to read encoder velocities");
      return hardware_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      if (cfg_.is_feedforward_)
      {
        motors[i].vel_ = motors[i].rpm_desired_;
      }
      else
      {
        motors[i].vel_ = encoder_velocities[i];
      }
    }

    // // Read motor currents
    // std::vector<float> motor_currents;
    // if (comms_.send_msg(REQUEST_MOTOR_CURRENT, motor_currents) != 0)
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("IntBrainHardware"), "Failed to read motor currents");
    //     return hardware_interface::return_type::ERROR;
    // }
    // for (size_t i = 0; i < info_.joints.size(); ++i)
    // {
    //     motors[i].current_ = motor_currents[i];
    // }

    // // Read battery voltage
    // std::vector<float> battery_voltage;
    // if (comms_.send_msg(REQUEST_BATTERY_VOLTAGE, battery_voltage) != 0)
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("IntBrainHardware"), "Failed to read battery voltage");
    //     return hardware_interface::return_type::ERROR;
    // }
    // battery_.voltage_ = battery_voltage[0];

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type int_brain_hardware ::IntBrainHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // Prepare motor commands
    std::vector<float> motor_commands;
    for (const auto &motor : motors)
    {
      motor_commands.push_back(motor.rpm_desired_);
      RCLCPP_DEBUG(
          rclcpp::get_logger("IntBrainHardware"),
          "Writing motor commands: %s",
          std::to_string(motor.rpm_desired_).c_str());
    }

    if (comms_.send_data(MOTOR_DESIRED_RPMS, motor_commands) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IntBrainHardware"), "Failed to write motor commands");
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

} // namespace int_brain_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    int_brain_hardware::IntBrainHardware, hardware_interface::SystemInterface)
