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
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.is_feedforward_ = info_.hardware_parameters["is_feedforward"] == "true" ? true : false;

    // IMU settings
    cfg_.imu_usage = info_.hardware_parameters["imu_usage"] == "true";
    cfg_.imu_sensor_fusion = info_.hardware_parameters["imu_sensor_fusion"] == "true";
    cfg_.imu_sensor_fusion_frequency = std::stoul(info_.hardware_parameters["imu_sensor_fusion_frequency"]);

    // Encoder settings
    cfg_.encoder_usage = info_.hardware_parameters["encoder_usage"] == "true";
    cfg_.encoder_velocity_calculation = info_.hardware_parameters["encoder_velocity_calculation"] == "true";
    cfg_.encoder_velocity_calculation_frequency = std::stoul(info_.hardware_parameters["encoder_velocity_calculation_frequency"]);
    cfg_.encoder_cpr = std::stoul(info_.hardware_parameters["encoder_cpr"]);

    // Motor current measurement settings
    cfg_.motor_current_meas_on_off = info_.hardware_parameters["motor_current_meas_on_off"] == "true";
    cfg_.motor_current_meas_frequency = std::stoul(info_.hardware_parameters["motor_current_meas_frequency"]);

    // Battery voltage measurement settings
    cfg_.battery_voltage_meas_on_off = info_.hardware_parameters["battery_voltage_meas_on_off"] == "true";
    cfg_.battery_voltage_meas_rate = std::stoul(info_.hardware_parameters["battery_voltage_meas_rate"]);

    // Motor closed loop control settings
    cfg_.closed_loop_control = info_.hardware_parameters["closed_loop_control"] == "true";
    cfg_.closed_loop_frequency = std::stoul(info_.hardware_parameters["closed_loop_frequency"]);

    // Initialize motors
    int index=0;
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      const auto command_interface = joint.command_interfaces[0];

      RCLCPP_INFO(
          rclcpp::get_logger("IntBrainHardware"),
          "Joint '%s' has command interface '%s' found.",
          joint.name.c_str(), command_interface.name.c_str());

      if (command_interface.name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IntBrainHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            command_interface.name.c_str(), hardware_interface::HW_IF_VELOCITY);

        return hardware_interface::CallbackReturn::ERROR;
      }

      float kp = std::stof(joint.parameters.at("kp"));
      float ki = std::stof(joint.parameters.at("ki"));
      float kd = std::stof(joint.parameters.at("kd"));
      float kd_filter_coeff = std::stof(joint.parameters.at("kd_filter_coeff"));
      float ff0 = std::stof(joint.parameters.at("ff0"));
      float ff1 = std::stof(joint.parameters.at("ff1"));
      motors[index++].setup(joint.name, kp, ki, kd, kd_filter_coeff, ff0, ff1);
      RCLCPP_INFO(
          rclcpp::get_logger("IntBrainHardware"),
          "Motor '%s' initialized with kp: %f, ki: %f, kd: %f, ff0: %f, ff1: %f",
          joint.name.c_str(), kp, ki, kd, ff0, ff1);
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
            sensor.name, "orientation.w", &imu_.orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "orientation.x", &imu_.orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "orientation.y", &imu_.orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            sensor.name, "orientation.z", &imu_.orientation_[3]));

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
    if (!comms_.connect(cfg_.device_addr, cfg_.timeout_ms)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to connect to INT BRAIN at %s",
          cfg_.device_addr.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    };

    // -------------------  Send Configurations --------------------

    // IMU settings
    std::vector<bool> imu_usage = {cfg_.imu_usage};
    if (comms_.send_config(IMU_USAGE, imu_usage) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set closed loop control on/off");
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<bool> imu_sensor_fusion = {cfg_.imu_sensor_fusion};
    if (comms_.send_config(IMU_SENSOR_FUSION, imu_sensor_fusion) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set IMU sensor fusion");
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<uint32_t> imu_sensor_fusion_frequency = {cfg_.imu_sensor_fusion_frequency};
    if (comms_.send_config(IMU_SENSOR_FUSION_FREQUENCY, imu_sensor_fusion_frequency) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set IMU sensor fusion frequency");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Encoder settings
    std::vector<bool> encoder_usage = {cfg_.encoder_usage};
    if (comms_.send_config(ENCODER_USAGE, encoder_usage) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set encoder usage");
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<bool> encoder_velocity_calculation = {cfg_.encoder_velocity_calculation};
    if (comms_.send_config(ENCODER_VELOCITY_CALCULATION, encoder_velocity_calculation) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set encoder velocity calculation");
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<uint32_t> encoder_velocity_calculation_frequency = {cfg_.encoder_velocity_calculation_frequency};
    if (comms_.send_config(ENCODER_VELOCITY_CALCULATION_FREQUENCY, encoder_velocity_calculation_frequency) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set encoder velocity calculation frequency");
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<uint32_t> encoder_cpr = {cfg_.encoder_cpr};
    if (comms_.send_config(ENCODER_CPR, encoder_cpr) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set encoder CPR");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Motor current measurement settings
    std::vector<bool> motor_current_meas_on_off = {cfg_.motor_current_meas_on_off};
    if (comms_.send_config(MOTOR_CURRENT_MEAS_ON_OFF, motor_current_meas_on_off) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set motor current measurement on/off");
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<uint32_t> motor_current_meas_frequency = {cfg_.motor_current_meas_frequency};
    if (comms_.send_config(MOTOR_CURRENT_MEAS_FREQUENCY, motor_current_meas_frequency) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set motor current measurement frequency");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Battery voltage measurement settings
    // // TODO (eccentricOrange): Battery config parsing is not implemented in the STM32 firmware yet
    // 
    // std::vector<bool> battery_voltage_meas_on_off = {false};
    // if (comms_.send_config(BATTERY_VOLTAGE_MEAS_ON_OFF, battery_voltage_meas_on_off) != 0) {
    //   RCLCPP_ERROR(
    //       rclcpp::get_logger("IntBrainHardware"),
    //       "Failed to set battery voltage measurement on/off");
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // std::vector<uint32_t> battery_voltage_meas_rate = {200};
    // if (comms_.send_config(BATTERY_VOLTAGE_MEAS_RATE, battery_voltage_meas_rate) != 0) {
    //   RCLCPP_ERROR(
    //       rclcpp::get_logger("IntBrainHardware"),
    //       "Failed to set battery voltage measurement rate");
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // Motor closed loop control settings
    std::vector<bool> closed_loop_control = {cfg_.closed_loop_control};
    if (comms_.send_config(MOTOR_CLOSED_LOOP_ON_OFF, closed_loop_control) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set closed loop control on/off");
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<uint32_t> closed_loop_frequency = {cfg_.closed_loop_frequency};
    if (comms_.send_config(MOTOR_CLOSED_LOOP_FREQUENCY, closed_loop_frequency) != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("IntBrainHardware"),
          "Failed to set closed loop control frequency");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Set motor-wise closed loop control parameters
    int index = 0;
    for (auto &motor : motors) {
      std::vector<float> kp_param;
      kp_param.push_back((float)index);
      kp_param.push_back(motor.kp_);
      if (comms_.send_config(MOTOR_CLOSED_LOOP_KP, kp_param) != 0) {
        RCLCPP_INFO(
            rclcpp::get_logger("IntBrainHardware"),
            "Failed to set Kp for Motor%d: %f", (int)kp_param[0], kp_param[1]);
      }

      std::vector<float> ki_param;
      ki_param.push_back((float)index);
      ki_param.push_back(motor.ki_);
      if (comms_.send_config(MOTOR_CLOSED_LOOP_KI, ki_param) != 0) {
        RCLCPP_INFO(
            rclcpp::get_logger("IntBrainHardware"),
            "Failed to set Ki for Motor%d: %f", (int)ki_param[0], ki_param[1]);
      }

      std::vector<float> kd_param;
      kd_param.push_back((float)index);
      kd_param.push_back(motor.kd_);
      if (comms_.send_config(MOTOR_CLOSED_LOOP_KD, kd_param) != 0) {
        RCLCPP_INFO(
            rclcpp::get_logger("IntBrainHardware"),
            "Failed to set Kd for Motor%d: %f", (int)kd_param[0], kd_param[1]);
      }

      std::vector<float> kd_filter_coeff_param;
      kd_filter_coeff_param.push_back((float)index);
      kd_filter_coeff_param.push_back(motor.kd_filter_coeff_);
      if (comms_.send_config(MOTOR_CLOSED_LOOP_FILTER_COEFF, kd_filter_coeff_param) != 0) {
        RCLCPP_INFO(
            rclcpp::get_logger("IntBrainHardware"),
            "Failed to set Kd filter coeff for Motor%d: %f", (int)kd_filter_coeff_param[0], kd_filter_coeff_param[1]);
      }

      std::vector<float> ff_param;
      ff_param.push_back((float)index);
      ff_param.push_back(motor.ff_param_[0]);
      ff_param.push_back(motor.ff_param_[1]);
      if (comms_.send_config(MOTOR_FEEDFORWARD_PARAM, ff_param) != 0) {
        RCLCPP_INFO(
            rclcpp::get_logger("IntBrainHardware"),
            "Failed to set FF Params for Motor%d: %f, %f", (int)ff_param[0], ff_param[1], ff_param[2]);
      }

      index++;
    }


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
        motors[i].set_encoder(encoder_positions[i]);
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
