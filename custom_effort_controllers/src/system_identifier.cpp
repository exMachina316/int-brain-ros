#include "custom_effort_controllers/system_identifier.hpp"

#include <optional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace custom_effort_controllers
{

  // Controller constructor
  SystemIdentifier::SystemIdentifier()
      : controller_interface::ControllerInterface(),
        command_buffer_(std::make_shared<Command>()) // Initialize buffer with a command message
  {
  }

  // Initialization: Called when the controller is loaded.
  controller_interface::CallbackReturn SystemIdentifier::on_init()
  {
    RCLCPP_INFO(get_node()->get_logger(), "Initializing SystemIdentifier");
    try
    {
      // Automatically declare parameters for the joint name
      auto_declare<std::string>("joint", "");

      joint_name_ = get_node()->get_parameter("joint").as_string();
      if (joint_name_.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint' parameter not set");
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Configuration: Called before the controller is activated.
  controller_interface::CallbackReturn SystemIdentifier::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring controller for joint '%s'", joint_name_.c_str());

    // Set up the command subscriber
    // The topic name will be <controller_name>/command
    command_subscriber_ = get_node()->create_subscription<Command>(
        "~/command", 10, [this](const std::shared_ptr<Command> msg)
        { command_buffer_.writeFromNonRT(msg); });

    response_publisher_ = get_node()->create_publisher<Response>("~/response", rclcpp::SystemDefaultsQoS());

    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Specify the interfaces this controller commands (effort on one joint)
  controller_interface::InterfaceConfiguration SystemIdentifier::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    return conf;
  }

  // Specify the interfaces this controller reads (position from one joint)
  controller_interface::InterfaceConfiguration SystemIdentifier::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    return conf;
  }

  // Activation: Called when the controller is switched from 'inactive' to 'active'.
  controller_interface::CallbackReturn SystemIdentifier::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Activating controller");

    auto initial_command = std::make_shared<Command>();
    initial_command->data = 0.0;
    command_buffer_.writeFromNonRT(initial_command);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Deactivation: Called when the controller is switched from 'active' to 'inactive'.
  controller_interface::CallbackReturn SystemIdentifier::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller");
    // Set effort to 0 on deactivation
    for (auto &command_interface : command_interfaces_)
    {
      if (!command_interface.set_value(0.0))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value");
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // The main update loop, executed at the control frequency.
  controller_interface::return_type SystemIdentifier::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Get the latest command from the real-time buffer
    auto desired_effort_msg = command_buffer_.readFromRT();
    if (!desired_effort_msg || !(*desired_effort_msg))
    {
      // No new command received, do nothing
      return controller_interface::return_type::OK;
    }

    const double desired_effort = (*desired_effort_msg)->data;

    // Apply the computed effort to the command interface
    if (!command_interfaces_[0].set_value(desired_effort))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }

    auto optional_state = state_interfaces_[0].get_optional();
    if (optional_state.has_value())
    {
      auto response_msg = std::make_unique<Response>();
      response_msg->data = optional_state.value();
      response_publisher_->publish(std::move(response_msg));
    }
    else
    {
      RCLCPP_WARN(get_node()->get_logger(), "State interface has NaN value.");
    }

    return controller_interface::return_type::OK;
  }

  // Lifecycle callbacks for cleanup, error, and shutdown
  controller_interface::CallbackReturn SystemIdentifier::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn SystemIdentifier::on_error(
      const rclcpp_lifecycle::State &)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn SystemIdentifier::on_shutdown(
      const rclcpp_lifecycle::State &)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

} // namespace custom_effort_controllers

// Export the controller class as a plugin
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    custom_effort_controllers::SystemIdentifier,
    controller_interface::ControllerInterface)
