#ifndef CUSTOM_EFFORT_CONTROLLERS__SYSTEM_IDENTIFIER_HPP_
#define CUSTOM_EFFORT_CONTROLLERS__SYSTEM_IDENTIFIER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64.hpp"

namespace custom_effort_controllers
{
  // Using a semantic alias for the command message
  using Command = std_msgs::msg::Float64;
  using Response = std_msgs::msg::Float64;

  class SystemIdentifier : public controller_interface::ControllerInterface
  {
  public:
    SystemIdentifier();

    // Standard ControllerInterface lifecycle methods
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    // Configuration for command and state interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // Main control loop
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // The name of the joint this controller controls
    std::string joint_name_;

    // Command subscriber and buffer
    rclcpp::Subscription<Command>::SharedPtr command_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<Command>> command_buffer_;

    // Response publisher
    rclcpp::Publisher<Response>::SharedPtr response_publisher_ = nullptr;
  };

} // namespace custom_effort_controllers

#endif // CUSTOM_EFFORT_CONTROLLERS__SYSTEM_IDENTIFIER_HPP_
