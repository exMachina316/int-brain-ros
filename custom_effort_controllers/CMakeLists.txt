cmake_minimum_required(VERSION 3.8)
project(custom_effort_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  include
)

# Add the C++ source file to create a shared library
add_library(${PROJECT_NAME} SHARED
  src/system_identifier.cpp
)

# Link the library against its dependencies
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
)

# Install the library to the destination 'lib' directory
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the plugin description file
pluginlib_export_plugin_description_file(controller_interface system_identifier.xml)

# Install the plugin description file
install(FILES system_identifier.xml
  DESTINATION share/${PROJECT_NAME}
)

# Register the package with ament
ament_package()