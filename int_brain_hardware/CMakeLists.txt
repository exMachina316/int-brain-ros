cmake_minimum_required(VERSION 3.8)
project(int_brain_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  botspeak
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

## COMPILE
add_library(
  int_brain_hardware
  SHARED
  src/actuator_system.cpp
  src/mcu_comms.cpp
  src/motor.cpp
)
target_compile_features(int_brain_hardware PUBLIC cxx_std_20)
target_include_directories(int_brain_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/int_brain_hardware>
)
ament_target_dependencies(
  int_brain_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(int_brain_hardware PUBLIC botspeak serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface int_brain_hardware.xml) 

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS int_brain_hardware
  EXPORT export_int_brain_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_int_brain_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
