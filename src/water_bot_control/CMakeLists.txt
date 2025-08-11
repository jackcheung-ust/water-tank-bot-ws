cmake_minimum_required(VERSION 3.16)
project(water_bot_control LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find libserial using PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

## COMPILE
add_library(
  water_bot_control
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(water_bot_control PUBLIC cxx_std_17)
target_include_directories(water_bot_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/water_bot_control>
${LIBSERIAL_INCLUDE_DIRS}
)
target_link_libraries(water_bot_control PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${LIBSERIAL_LIBRARIES}
)

# Add compiler flags for libserial
target_compile_options(water_bot_control PUBLIC ${LIBSERIAL_CFLAGS_OTHER})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface water_bot_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/water_bot_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/water_bot_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/water_bot_control
)
install(TARGETS water_bot_control
  EXPORT export_water_bot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_water_bot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
