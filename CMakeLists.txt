cmake_minimum_required(VERSION 3.16)
project(diffbot_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  imu_sensor_broadcaster
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  std_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  diffbot_control
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(diffbot_control PUBLIC cxx_std_17)
target_include_directories(diffbot_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diffbot_control>
)
ament_target_dependencies(
  diffbot_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(diffbot_control PUBLIC serial)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffbot_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffbot_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/diffbot_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffbot_control
)
install(TARGETS diffbot_control
  EXPORT export_diffbot_control
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
ament_export_targets(export_diffbot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
