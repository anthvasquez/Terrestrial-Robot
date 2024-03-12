cmake_minimum_required(VERSION 3.8)
project(terrestrial_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  terrestrial_robot
  SHARED
  src/hardware/BDC_LM298_SystemHardware.cpp
)
target_compile_features(terrestrial_robot PUBLIC cxx_std_17)
target_include_directories(terrestrial_robot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/hardware>
)
ament_target_dependencies(
  terrestrial_robot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface terrestrial_robot.xml)

# install hardware implementations
install(
  DIRECTORY include/
  DESTINATION include/terrestrial_robot
)
# install robot description and test launch files
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/terrestrial_robot
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
