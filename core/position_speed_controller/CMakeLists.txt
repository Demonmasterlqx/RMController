cmake_minimum_required(VERSION 3.22)
project(position_speed_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_geometry_msgs
  rm_controller_interface
  tf2_msgs
)

option(DEBUG "启动debug发布者" ON)

add_subdirectory(../../submodule_lib/pid_controller ${CMAKE_CURRENT_BINARY_DIR}/pid_controller_build)
add_subdirectory(../../submodule_lib/watchdog ${CMAKE_CURRENT_BINARY_DIR}/watchdog_build)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


generate_parameter_library(position_speed_controller_parameters
  src/position_speed_controller.yaml
)

file(GLOB_RECURSE SOURCES "src/*.cpp")

message(${SOURCES})

add_library(position_speed_controller SHARED ${SOURCES})

if(DEBUG)
    target_compile_definitions(position_speed_controller PRIVATE DEBUG)
    message("Debug mode is ON")
else()
    message("Debug mode is OFF")
endif()

target_include_directories(position_speed_controller
  PUBLIC
  include
)

target_compile_features(position_speed_controller PUBLIC cxx_std_17)
target_include_directories(position_speed_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(position_speed_controller
  position_speed_controller_parameters
  rm_pid_controller
  rm_watch_dog
)

ament_target_dependencies(position_speed_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(position_speed_controller Eigen3::Eigen)


ament_export_include_directories(
  include
)

pluginlib_export_plugin_description_file(controller_interface position_speed_controller.xml)

install(
  TARGETS position_speed_controller_parameters position_speed_controller
  EXPORT position_speed_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
