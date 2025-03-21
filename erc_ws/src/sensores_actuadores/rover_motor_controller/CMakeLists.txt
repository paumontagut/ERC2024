cmake_minimum_required(VERSION 3.8)
project(rover_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(motor_controller src/motor_controller.cpp)
add_executable(motor_vel_controller src/motor_vel_controller.cpp)
add_executable(zed2_rotation src/zed2_rotation.cpp)

ament_target_dependencies(motor_controller 
  rclcpp
  dynamixel_sdk
  custom_interfaces
)
ament_target_dependencies(motor_vel_controller 
  rclcpp
  dynamixel_sdk
  custom_interfaces
  geometry_msgs
  std_msgs
)
ament_target_dependencies(zed2_rotation 
  rclcpp
  dynamixel_sdk
  custom_interfaces
  geometry_msgs
  std_msgs
)

install(TARGETS
  motor_controller
  motor_vel_controller
  zed2_rotation
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
