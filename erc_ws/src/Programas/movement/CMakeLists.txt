cmake_minimum_required(VERSION 3.5)
project(movement)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# Create a cpp executable
add_executable(ejemplo src/ejemplo.cpp)
ament_target_dependencies(ejemplo rclcpp)

add_executable(subscriber_member_function src/subscriber_member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp geometry_msgs)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop rclcpp custom_interfaces)

# Install cpp executables
install(TARGETS
  ejemplo
  subscriber_member_function
  keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})

# Install python modules
ament_python_install_package(${PROJECT_NAME})

install (PROGRAMS
  scripts/ejemplo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
