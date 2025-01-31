cmake_minimum_required(VERSION 3.5)
project(unitree_lidar_ros2)

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

cmake_policy(SET CMP0074 NEW)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS} 
  include
  ../../drivers/unitree_lidar_sdk/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ../../drivers/unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}
)

add_definitions(${PCL_DEFINITIONS})

find_library(UNITREE_LIDAR_LIB libunitree_lidar_sdk PATHS ../../drivers/unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR})

add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

target_link_libraries(unitree_lidar_ros2_node
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  ${UNITREE_LIDAR_LIB}
)

ament_target_dependencies(
  unitree_lidar_ros2_node
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

# Install the node executable
install(TARGETS
  unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install rviz and launch files
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz/view.rviz
  ${CMAKE_CURRENT_SOURCE_DIR}/../../rover_bringup/launch/unitree_lidar.launch.py
  DESTINATION share/${PROJECT_NAME}
)

# Install the static library if needed
install(FILES ../../drivers/unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
