cmake_minimum_required(VERSION 3.8)
project(swarm_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)

include_directories(
  include
)

# Shared Controller Implementation Library
add_library(drone_controller_lib SHARED
  src/drone_controller_node.cpp
  src/px4_interface.cpp
  src/peer_tracker.cpp
  src/cbf_avoidance.cpp
  src/task_allocator.cpp
  src/mission_executor.cpp
  src/mission_helix.cpp
  src/mission_triangle.cpp
  src/cbf_test.cpp
)
target_include_directories(drone_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(drone_controller_lib
  rclcpp
  rclcpp_action
  std_msgs
  px4_msgs
  swarm_interfaces
)

# 1. Generic Controller Executable
add_executable(drone_controller_node src/drone_controller_node.cpp)
target_link_libraries(drone_controller_node drone_controller_lib)
ament_target_dependencies(drone_controller_node rclcpp rclcpp_action std_msgs px4_msgs swarm_interfaces)

add_executable(drone_1_node src/drone_1_node.cpp)
target_link_libraries(drone_1_node drone_controller_lib)
ament_target_dependencies(drone_1_node rclcpp rclcpp_action std_msgs px4_msgs swarm_interfaces)

add_executable(drone_2_node src/drone_2_node.cpp)
target_link_libraries(drone_2_node drone_controller_lib)
ament_target_dependencies(drone_2_node rclcpp rclcpp_action std_msgs px4_msgs swarm_interfaces)

add_executable(drone_3_node src/drone_3_node.cpp)
target_link_libraries(drone_3_node drone_controller_lib)
ament_target_dependencies(drone_3_node rclcpp rclcpp_action std_msgs px4_msgs swarm_interfaces)

add_executable(drone_4_node src/drone_4_node.cpp)
target_link_libraries(drone_4_node drone_controller_lib)
ament_target_dependencies(drone_4_node rclcpp rclcpp_action std_msgs px4_msgs swarm_interfaces)

# 3. Swarm Arming Manager Executable
add_executable(swarm_arming_manager src/swarm_arming_manager.cpp)
target_include_directories(swarm_arming_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(swarm_arming_manager rclcpp std_msgs px4_msgs swarm_interfaces)

# 4. Practice Single Drone Controller Executable
add_executable(practice_controller include/swarm_core/practice/controller.cpp)
ament_target_dependencies(practice_controller rclcpp px4_msgs)

# Install Executables
install(TARGETS
  drone_controller_node
  drone_1_node
  drone_2_node
  drone_3_node
  drone_4_node
  swarm_arming_manager
  practice_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Header Files
install(DIRECTORY include/
  DESTINATION include
)

# Install Launch and Config Directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
  install(FILES README.md
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cbf_avoidance test/test_cbf_avoidance.cpp)
  set_tests_properties(test_cbf_avoidance PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=/tmp"
  )
  target_link_libraries(test_cbf_avoidance drone_controller_lib)
  ament_target_dependencies(test_cbf_avoidance
    rclcpp
    swarm_interfaces
  )
endif()

ament_package()
