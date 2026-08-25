cmake_minimum_required(VERSION 3.8)
project(wheel_leg_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wheel_leg_core REQUIRED)
find_package(wheel_leg_msgs REQUIRED)

add_library(wheel_leg_conversions src/conversions.cpp)
target_include_directories(wheel_leg_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(wheel_leg_conversions wheel_leg_msgs)
target_link_libraries(wheel_leg_conversions wheel_leg_core::wheel_leg_core)

add_library(wheel_leg_controller_node src/controller_node.cpp)
target_include_directories(wheel_leg_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(wheel_leg_controller_node rclcpp wheel_leg_msgs)
target_link_libraries(
  wheel_leg_controller_node
  wheel_leg_conversions wheel_leg_core::wheel_leg_core)

add_executable(controller_node src/main.cpp)
target_link_libraries(controller_node wheel_leg_controller_node)
ament_target_dependencies(controller_node rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions wheel_leg_conversions)
  ament_add_gtest(test_pubsub test/test_pubsub.cpp)
  target_link_libraries(test_pubsub wheel_leg_controller_node)
  ament_target_dependencies(test_pubsub rclcpp wheel_leg_msgs)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  wheel_leg_conversions wheel_leg_controller_node controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
