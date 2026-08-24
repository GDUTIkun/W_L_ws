cmake_minimum_required(VERSION 3.8)
project(wheel_leg_stm32_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NormalCommand.msg"
  "msg/NormalState.msg"
  "msg/IdentificationCommand.msg"
  "msg/IdentificationTelemetry.msg"
  "msg/BridgeStatus.msg"
  DEPENDENCIES builtin_interfaces
)

add_library(stm32_protocol
  src/protocol.cpp
  src/serial_port.cpp
)
target_include_directories(stm32_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(stm32_bridge_node src/stm32_bridge_node.cpp)
target_link_libraries(stm32_bridge_node stm32_protocol)
ament_target_dependencies(stm32_bridge_node rclcpp sensor_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(stm32_bridge_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  if(TARGET test_protocol)
    target_link_libraries(test_protocol stm32_protocol)
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS stm32_protocol stm32_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
