cmake_minimum_required(VERSION 3.8)
project(wheel_leg_bridge)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wheel_leg_common REQUIRED)
find_package(wheel_leg_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/message_conversions.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  std_msgs
  wheel_leg_common
  wheel_leg_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs std_msgs wheel_leg_common wheel_leg_msgs)
ament_export_include_directories(include)
ament_package()
