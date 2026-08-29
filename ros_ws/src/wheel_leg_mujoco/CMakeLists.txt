cmake_minimum_required(VERSION 3.16)
project(wheel_leg_mujoco LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wheel_leg_core REQUIRED)
find_package(wheel_leg_msgs REQUIRED)
find_package(wheel_leg_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS "$ENV{MUJOCO_DIR}/include" /opt/mujoco-3.7.0/include REQUIRED)
find_library(MUJOCO_LIBRARY mujoco
  HINTS "$ENV{MUJOCO_DIR}/lib" /opt/mujoco-3.7.0/lib REQUIRED)
add_library(MuJoCo::MuJoCo UNKNOWN IMPORTED)
set_target_properties(MuJoCo::MuJoCo PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")

add_library(wheel_leg_mujoco_adapter src/adapter.cpp)
target_include_directories(wheel_leg_mujoco_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  wheel_leg_mujoco_adapter PUBLIC
  MuJoCo::MuJoCo wheel_leg_core::wheel_leg_core)
target_compile_options(
  wheel_leg_mujoco_adapter PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(mujoco_node src/mujoco_node.cpp)
target_link_libraries(
  mujoco_node
  wheel_leg_mujoco_adapter wheel_leg_ros::wheel_leg_conversions)
ament_target_dependencies(mujoco_node rclcpp std_srvs wheel_leg_msgs)
target_compile_options(mujoco_node PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(deterministic_loop src/deterministic_loop.cpp)
target_link_libraries(deterministic_loop wheel_leg_mujoco_adapter)
target_compile_options(
  deterministic_loop PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(planar_standing_loop src/planar_standing_loop.cpp)
target_link_libraries(planar_standing_loop wheel_leg_mujoco_adapter)
target_compile_options(
  planar_standing_loop PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(standing_3d_loop src/standing_3d_loop.cpp)
target_link_libraries(standing_3d_loop wheel_leg_mujoco_adapter)
target_compile_options(
  standing_3d_loop PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(weighted_wbc_loop src/weighted_wbc_loop.cpp)
target_link_libraries(weighted_wbc_loop wheel_leg_mujoco_adapter PkgConfig::GLFW)
target_compile_options(weighted_wbc_loop PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_adapter test/test_adapter.cpp)
  target_link_libraries(test_adapter wheel_leg_mujoco_adapter)
  target_compile_definitions(test_adapter PRIVATE
    WHEEL_LEG_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../simulation/mujoco/model/scence.xml")
  target_compile_options(test_adapter PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../simulation/mujoco/model
  DESTINATION share/${PROJECT_NAME}/simulation)
install(TARGETS wheel_leg_mujoco_adapter mujoco_node deterministic_loop
  planar_standing_loop standing_3d_loop weighted_wbc_loop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
