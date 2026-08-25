cmake_minimum_required(VERSION 3.16)
project(wheel_leg_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(wheel_leg_core src/controller_core.cpp)
add_library(wheel_leg_core::wheel_leg_core ALIAS wheel_leg_core)
target_include_directories(wheel_leg_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(MSVC)
  target_compile_options(wheel_leg_core PRIVATE /W4 /WX)
else()
  target_compile_options(wheel_leg_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_contract test/test_contract.cpp)
  target_link_libraries(test_contract PRIVATE wheel_leg_core)
  if(MSVC)
    target_compile_options(test_contract PRIVATE /W4 /WX)
  else()
    target_compile_options(test_contract PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_contract COMMAND test_contract)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wheel_leg_core EXPORT export_wheel_leg_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_targets(export_wheel_leg_core HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_package()
endif()
