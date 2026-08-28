cmake_minimum_required(VERSION 3.16)
project(wheel_leg_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_library(wheel_leg_core
  src/controller_core.cpp
  src/dense_qp_solver.cpp
  src/nominal_wbc_model.cpp
  src/weighted_wbc_problem.cpp
  src/weighted_wbc_controller.cpp)
add_library(wheel_leg_core::wheel_leg_core ALIAS wheel_leg_core)
target_include_directories(wheel_leg_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wheel_leg_core PUBLIC Eigen3::Eigen)
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

  add_executable(test_dense_qp_solver test/test_dense_qp_solver.cpp)
  target_link_libraries(test_dense_qp_solver PRIVATE wheel_leg_core)
  if(MSVC)
    target_compile_options(test_dense_qp_solver PRIVATE /W4 /WX)
  else()
    target_compile_options(test_dense_qp_solver PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_dense_qp_solver COMMAND test_dense_qp_solver)

  add_executable(test_nominal_wbc_model test/test_nominal_wbc_model.cpp)
  target_link_libraries(test_nominal_wbc_model PRIVATE wheel_leg_core)
  target_compile_definitions(test_nominal_wbc_model PRIVATE
    WBC_MODEL_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/phase21_runtime_model_golden_v2.txt")
  if(MSVC)
    target_compile_options(test_nominal_wbc_model PRIVATE /W4 /WX)
  else()
    target_compile_options(test_nominal_wbc_model PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_nominal_wbc_model COMMAND test_nominal_wbc_model)

  add_executable(test_weighted_wbc_problem test/test_weighted_wbc_problem.cpp)
  target_link_libraries(test_weighted_wbc_problem PRIVATE wheel_leg_core)
  target_compile_definitions(test_weighted_wbc_problem PRIVATE
    WBC_PROBLEM_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/phase21_weighted_wbc_problem_golden_v2.txt")
  if(MSVC)
    target_compile_options(test_weighted_wbc_problem PRIVATE /W4 /WX)
  else()
    target_compile_options(test_weighted_wbc_problem PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_weighted_wbc_problem COMMAND test_weighted_wbc_problem)

  add_executable(test_weighted_wbc_controller test/test_weighted_wbc_controller.cpp)
  target_link_libraries(test_weighted_wbc_controller PRIVATE wheel_leg_core)
  target_compile_definitions(test_weighted_wbc_controller PRIVATE
    WBC_PROBLEM_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/phase21_weighted_wbc_problem_golden_v2.txt")
  if(MSVC)
    target_compile_options(test_weighted_wbc_controller PRIVATE /W4 /WX)
  else()
    target_compile_options(test_weighted_wbc_controller PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_weighted_wbc_controller COMMAND test_weighted_wbc_controller)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wheel_leg_core EXPORT export_wheel_leg_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_dependencies(Eigen3)
  ament_export_targets(export_wheel_leg_core HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_package()
endif()
