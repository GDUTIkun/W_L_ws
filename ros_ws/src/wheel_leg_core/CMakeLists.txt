cmake_minimum_required(VERSION 3.16)
project(wheel_leg_core LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(proxsuite 0.7.3 EXACT CONFIG REQUIRED)

set(ACADOS_ROOT "/home/t/opt/acados" CACHE PATH
  "Frozen acados installation used by Phase 23")
find_path(ACADOS_INCLUDE_DIR acados_c/ocp_nlp_interface.h
  PATHS "${ACADOS_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(ACADOS_LIBRARY acados
  PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(HPIPM_LIBRARY hpipm
  PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(BLASFEO_LIBRARY NAMES blasfeo libblasfeo.so.0
  PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

set(PHASE23_GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/phase23_nominal_nmpc_v1")
add_library(phase23_acados_generated OBJECT
  ${PHASE23_GENERATED_DIR}/acados_solver_ocp_phase23_nominal_nmpc_4f8fd2e4.c
  ${PHASE23_GENERATED_DIR}/phase23_nominal_nmpc_model/phase23_nominal_nmpc_dyn_disc_phi_fun.c
  ${PHASE23_GENERATED_DIR}/phase23_nominal_nmpc_model/phase23_nominal_nmpc_dyn_disc_phi_fun_jac.c)
target_include_directories(phase23_acados_generated PRIVATE
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
  ${PHASE23_GENERATED_DIR})

add_library(wheel_leg_core
  src/controller_core.cpp
  src/dense_qp_solver.cpp
  src/nominal_nmpc_model.cpp
  src/nominal_nmpc_solver.cpp
  src/nominal_wbc_model.cpp
  src/weighted_wbc_problem.cpp
  src/weighted_wbc_controller.cpp
  $<TARGET_OBJECTS:phase23_acados_generated>)
add_library(wheel_leg_core::wheel_leg_core ALIAS wheel_leg_core)
target_include_directories(wheel_leg_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(wheel_leg_core PRIVATE
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
  ${PHASE23_GENERATED_DIR})
target_link_libraries(wheel_leg_core PUBLIC Eigen3::Eigen PRIVATE
  proxsuite::proxsuite
  ${ACADOS_LIBRARY}
  ${HPIPM_LIBRARY}
  ${BLASFEO_LIBRARY}
  m)
set_target_properties(wheel_leg_core PROPERTIES
  BUILD_RPATH "${ACADOS_ROOT}/lib"
  INSTALL_RPATH "${ACADOS_ROOT}/lib")
target_link_options(wheel_leg_core INTERFACE
  "LINKER:--disable-new-dtags,-rpath,${ACADOS_ROOT}/lib")
if(MSVC)
  target_compile_options(wheel_leg_core PRIVATE /W4 /WX)
else()
  target_compile_options(wheel_leg_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(profile_proxqp_solver
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/experiments/profile_proxqp_solver.cpp)
  target_link_libraries(profile_proxqp_solver PRIVATE
    Eigen3::Eigen proxsuite::proxsuite)
  if(MSVC)
    target_compile_options(profile_proxqp_solver PRIVATE /W4 /WX)
  else()
    target_compile_options(profile_proxqp_solver PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
  endif()

  add_executable(benchmark_dense_qp_solver
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/experiments/benchmark_dense_qp_solver.cpp)
  target_link_libraries(benchmark_dense_qp_solver PRIVATE wheel_leg_core)
  if(MSVC)
    target_compile_options(benchmark_dense_qp_solver PRIVATE /W4 /WX)
  else()
    target_compile_options(benchmark_dense_qp_solver PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
  endif()

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

  add_executable(test_nominal_nmpc_model test/test_nominal_nmpc_model.cpp)
  target_link_libraries(test_nominal_nmpc_model PRIVATE wheel_leg_core)
  target_compile_definitions(test_nominal_nmpc_model PRIVATE
    NMPC_MODEL_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/phase23_nominal_nmpc_model_golden_v2.txt")
  if(MSVC)
    target_compile_options(test_nominal_nmpc_model PRIVATE /W4 /WX)
  else()
    target_compile_options(test_nominal_nmpc_model PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_nominal_nmpc_model COMMAND test_nominal_nmpc_model)

  add_executable(test_nominal_nmpc_solver test/test_nominal_nmpc_solver.cpp)
  target_link_libraries(test_nominal_nmpc_solver PRIVATE wheel_leg_core)
  if(MSVC)
    target_compile_options(test_nominal_nmpc_solver PRIVATE /W4 /WX)
  else()
    target_compile_options(test_nominal_nmpc_solver PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_nominal_nmpc_solver
    COMMAND test_nominal_nmpc_solver)

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

  add_executable(test_controller_core_weighted_wbc
    test/test_controller_core_weighted_wbc.cpp)
  target_link_libraries(test_controller_core_weighted_wbc PRIVATE wheel_leg_core)
  target_compile_definitions(test_controller_core_weighted_wbc PRIVATE
    WBC_PROBLEM_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/phase21_weighted_wbc_problem_golden_v2.txt")
  if(MSVC)
    target_compile_options(test_controller_core_weighted_wbc PRIVATE /W4 /WX)
  else()
    target_compile_options(test_controller_core_weighted_wbc PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME wheel_leg_core_controller_core_weighted_wbc
    COMMAND test_controller_core_weighted_wbc)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wheel_leg_core EXPORT export_wheel_leg_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_dependencies(Eigen3 proxsuite)
  ament_export_targets(export_wheel_leg_core HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_package()
endif()
