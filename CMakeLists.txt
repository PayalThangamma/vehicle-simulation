cmake_minimum_required(VERSION 3.16)

project(vehicle_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ============================================================
# Main simulator
# ============================================================

add_executable(main
    src/main.cpp
    src/scenario_loader.cpp
    src/simulator.cpp
    src/vehicle_model.cpp
)


target_include_directories(main
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)


# ============================================================
# Vehicle model unit tests
# ============================================================

add_executable(vehicle_model_tests
    tests/test_vehicle_model.cpp
    src/vehicle_model.cpp
)


target_include_directories(vehicle_model_tests
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)


# ============================================================
# Euler vs RK4 comparison
# ============================================================

add_executable(integration_compare
    tests/integration_compare.cpp
    src/vehicle_model.cpp
)


target_include_directories(integration_compare
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)


# ============================================================
# Timestep sensitivity
# ============================================================

add_executable(timestep_sensitivity
    tests/timestep_sensitivity.cpp
    src/vehicle_model.cpp
)


target_include_directories(timestep_sensitivity
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)


# ============================================================
# Compiler warnings
# ============================================================

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")

    target_compile_options(main PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )

    target_compile_options(vehicle_model_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )

    target_compile_options(integration_compare PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )

    target_compile_options(timestep_sensitivity PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )

endif()


# ============================================================
# CTest
# ============================================================

enable_testing()

add_test(
    NAME vehicle_model_tests
    COMMAND vehicle_model_tests
)