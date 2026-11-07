
cmake_minimum_required(VERSION 3.20)

project(LIO
    VERSION 1.0.0
    DESCRIPTION "LiDAR-Inertial Odometry from Scratch"
    LANGUAGES CXX
)

# ==========================================
# 1. C++ Standard
# ==========================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==========================================
# 2. Project Directories
# ==========================================

set(LIO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(LIO_SOURCE_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/src")

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# ==========================================
# 3. Eigen3 Dependency (Header-only)
# ==========================================

set(EIGEN_INCLUDE_DIR "${THIRD_PARTY_DIR}/eigen")

if(NOT EXISTS "${EIGEN_INCLUDE_DIR}/Eigen/Dense")
    message(FATAL_ERROR "Eigen3 not found: ${EIGEN_INCLUDE_DIR}")
endif()

message(STATUS "Eigen3 found: ${EIGEN_INCLUDE_DIR}")

# ==========================================
# 4. Eigen Interface Target
# ==========================================

add_library(Eigen3::Eigen INTERFACE IMPORTED)

set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIR}"
)

# ==========================================
# 5. LIO Executable
# ==========================================

add_executable(lio
    src/main.cpp
)

target_include_directories(lio PRIVATE
    "${LIO_INCLUDE_DIR}"
)

target_link_libraries(lio PRIVATE
    Eigen3::Eigen
)

# ==========================================
# 6. Testing
# ==========================================

enable_testing()

add_executable(test_types
    tests/test_types.cpp
)

target_include_directories(test_types PRIVATE
    "${LIO_INCLUDE_DIR}"
)

target_link_libraries(test_types PRIVATE
    Eigen3::Eigen
)

add_test(
    NAME TestTypes
    COMMAND test_types
)

# ==========================================
# 7. Compiler Warnings
# ==========================================

if(MSVC)
    target_compile_options(lio PRIVATE /W4)
    target_compile_options(test_types PRIVATE /W4)
else()
    target_compile_options(lio PRIVATE
        -Wall -Wextra -Wpedantic
    )

    target_compile_options(test_types PRIVATE
        -Wall -Wextra -Wpedantic
    )
endif()

# ==========================================
# 8. Build Information
# ==========================================

message(STATUS "================================")
message(STATUS "LIO Project Configuration")
message(STATUS "C++ Standard: C++17")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "================================")
