
cmake_minimum_required(VERSION 3.20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(LIO
    VERSION 1.0.0
    DESCRIPTION "LiDAR-Inertial Odometry from Scratch"
    LANGUAGES CXX
)

# ==================================================
# 1. Project Configuration
# ==================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LIO_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(LIO_SOURCE_DIR  "${PROJECT_SOURCE_DIR}/src")
set(THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/third_party")

# ==================================================
# 2. Dependencies
# ==================================================

set(EIGEN_INCLUDE_DIR "${THIRD_PARTY_DIR}/eigen")
set(NANOFLANN_INCLUDE_DIR "${THIRD_PARTY_DIR}/nanoflann/include")
set(IKDTREE_INCLUDE_DIR "${THIRD_PARTY_DIR}/ikd-Tree/ikd-Tree")

if(NOT EXISTS "${EIGEN_INCLUDE_DIR}/Eigen/Dense")
    message(FATAL_ERROR "Eigen3 not found in ${EIGEN_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${NANOFLANN_INCLUDE_DIR}/nanoflann.hpp")
    message(FATAL_ERROR "Nanoflann not found in ${NANOFLANN_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${IKDTREE_INCLUDE_DIR}/ikd_Tree.h")
    message(FATAL_ERROR "IKD-Tree not found in ${IKDTREE_INCLUDE_DIR}")
endif()

# ==================================================
# 2.1 Eigen
# ==================================================

add_library(Eigen3::Eigen INTERFACE IMPORTED)

target_include_directories(Eigen3::Eigen INTERFACE
    "${EIGEN_INCLUDE_DIR}"
)

# ==================================================
# 2.2 Nanoflann
# ==================================================

add_library(NanoFLANN::NanoFLANN INTERFACE IMPORTED)

target_include_directories(NanoFLANN::NanoFLANN INTERFACE
    "${NANOFLANN_INCLUDE_DIR}"
)

# ==================================================
# 2.3 IKD-Tree
# ==================================================

find_package(Threads REQUIRED)

add_library(ikdtree STATIC
    "${IKDTREE_INCLUDE_DIR}/ikd_Tree.cpp"
)

target_include_directories(ikdtree PUBLIC
    "${IKDTREE_INCLUDE_DIR}"
)

target_compile_features(ikdtree PUBLIC cxx_std_17)

target_link_libraries(ikdtree PUBLIC
    Eigen3::Eigen
    Threads::Threads
)

add_library(IKDTree::IKDTree ALIAS ikdtree)

# ==================================================
# 3. Compiler Settings
# ==================================================

add_library(lio_options INTERFACE)

if(MSVC)
    target_compile_options(lio_options INTERFACE /W4)
else()
    target_compile_options(lio_options INTERFACE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()

# ==================================================
# 4. Main Library
# ==================================================

add_library(lio_core
    src/common/math.cpp
    src/common/se3.cpp
    src/common/config.cpp

    src/io/kitti_reader.cpp
    src/io/timestamp_reader.cpp
    src/io/kitti_sequence.cpp
    src/io/ground_truth_reader.cpp
    src/io/kitti_dataset.cpp

    src/preprocessing/range_filter.cpp
    src/preprocessing/invalid_point_filter.cpp
    src/preprocessing/voxel_filter.cpp
    src/preprocessing/cloud_transform.cpp
    src/preprocessing/preprocessing_pipeline.cpp

    src/registration/nearest_neighbor.cpp
    src/registration/correspondence.cpp
    src/registration/point_to_point_icp.cpp
    src/registration/point_to_plane_icp.cpp
    src/registration/correspondence_filter.cpp
    src/registration/icp_convergence.cpp
    src/registration/registration_evaluator.cpp

    src/odometry/relative_pose.cpp
    src/odometry/pose_accumulator.cpp
    src/odometry/trajectory.cpp
    src/odometry/trajectory_io.cpp
    src/odometry/trajectory_evaluator.cpp
    src/odometry/trajectory_evaluator_comparison.cpp

    # Local Mapping
    src/mapping/local_map.cpp
    src/registration/scan_to_map_registration.cpp
    src/mapping/local_mapping.cpp
)

target_include_directories(lio_core PUBLIC
    "${LIO_INCLUDE_DIR}"
)

target_link_libraries(lio_core PUBLIC
    Eigen3::Eigen
    NanoFLANN::NanoFLANN
    IKDTree::IKDTree
    lio_options
)

# ==================================================
# 5. Main Executable
# ==================================================

add_executable(lio
    src/main.cpp
)
target_link_libraries(lio PRIVATE lio_core)

add_executable(test_ikdtree
    tests/test_ikdtree.cpp
)

target_link_libraries(test_ikdtree PRIVATE
    IKDTree::IKDTree
)

add_test(
    NAME test_ikdtree
    COMMAND test_ikdtree
)


# ==================================================
# 6. Testing
# ==================================================

enable_testing()

function(lio_add_test test_name source_file)

    add_executable(${test_name} ${source_file})

    target_link_libraries(${test_name} PRIVATE lio_core)

    add_test(
        NAME ${test_name}
        COMMAND ${test_name}
    )

endfunction()

lio_add_test(test_types tests/test_types.cpp)
lio_add_test(test_se3 tests/test_se3.cpp)
lio_add_test(test_math tests/test_math.cpp)
lio_add_test(test_state tests/test_state.cpp)
lio_add_test(test_config tests/test_config.cpp)

lio_add_test(test_kittireader tests/test_kitti_reader.cpp)
lio_add_test(test_timestamp_reader tests/test_timestamp_reader.cpp)
lio_add_test(test_kitti_sequence tests/test_kitti_sequence.cpp)
lio_add_test(test_ground_truth_reader tests/test_ground_truth_reader.cpp)
lio_add_test(test_dataset_interface tests/test_dataset_interface.cpp)

lio_add_test(test_range_filter tests/test_range_filter.cpp)
lio_add_test(test_invalid_point_filter tests/test_invalid_point_filter.cpp)
lio_add_test(test_voxel_filter tests/test_voxel_filter.cpp)
lio_add_test(test_preprocessing_pipeline tests/test_preprocessing_pipeline.cpp)
lio_add_test(test_preprocessing_integration tests/test_preprocessing_integration.cpp)

lio_add_test(test_nearest_neighbor tests/test_nearest_neighbor.cpp)
lio_add_test(test_correspondence tests/test_correspondence.cpp)
lio_add_test(test_point_to_point_icp tests/test_point_to_point_icp.cpp)
lio_add_test(test_point_to_plane_icp tests/test_point_to_plane_icp.cpp)
lio_add_test(test_correspondence_filter tests/test_correspondence_filter.cpp)
lio_add_test(test_icp_convergence tests/test_icp_convergence.cpp)
lio_add_test(test_registration_evaluator tests/test_registration_evaluator.cpp)
lio_add_test(test_kitti_registration tests/test_kitti_registration.cpp)

lio_add_test(test_relative_pose tests/test_relative_pose.cpp)
lio_add_test(test_pose_accumulator tests/test_pose_accumulator.cpp)
lio_add_test(test_trajectory tests/test_trajectory.cpp)
lio_add_test(test_trajectory_io tests/test_trajectory_io.cpp)
lio_add_test(test_trajectory_evaluator tests/test_trajectory_evaluator.cpp)
lio_add_test(test_trajectory_comparison tests/test_trajectory_evaluator_comparison.cpp)

lio_add_test(test_local_map tests/test_local_map.cpp)
lio_add_test(test_scan_to_map_registration tests/test_scan_to_map_registration.cpp)
lio_add_test(test_local_mapping tests/test_local_mapping.cpp)

# ==================================================
# 7. Build Information
# ==================================================

message(STATUS "================================")
message(STATUS "LIO Configuration")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "================================")
