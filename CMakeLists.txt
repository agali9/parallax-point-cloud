cmake_minimum_required(VERSION 3.20)

project(pointcloud_pipeline
    VERSION 0.1.0
    DESCRIPTION "Real-time LiDAR point cloud processing pipeline"
    LANGUAGES CXX)

option(POINTCLOUD_PIPELINE_BUILD_PYTHON "Build pybind11 Python extension when pybind11 is available" ON)
option(POINTCLOUD_PIPELINE_BUILD_TESTS "Build C++ unit tests" ON)
option(POINTCLOUD_PIPELINE_BUILD_BENCHMARKS "Build benchmark executable" ON)
option(POINTCLOUD_PIPELINE_USE_OPEN3D "Build optional Open3D conversion helpers when Open3D is available" ON)
option(POINTCLOUD_PIPELINE_ENABLE_COVERAGE "Enable gcov/llvm-cov coverage flags for tests" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(pointcloud_pipeline SHARED
    src/filtering.cpp
    src/pipeline.cpp
    src/segmentation.cpp
    src/voxel_grid.cpp)
set_target_properties(pointcloud_pipeline PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(pointcloud_pipeline::pointcloud_pipeline ALIAS pointcloud_pipeline)

target_include_directories(pointcloud_pipeline
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(pointcloud_pipeline PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(pointcloud_pipeline PRIVATE /W4 /permissive-)
else()
    target_compile_options(pointcloud_pipeline PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(POINTCLOUD_PIPELINE_ENABLE_COVERAGE AND NOT MSVC)
    target_compile_options(pointcloud_pipeline PRIVATE --coverage -O0 -g)
    target_link_options(pointcloud_pipeline PRIVATE --coverage)
endif()

if(POINTCLOUD_PIPELINE_USE_OPEN3D)
    find_package(Open3D QUIET)
    if(Open3D_FOUND)
        target_sources(pointcloud_pipeline PRIVATE src/open3d_bridge.cpp)
        target_compile_definitions(pointcloud_pipeline PUBLIC POINTCLOUD_PIPELINE_WITH_OPEN3D)
        target_link_libraries(pointcloud_pipeline PUBLIC Open3D::Open3D)
        message(STATUS "Open3D found: optional Open3D bridge enabled")
    else()
        message(STATUS "Open3D not found: core library still builds without Open3D bridge")
    endif()
endif()

if(POINTCLOUD_PIPELINE_BUILD_PYTHON)
    find_package(pybind11 CONFIG QUIET)
    if(pybind11_FOUND)
        pybind11_add_module(pointcloud_pipeline_py python/bindings.cpp)
        target_link_libraries(pointcloud_pipeline_py PRIVATE pointcloud_pipeline)
        if(MSVC)
            target_compile_options(pointcloud_pipeline_py PRIVATE /W4 /permissive-)
        else()
            target_compile_options(pointcloud_pipeline_py PRIVATE -Wall -Wextra -Wpedantic)
        endif()
    else()
        message(STATUS "pybind11 not found: Python extension skipped")
    endif()
endif()

if(POINTCLOUD_PIPELINE_BUILD_TESTS)
    enable_testing()
    find_package(GTest QUIET)

    add_executable(pointcloud_pipeline_tests
        tests/test_filtering.cpp
        tests/test_pipeline.cpp
        tests/test_segmentation.cpp
        tests/test_voxel_grid.cpp)
    target_link_libraries(pointcloud_pipeline_tests PRIVATE pointcloud_pipeline)
    target_include_directories(pointcloud_pipeline_tests PRIVATE tests)

    if(GTest_FOUND)
        target_link_libraries(pointcloud_pipeline_tests PRIVATE GTest::gtest_main)
        include(GoogleTest)
        gtest_discover_tests(pointcloud_pipeline_tests)
    else()
        target_sources(pointcloud_pipeline_tests PRIVATE tests/minigtest_main.cpp)
        target_compile_definitions(pointcloud_pipeline_tests PRIVATE POINTCLOUD_PIPELINE_USE_MINIGTEST)
        add_test(NAME pointcloud_pipeline_tests COMMAND pointcloud_pipeline_tests)
        message(STATUS "GTest not found: using a tiny local compatibility runner")
    endif()

    if(MSVC)
        target_compile_options(pointcloud_pipeline_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(pointcloud_pipeline_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

if(POINTCLOUD_PIPELINE_BUILD_BENCHMARKS)
    add_executable(pointcloud_pipeline_benchmark benchmarks/pipeline_benchmark.cpp)
    target_link_libraries(pointcloud_pipeline_benchmark PRIVATE pointcloud_pipeline)
    if(MSVC)
        target_compile_options(pointcloud_pipeline_benchmark PRIVATE /W4 /permissive-)
    else()
        target_compile_options(pointcloud_pipeline_benchmark PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS pointcloud_pipeline
    EXPORT pointcloud_pipelineTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT pointcloud_pipelineTargets
    NAMESPACE pointcloud_pipeline::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pointcloud_pipeline)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_pipelineConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

configure_package_config_file(
    cmake/pointcloud_pipelineConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_pipelineConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pointcloud_pipeline)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_pipelineConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/pointcloud_pipelineConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pointcloud_pipeline)
