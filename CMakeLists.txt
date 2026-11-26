cmake_minimum_required(VERSION 3.20)

project(
  rgbd_perception
  VERSION 0.1.0
  DESCRIPTION "ROS-free RGB-D object perception framework"
  LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(RGBD_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)
option(RGBD_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

find_package(Threads REQUIRED)

add_library(rgbd_core
  src/center_patch_depth_estimator.cpp
  src/perception_pipeline.cpp
  src/synthetic_camera.cpp)
add_library(rgbd::core ALIAS rgbd_core)

target_compile_features(rgbd_core PUBLIC cxx_std_17)
set_target_properties(rgbd_core PROPERTIES
  CXX_EXTENSIONS OFF
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  EXPORT_NAME core)

target_include_directories(rgbd_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(rgbd_core PRIVATE Threads::Threads)

configure_file(
  include/rgbd/version.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/rgbd/version.hpp
  @ONLY)

function(rgbd_set_project_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
    if(RGBD_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wsign-conversion
      -Wshadow)
    if(RGBD_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

function(rgbd_enable_sanitizers target)
  if(RGBD_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
      target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    else()
      message(FATAL_ERROR "RGBD_ENABLE_SANITIZERS currently supports GCC and Clang only")
    endif()
  endif()
endfunction()

rgbd_set_project_warnings(rgbd_core)
rgbd_enable_sanitizers(rgbd_core)

add_executable(rgbd_demo apps/rgbd_demo.cpp)
target_link_libraries(rgbd_demo PRIVATE rgbd::core)
rgbd_set_project_warnings(rgbd_demo)
rgbd_enable_sanitizers(rgbd_demo)

if(BUILD_TESTING)
  add_executable(rgbd_tests tests/test_main.cpp)
  target_link_libraries(rgbd_tests PRIVATE rgbd::core)
  rgbd_set_project_warnings(rgbd_tests)
  rgbd_enable_sanitizers(rgbd_tests)
  add_test(NAME rgbd_unit_tests COMMAND rgbd_tests)
endif()

install(
  TARGETS rgbd_core rgbd_demo
  EXPORT rgbd_perceptionTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  DIRECTORY include/rgbd
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp")
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated/rgbd
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp")

configure_package_config_file(
  cmake/rgbd_perceptionConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rgbd_perceptionConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rgbd_perception)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/rgbd_perceptionConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(
  EXPORT rgbd_perceptionTargets
  FILE rgbd_perceptionTargets.cmake
  NAMESPACE rgbd::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rgbd_perception)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/rgbd_perceptionConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/rgbd_perceptionConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rgbd_perception)
