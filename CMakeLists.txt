cmake_minimum_required(VERSION 3.20)

project(
  rgbd_perception
  VERSION 0.2.0
  DESCRIPTION "ROS-free RGB-D object perception framework"
  LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(RGBD_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)
option(RGBD_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(RGBD_WITH_ORBBEC "Build the Orbbec Gemini 305 camera backend" OFF)
option(RGBD_WITH_REALSENSE "Build the Intel RealSense D435i camera backend" OFF)
option(RGBD_WITH_MCAP "Build the MCAP writer and Lichtblick app" ON)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

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
      -Wshadow)
    if(RGBD_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

function(rgbd_enable_sanitizers target)
  if(RGBD_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${target} PRIVATE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer)
      target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    else()
      message(FATAL_ERROR "RGBD_ENABLE_SANITIZERS supports GCC and Clang")
    endif()
  endif()
endfunction()

function(rgbd_configure_target target export_name)
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES
    CXX_EXTENSIONS OFF
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    EXPORT_NAME ${export_name})
  rgbd_set_project_warnings(${target})
  rgbd_enable_sanitizers(${target})
endfunction()

add_library(rgbd_core
  src/core/config.cpp
  src/core/depth_estimator.cpp
  src/core/logging.cpp
  src/core/perception_pipeline.cpp)
add_library(rgbd::core ALIAS rgbd_core)
rgbd_configure_target(rgbd_core core)
target_include_directories(rgbd_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rgbd_core PRIVATE spdlog::spdlog yaml-cpp::yaml-cpp)

add_library(rgbd_inference src/inference/tensor.cpp)
add_library(rgbd::inference ALIAS rgbd_inference)
rgbd_configure_target(rgbd_inference inference)
target_include_directories(rgbd_inference
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rgbd_inference PUBLIC rgbd::core)

add_library(rgbd_recognition src/recognition/center_box_recognizer.cpp)
add_library(rgbd::recognition ALIAS rgbd_recognition)
rgbd_configure_target(rgbd_recognition recognition)
target_include_directories(rgbd_recognition
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rgbd_recognition PUBLIC rgbd::core rgbd::inference)

add_library(rgbd_io
  src/io/recording.cpp
  src/io/replay_camera.cpp)
add_library(rgbd::io ALIAS rgbd_io)
rgbd_configure_target(rgbd_io io)
target_include_directories(rgbd_io
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rgbd_io PUBLIC rgbd::core)

set(RGBD_MCAP_ENABLED OFF)
set(RGBD_APPS rgbd_viewer rgbd_replay)

if(RGBD_WITH_MCAP)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mcap/cpp/mcap/include/mcap/writer.hpp")
    message(WARNING "third_party/mcap is not initialized; RGBD_WITH_MCAP is disabled")
  else()
    find_package(Protobuf REQUIRED)
    find_package(nlohmann_json REQUIRED)
    find_package(libwebsockets CONFIG REQUIRED)
    if(TARGET websockets_shared)
      set(RGBD_WEBSOCKETS_TARGET websockets_shared)
    elseif(TARGET websockets)
      set(RGBD_WEBSOCKETS_TARGET websockets)
    else()
      message(FATAL_ERROR "libwebsockets target not found")
    endif()
    set(RGBD_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
    set(RGBD_PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/proto")
    set(RGBD_PROTO_FILES
      "${RGBD_PROTO_DIR}/foxglove/RawImage.proto"
      "${RGBD_PROTO_DIR}/foxglove/CameraCalibration.proto")
    set(RGBD_PROTO_GENERATED
      "${RGBD_PROTO_GEN_DIR}/foxglove/RawImage.pb.cc"
      "${RGBD_PROTO_GEN_DIR}/foxglove/RawImage.pb.h"
      "${RGBD_PROTO_GEN_DIR}/foxglove/CameraCalibration.pb.cc"
      "${RGBD_PROTO_GEN_DIR}/foxglove/CameraCalibration.pb.h")
    set(RGBD_PROTO_PATH_ARGS "--proto_path=${RGBD_PROTO_DIR}")
    foreach(include_dir ${Protobuf_INCLUDE_DIRS})
      list(APPEND RGBD_PROTO_PATH_ARGS "--proto_path=${include_dir}")
    endforeach()
    add_custom_command(
      OUTPUT ${RGBD_PROTO_GENERATED}
      COMMAND ${CMAKE_COMMAND} -E make_directory "${RGBD_PROTO_GEN_DIR}"
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ${RGBD_PROTO_PATH_ARGS}
        "--cpp_out=${RGBD_PROTO_GEN_DIR}"
        ${RGBD_PROTO_FILES}
      DEPENDS ${RGBD_PROTO_FILES}
      VERBATIM)
    add_library(rgbd_mcap_proto OBJECT ${RGBD_PROTO_GENERATED})
    target_include_directories(rgbd_mcap_proto PRIVATE
      "${RGBD_PROTO_GEN_DIR}"
      ${Protobuf_INCLUDE_DIRS})
    target_link_libraries(rgbd_mcap_proto PRIVATE protobuf::libprotobuf)

    add_library(rgbd_mcap
      src/io/foxglove_messages.cpp
      src/io/local_web_server.cpp
      src/io/lichtblick_server.cpp
      src/io/mcap_implementation.cpp
      src/io/mcap_image_writer.cpp
      $<TARGET_OBJECTS:rgbd_mcap_proto>)
    add_library(rgbd::mcap ALIAS rgbd_mcap)
    rgbd_configure_target(rgbd_mcap mcap)
    target_include_directories(rgbd_mcap
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      PRIVATE
        "${RGBD_PROTO_GEN_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mcap/cpp/mcap/include")
    target_compile_definitions(rgbd_mcap PRIVATE
      MCAP_COMPRESSION_NO_ZSTD
      MCAP_COMPRESSION_NO_LZ4)
    target_link_libraries(rgbd_mcap
      PUBLIC
        rgbd::io
        protobuf::libprotobuf
      PRIVATE
        nlohmann_json::nlohmann_json
        ${RGBD_WEBSOCKETS_TARGET})
    list(APPEND RGBD_APPS rgbd_lichtblick)
    set(RGBD_MCAP_ENABLED ON)
  endif()
endif()

add_library(rgbd_cameras
  src/cameras/factory.cpp
  src/cameras/synthetic_camera.cpp)
add_library(rgbd::cameras ALIAS rgbd_cameras)
rgbd_configure_target(rgbd_cameras cameras)
target_include_directories(rgbd_cameras
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rgbd_cameras PUBLIC rgbd::core rgbd::io)

set(RGBD_EXPORT_TARGETS
  rgbd_core
  rgbd_inference
  rgbd_recognition
  rgbd_io
  rgbd_cameras)

if(RGBD_MCAP_ENABLED)
  list(APPEND RGBD_EXPORT_TARGETS rgbd_mcap)
endif()

if(RGBD_WITH_ORBBEC)
  if(NOT DEFINED OrbbecSDK_DIR)
    set(OrbbecSDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/_sdk/install/lib")
  endif()
  find_package(OrbbecSDK REQUIRED)
  add_library(rgbd_orbbec src/cameras/orbbec/orbbec_camera.cpp)
  add_library(rgbd::orbbec ALIAS rgbd_orbbec)
  rgbd_configure_target(rgbd_orbbec orbbec)
  target_include_directories(rgbd_orbbec
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(rgbd_orbbec
    PUBLIC rgbd::core
    PRIVATE ob::OrbbecSDK)
  set_target_properties(rgbd_orbbec PROPERTIES
    BUILD_RPATH "${OrbbecSDK_DIR}/..")
  target_compile_definitions(rgbd_cameras PRIVATE RGBD_WITH_ORBBEC=1)
  target_link_libraries(rgbd_cameras PRIVATE rgbd::orbbec)
  list(APPEND RGBD_EXPORT_TARGETS rgbd_orbbec)
endif()

if(RGBD_WITH_REALSENSE)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/librealsense/CMakeLists.txt")
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(third_party/librealsense EXCLUDE_FROM_ALL)
  endif()
  find_package(realsense2 REQUIRED)
  if(TARGET realsense2::realsense2)
    set(RGBD_REALSENSE_TARGET realsense2::realsense2)
  else()
    set(RGBD_REALSENSE_TARGET realsense2)
  endif()
  add_library(rgbd_realsense src/cameras/realsense/realsense_camera.cpp)
  add_library(rgbd::realsense ALIAS rgbd_realsense)
  rgbd_configure_target(rgbd_realsense realsense)
  target_include_directories(rgbd_realsense
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(rgbd_realsense
    PUBLIC rgbd::core
    PRIVATE ${RGBD_REALSENSE_TARGET})
  target_compile_definitions(rgbd_cameras PRIVATE RGBD_WITH_REALSENSE=1)
  target_link_libraries(rgbd_cameras PRIVATE rgbd::realsense)
  list(APPEND RGBD_EXPORT_TARGETS rgbd_realsense)
endif()

configure_file(
  include/rgbd/version.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/rgbd/version.hpp
  @ONLY)

add_executable(rgbd_viewer apps/rgbd_viewer.cpp)
target_link_libraries(rgbd_viewer PRIVATE rgbd::cameras rgbd::recognition)
rgbd_set_project_warnings(rgbd_viewer)
rgbd_enable_sanitizers(rgbd_viewer)

add_executable(rgbd_replay apps/rgbd_replay.cpp)
target_link_libraries(rgbd_replay PRIVATE rgbd::cameras rgbd::recognition)
rgbd_set_project_warnings(rgbd_replay)
rgbd_enable_sanitizers(rgbd_replay)

if(RGBD_MCAP_ENABLED)
  add_executable(rgbd_lichtblick apps/rgbd_lichtblick.cpp)
  target_link_libraries(rgbd_lichtblick PRIVATE rgbd::cameras rgbd::mcap)
  rgbd_set_project_warnings(rgbd_lichtblick)
  rgbd_enable_sanitizers(rgbd_lichtblick)
endif()

if(BUILD_TESTING)
  add_executable(rgbd_tests
    tests/test_main.cpp
    tests/test_camera_factory.cpp
    tests/test_config.cpp
    tests/test_depth_estimator.cpp
    tests/test_geometry.cpp
    tests/test_pipeline.cpp
    tests/test_recording.cpp
    tests/test_synthetic_camera.cpp
    tests/test_tensor.cpp)
  target_include_directories(rgbd_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_link_libraries(rgbd_tests PRIVATE
    rgbd::cameras
    rgbd::recognition
    rgbd::inference
    rgbd::io)
  rgbd_set_project_warnings(rgbd_tests)
  rgbd_enable_sanitizers(rgbd_tests)
  if(RGBD_MCAP_ENABLED)
    target_sources(rgbd_tests PRIVATE tests/test_mcap_writer.cpp)
    target_link_libraries(rgbd_tests PRIVATE rgbd::mcap)
  endif()
  add_test(NAME rgbd_unit_tests COMMAND rgbd_tests)
endif()

find_program(RGBD_CPPLINT_EXECUTABLE NAMES cpplint)
if(RGBD_CPPLINT_EXECUTABLE)
  add_custom_target(cpplint
    COMMAND ${RGBD_CPPLINT_EXECUTABLE} --recursive apps include src tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

install(
  TARGETS ${RGBD_EXPORT_TARGETS}
  EXPORT rgbd_perceptionTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  TARGETS ${RGBD_APPS}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
  FILES config/rgbd.yaml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/rgbd_perception)

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
