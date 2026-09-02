cmake_minimum_required(VERSION 3.5)
project(spinal)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(SPINAL_LINK_TOPIC_CONFIG
  ${CMAKE_CURRENT_SOURCE_DIR}/config/spinal_link_topics.yaml)
set(SPINAL_LINK_TOPIC_HEADER
  ${CMAKE_CURRENT_SOURCE_DIR}/mcu_project/lib/My_Lib/communication/spinal_link_topics.generated.h)
add_custom_target(spinal_link_topic_config_check ALL
  COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_spinal_link_topics.py
    --config ${SPINAL_LINK_TOPIC_CONFIG}
    --output ${SPINAL_LINK_TOPIC_HEADER}
    --check
  DEPENDS
    ${SPINAL_LINK_TOPIC_CONFIG}
    ${SPINAL_LINK_TOPIC_HEADER}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_spinal_link_topics.py
  COMMENT "Checking generated Spinal Link DDS topic configuration")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# ADDED
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(spinal_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(fastrtps REQUIRED)

include(FetchContent)
FetchContent_Declare(
  mavlink_headers
  URL https://github.com/mavlink/c_library_v2/archive/663f9109025433f241ee9fef60e0dbbec64a6cea.tar.gz
)
FetchContent_GetProperties(mavlink_headers)
if(NOT mavlink_headers_POPULATED)
  FetchContent_Populate(mavlink_headers)
endif()

set(SPINAL_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/mcu_project/lib/My_Lib)

include_directories(
  ${SPINAL_DIRS}
)

add_definitions(-DSIMULATION)

add_library(spinal_math
  ${SPINAL_DIRS}/math/AP_Math.cpp
  ${SPINAL_DIRS}/math/location.cpp
  ${SPINAL_DIRS}/math/matrix3.cpp
  ${SPINAL_DIRS}/math/vector2.cpp
  ${SPINAL_DIRS}/math/vector3.cpp
  ${SPINAL_DIRS}/math/quaternion.cpp
)

set_target_properties(spinal_math PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(spinal_flight_control
  ${SPINAL_DIRS}/rc/crsf_protocol.cpp
  ${SPINAL_DIRS}/rc/crsf_teleop.cpp
  ${SPINAL_DIRS}/rc/crsf_input.cpp
  ${SPINAL_DIRS}/rc/simulation/crsf_posix_serial_transport.cpp
  ${SPINAL_DIRS}/rc/simulation/crsf_ros_adapter.cpp
  ${SPINAL_DIRS}/flashmemory/parameter_database.cpp
  ${SPINAL_DIRS}/flashmemory/config_flash_database.cpp
  ${SPINAL_DIRS}/flashmemory/config_flash_storage_sim.cpp
  ${SPINAL_DIRS}/flight_control/flight_control.cpp
  ${SPINAL_DIRS}/flight_control/flight_control_ros_adapter.cpp
  ${SPINAL_DIRS}/flight_control/health/actuator_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/health/compute_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/health/config_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/health/estimation_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/health/flight_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/health/health_manager.cpp
  ${SPINAL_DIRS}/flight_control/health/link_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/health/power_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/health/sensor_health_monitor.cpp
  ${SPINAL_DIRS}/flight_control/supervisor/flight_supervisor.cpp
  ${SPINAL_DIRS}/flight_control/attitude/attitude_control.cpp
  ${SPINAL_DIRS}/flight_control/position/position_control.cpp
  ${SPINAL_DIRS}/flight_control/simulation/flight_control_ros_module.cpp
  ${SPINAL_DIRS}/thruster/simulation/thruster_manager.cpp
  ${SPINAL_DIRS}/thruster/simulation/thruster_ros_module.cpp
)

set_target_properties(spinal_flight_control PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(spinal_flight_control
  PUBLIC
  $<BUILD_INTERFACE:${SPINAL_DIRS}>
  $<INSTALL_INTERFACE:include/My_Lib>
)

target_link_libraries(spinal_flight_control
  PUBLIC
  spinal_math
  spinal_state_estimation
)

ament_target_dependencies(spinal_flight_control
  PUBLIC
  diagnostic_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  spinal_msgs
  std_msgs
  std_srvs
)

add_library(spinal_state_estimation
  ${SPINAL_DIRS}/state_estimate/eskf/eskf15.cpp
)

target_include_directories(spinal_state_estimation
  PUBLIC
  $<BUILD_INTERFACE:${SPINAL_DIRS}>
  $<INSTALL_INTERFACE:include/My_Lib>
)

target_link_libraries(spinal_state_estimation
  PUBLIC
  spinal_math
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

add_executable(spinal_namespace_bridge
  src/spinal_namespace_bridge.cpp
)

# ROS-independent DDS transport shared by the vehicle gateway and the optional
# ROS 2 adapter. This binary layer can run on an aircraft Linux image without a
# ROS graph; ament is only used to install it with the rest of this repository.
add_library(spinal_dds_transport
  src/spinal_link/communication_monitor.cpp
  src/spinal_link/spinal_dds_endpoint.cpp
)
add_dependencies(spinal_dds_transport spinal_link_topic_config_check)
target_compile_features(spinal_dds_transport PUBLIC cxx_std_17)
target_include_directories(spinal_dds_transport
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/spinal_link>
  $<BUILD_INTERFACE:${SPINAL_DIRS}>
  $<INSTALL_INTERFACE:include/spinal_link>
  $<INSTALL_INTERFACE:include/My_Lib>
)
target_link_libraries(spinal_dds_transport PUBLIC fastrtps)

add_executable(gcs_gateway
  src/spinal_link/mavlink_udp_gateway.cpp
  src/spinal_link/gcs_gateway_main.cpp
)
target_compile_features(gcs_gateway PRIVATE cxx_std_17)
target_include_directories(gcs_gateway SYSTEM PRIVATE ${mavlink_headers_SOURCE_DIR})
target_link_libraries(gcs_gateway PRIVATE spinal_dds_transport)

add_executable(spinal_ros2_bridge
  src/spinal_link/spinal_ros2_bridge.cpp
)
target_compile_features(spinal_ros2_bridge PRIVATE cxx_std_17)
target_link_libraries(spinal_ros2_bridge spinal_dds_transport spinal_flight_control)
ament_target_dependencies(spinal_ros2_bridge
  diagnostic_msgs
  rclcpp
  spinal_msgs
  std_msgs
  std_srvs
)

ament_target_dependencies(spinal_namespace_bridge
  rclcpp
)

ament_export_dependencies(diagnostic_msgs fastrtps rclcpp rclcpp_lifecycle spinal_msgs sensor_msgs std_msgs std_srvs)

install(
  TARGETS spinal_state_estimation spinal_math spinal_flight_control spinal_dds_transport
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS spinal_namespace_bridge gcs_gateway spinal_ros2_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${SPINAL_DIRS}/
  DESTINATION include/My_Lib
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY src/spinal_link/
  DESTINATION include/spinal_link
  FILES_MATCHING PATTERN "*.h"
)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/generate_spinal_link_topics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(crsf_protocol_test
    test/crsf_protocol_test.cpp
    ${SPINAL_DIRS}/rc/crsf_protocol.cpp
  )
  target_include_directories(crsf_protocol_test PRIVATE ${SPINAL_DIRS})
  ament_add_gtest(crsf_teleop_test
    test/crsf_teleop_test.cpp
    ${SPINAL_DIRS}/rc/crsf_teleop.cpp
  )
  target_include_directories(crsf_teleop_test PRIVATE ${SPINAL_DIRS})
  ament_add_gtest(crsf_input_test
    test/crsf_input_test.cpp
  )
  target_link_libraries(crsf_input_test spinal_flight_control)
  ament_add_gtest(eskf15_test
    test/eskf15_test.cpp
  )
  target_link_libraries(eskf15_test spinal_state_estimation)
  ament_add_gtest(state_estimate_test
    test/state_estimate_test.cpp
  )
  target_link_libraries(state_estimate_test spinal_flight_control)
  ament_add_gtest(position_control_test
    test/position_control_test.cpp
    ${SPINAL_DIRS}/flight_control/position/position_control.cpp
  )
  target_link_libraries(position_control_test spinal_state_estimation)
  ament_add_gtest(attitude_control_test
    test/attitude_control_test.cpp
  )
  target_link_libraries(attitude_control_test spinal_flight_control)
  ament_add_gtest(flight_supervisor_test
    test/flight_supervisor_test.cpp
  )
  target_link_libraries(flight_supervisor_test spinal_flight_control)
  ament_add_gtest(health_manager_test
    test/health_manager_test.cpp
  )
  target_link_libraries(health_manager_test spinal_flight_control)
  ament_add_gtest(parameter_database_test
    test/parameter_database_test.cpp
  )
  target_link_libraries(parameter_database_test spinal_flight_control)
  ament_add_gtest(config_flash_database_test
    test/config_flash_database_test.cpp
  )
  target_link_libraries(config_flash_database_test spinal_flight_control)
  ament_add_gtest(spinal_link_protocol_test
    test/spinal_link_protocol_test.cpp
  )
  target_include_directories(spinal_link_protocol_test PRIVATE ${SPINAL_DIRS})
  ament_add_gtest(spinal_dds_endpoint_test
    test/spinal_dds_endpoint_test.cpp
  )
  target_link_libraries(spinal_dds_endpoint_test spinal_dds_transport)
  ament_add_pytest_test(crsf_simulator_test
    test/test_crsf_simulator.py
  )
endif()

install(PROGRAMS
  scripts/topic_hz_checker.py
  scripts/flash_firmware.py
  scripts/build_micro_xrce_agent.sh
  scripts/crsf_simulator.py
  scripts/crsf_gazebo_test.py
  DESTINATION lib/${PROJECT_NAME}
  )

install(FILES
  COMMUNICATION_ARCHITECTURE.md
  CRSF_SIMULATION.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include include/My_Lib)

ament_package()
