cmake_minimum_required(VERSION 3.16)
project(spinal_stm32h743 LANGUAGES C CXX ASM)

include(FetchContent)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

get_filename_component(REPOSITORY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MCU_PROJECT_DIR "${REPOSITORY_DIR}/spinal/mcu_project")
set(BOARD_DIR "${MCU_PROJECT_DIR}/boards/stm32H7_v2")
set(MY_LIB_DIR "${MCU_PROJECT_DIR}/lib/My_Lib")
set(UART_TRANSPORT_DIR "${REPOSITORY_DIR}/spinal/src/micro_ros_lib")
set(XRCE_ADAPTER_DIR "${REPOSITORY_DIR}/spinal/src/xrce_dds")
set(SPINAL_LINK_TOPIC_CONFIG "${REPOSITORY_DIR}/spinal/config/spinal_link_topics.yaml")
set(SPINAL_LINK_TOPIC_HEADER "${MY_LIB_DIR}/communication/spinal_link_topics.generated.h")
set(FREERTOS_DIR "${BOARD_DIR}/Middlewares/Third_Party/FreeRTOS")
set(LWIP_DIR "${BOARD_DIR}/Middlewares/Third_Party/LwIP")
set(LINKER_SCRIPT "${BOARD_DIR}/STM32CubeIDE/STM32H743VITX_FLASH.ld")
FetchContent_Declare(
  microcdr_source
  URL https://github.com/eProsima/Micro-CDR/archive/refs/tags/v2.0.1.tar.gz)
FetchContent_Declare(
  microxrcedds_client_source
  URL https://github.com/eProsima/Micro-XRCE-DDS-Client/archive/refs/tags/v2.4.3.tar.gz)
FetchContent_GetProperties(microcdr_source)
if(NOT microcdr_source_POPULATED)
  FetchContent_Populate(microcdr_source)
endif()
FetchContent_GetProperties(microxrcedds_client_source)
if(NOT microxrcedds_client_source_POPULATED)
  FetchContent_Populate(microxrcedds_client_source)
endif()

set(XRCE_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/xrce_generated/include")
file(MAKE_DIRECTORY "${XRCE_GENERATED_INCLUDE_DIR}/ucdr" "${XRCE_GENERATED_INCLUDE_DIR}/uxr/client")

set(PROJECT_VERSION_MAJOR 2)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 1)
set(PROJECT_VERSION 2.0.1)
set(CONFIG_MACHINE_ENDIANNESS UCDR_LITTLE_ENDIANNESS)
configure_file(
  "${microcdr_source_SOURCE_DIR}/include/ucdr/config.h.in"
  "${XRCE_GENERATED_INCLUDE_DIR}/ucdr/config.h")

set(PROJECT_VERSION_MAJOR 2)
set(PROJECT_VERSION_MINOR 4)
set(PROJECT_VERSION_PATCH 3)
set(PROJECT_VERSION 2.4.3)
set(UCLIENT_PROFILE_CUSTOM_TRANSPORT ON)
set(UCLIENT_PROFILE_STREAM_FRAMING ON)
set(UCLIENT_MAX_OUTPUT_BEST_EFFORT_STREAMS 1)
set(UCLIENT_MAX_OUTPUT_RELIABLE_STREAMS 1)
set(UCLIENT_MAX_INPUT_BEST_EFFORT_STREAMS 1)
set(UCLIENT_MAX_INPUT_RELIABLE_STREAMS 1)
set(UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS 3)
set(UCLIENT_MIN_SESSION_CONNECTION_INTERVAL 100)
set(UCLIENT_MIN_HEARTBEAT_TIME_INTERVAL 100)
set(UCLIENT_CUSTOM_TRANSPORT_MTU 512)
set(UCLIENT_TWEAK_XRCE_WRITE_LIMIT ON)
configure_file(
  "${microxrcedds_client_source_SOURCE_DIR}/include/uxr/client/config.h.in"
  "${XRCE_GENERATED_INCLUDE_DIR}/uxr/client/config.h")

file(GLOB_RECURSE MY_LIB_SOURCES CONFIGURE_DEPENDS "${MY_LIB_DIR}/*.cpp")
list(FILTER MY_LIB_SOURCES EXCLUDE REGEX ".*/simulation/.*")
list(FILTER MY_LIB_SOURCES EXCLUDE REGEX ".*_sim\\.cpp$")
list(FILTER MY_LIB_SOURCES EXCLUDE REGEX ".*_ros_(module|adapter)\\.cpp$")
set(XRCE_ADAPTER_SOURCES
  "${XRCE_ADAPTER_DIR}/xrce_dds_client.cpp"
  "${XRCE_ADAPTER_DIR}/xrce_uart_transport.cpp"
  "${UART_TRANSPORT_DIR}/STM32Hardware.cpp")
set(MICROCDR_SOURCES
  "${microcdr_source_SOURCE_DIR}/src/c/common.c"
  "${microcdr_source_SOURCE_DIR}/src/c/types/basic.c"
  "${microcdr_source_SOURCE_DIR}/src/c/types/string.c"
  "${microcdr_source_SOURCE_DIR}/src/c/types/array.c"
  "${microcdr_source_SOURCE_DIR}/src/c/types/sequence.c")
set(MICRO_XRCE_CLIENT_SOURCES
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/stream/input_best_effort_stream.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/stream/input_reliable_stream.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/stream/output_best_effort_stream.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/stream/output_reliable_stream.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/stream/stream_storage.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/stream/stream_id.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/stream/seq_num.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/session.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/session_info.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/submessage.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/object_id.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/serialization/xrce_types.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/serialization/xrce_header.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/serialization/xrce_subheader.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/util/ping.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/common_create_entities.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/create_entities_ref.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/create_entities_xml.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/create_entities_bin.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/read_access.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/core/session/write_access.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/profile/transport/custom/custom_transport.c"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c/profile/transport/stream_framing/stream_framing_protocol.c"
  "${XRCE_ADAPTER_DIR}/xrce_time.cpp")
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
  "${BOARD_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c")
file(GLOB_RECURSE FREERTOS_SOURCES CONFIGURE_DEPENDS
  "${FREERTOS_DIR}/Source/*.c")
file(GLOB_RECURSE LWIP_SOURCES CONFIGURE_DEPENDS
  "${LWIP_DIR}/src/*.c"
  "${LWIP_DIR}/system/OS/*.c")
file(GLOB BOARD_C_SOURCES CONFIGURE_DEPENDS "${BOARD_DIR}/Src/*.c")

# The Cube project keeps main.cpp as a symlink to main.c so its USER CODE blocks
# can contain C++. Compile main.c once, explicitly as C++.
set(MAIN_SOURCE "${BOARD_DIR}/Src/main.c")
list(REMOVE_ITEM BOARD_C_SOURCES "${MAIN_SOURCE}")
set_source_files_properties("${MAIN_SOURCE}" PROPERTIES LANGUAGE CXX)

set(FIRMWARE_SOURCES
  ${MY_LIB_SOURCES}
  ${XRCE_ADAPTER_SOURCES}
  ${MICROCDR_SOURCES}
  ${MICRO_XRCE_CLIENT_SOURCES}
  ${HAL_SOURCES}
  ${FREERTOS_SOURCES}
  ${LWIP_SOURCES}
  ${BOARD_C_SOURCES}
  "${MAIN_SOURCE}"
  "${BOARD_DIR}/Drivers/BSP/Components/lan8742/lan8742.c"
  "${BOARD_DIR}/STM32CubeIDE/Application/User/syscalls.c"
  "${BOARD_DIR}/STM32CubeIDE/Application/User/sysmem.c"
  "${BOARD_DIR}/STM32CubeIDE/Application/Startup/startup_stm32h743vitx.s")

add_executable(spinal ${FIRMWARE_SOURCES})
add_custom_target(spinal_link_topic_config_check
  COMMAND ${Python3_EXECUTABLE}
    ${REPOSITORY_DIR}/spinal/scripts/generate_spinal_link_topics.py
    --config ${SPINAL_LINK_TOPIC_CONFIG}
    --output ${SPINAL_LINK_TOPIC_HEADER}
    --check
  DEPENDS
    ${SPINAL_LINK_TOPIC_CONFIG}
    ${SPINAL_LINK_TOPIC_HEADER}
    ${REPOSITORY_DIR}/spinal/scripts/generate_spinal_link_topics.py)
add_dependencies(spinal spinal_link_topic_config_check)
set_target_properties(spinal PROPERTIES OUTPUT_NAME spinal SUFFIX ".elf")

target_compile_definitions(spinal PRIVATE
  USE_HAL_DRIVER
  STM32H743xx
  STMCUBEMX_NEW_VERSION
  $<$<CONFIG:Debug>:DEBUG>)

target_include_directories(spinal PRIVATE
  "${BOARD_DIR}/Drivers/CMSIS/Include"
  "${BOARD_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
  "${BOARD_DIR}/Inc"
  "${BOARD_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
  "${BOARD_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc"
  "${FREERTOS_DIR}/Source/include"
  "${FREERTOS_DIR}/Source/CMSIS_RTOS"
  "${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F"
  "${MY_LIB_DIR}/configs/STM32H7_v2"
  "${MY_LIB_DIR}"
  "${UART_TRANSPORT_DIR}"
  "${XRCE_ADAPTER_DIR}/.."
  "${XRCE_GENERATED_INCLUDE_DIR}"
  "${microcdr_source_SOURCE_DIR}/include"
  "${microxrcedds_client_source_SOURCE_DIR}/include"
  "${microxrcedds_client_source_SOURCE_DIR}/src/c"
  "${LWIP_DIR}/src/include"
  "${LWIP_DIR}/system"
  "${LWIP_DIR}/src/include/netif/ppp"
  "${LWIP_DIR}/src/include/lwip"
  "${LWIP_DIR}/src/include/lwip/apps"
  "${LWIP_DIR}/src/include/lwip/priv"
  "${LWIP_DIR}/src/include/lwip/prot"
  "${LWIP_DIR}/src/include/netif"
  "${LWIP_DIR}/src/include/compat/posix"
  "${LWIP_DIR}/src/include/compat/posix/arpa"
  "${LWIP_DIR}/src/include/compat/posix/net"
  "${LWIP_DIR}/src/include/compat/posix/sys"
  "${LWIP_DIR}/src/include/compat/stdc"
  "${LWIP_DIR}/system/arch"
  "${BOARD_DIR}/Drivers/BSP/Components/lan8742")

target_compile_options(spinal PRIVATE
  -mcpu=cortex-m7
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb
  -ffunction-sections
  -fdata-sections
  -Wall
  -fstack-usage
  --specs=nano.specs
  $<$<CONFIG:Debug>:-O0>
  $<$<CONFIG:Debug>:-g3>
  $<$<NOT:$<CONFIG:Debug>>:-Os>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>)

target_link_options(spinal PRIVATE
  -mcpu=cortex-m7
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb
  -T${LINKER_SCRIPT}
  --specs=nosys.specs
  --specs=nano.specs
  -static
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/spinal.map)

target_link_libraries(spinal PRIVATE
  -Wl,--start-group
  c
  m
  stdc++
  supc++
  -Wl,--end-group)

add_custom_command(TARGET spinal POST_BUILD
  COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:spinal>" "${CMAKE_CURRENT_BINARY_DIR}/spinal.bin"
  COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:spinal>" "${CMAKE_CURRENT_BINARY_DIR}/spinal.hex"
  COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:spinal>"
  COMMENT "Generating spinal.bin and spinal.hex")
