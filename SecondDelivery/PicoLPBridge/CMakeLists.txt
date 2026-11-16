# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(PicoLPBridge C CXX ASM)
pico_sdk_init()

if (NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/wifi_config.h)
    message(FATAL_ERROR "Falta wifi_config.h: copia wifi_config.example.h como wifi_config.h y pon la red del laboratorio")
endif()

add_executable(PicoLPBridge main.c mcp2515.c)

pico_set_program_name(PicoLPBridge "PicoLPBridge")
pico_set_program_version(PicoLPBridge "0.1")

pico_enable_stdio_uart(PicoLPBridge 0)
pico_enable_stdio_usb(PicoLPBridge 1)

target_link_libraries(PicoLPBridge
        pico_stdlib
        hardware_spi
        hardware_watchdog
        pico_cyw43_arch_lwip_poll)

target_compile_definitions(PicoLPBridge PRIVATE PICO_STACK_SIZE=0x1000)
target_include_directories(PicoLPBridge PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(PicoLPBridge PRIVATE -Wall -Wextra)
pico_add_extra_outputs(PicoLPBridge)
