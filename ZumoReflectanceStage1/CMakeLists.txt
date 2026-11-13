cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
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

set(PICO_BOARD pico CACHE STRING "RP2040 board")

include(pico_sdk_import.cmake)

project(ZumoReflectanceStage1 C CXX ASM)

pico_sdk_init()

add_executable(ZumoReflectanceStage1 main.c)

target_link_libraries(ZumoReflectanceStage1
    pico_stdlib
    hardware_gpio
    hardware_sync
)

pico_enable_stdio_usb(ZumoReflectanceStage1 1)
pico_enable_stdio_uart(ZumoReflectanceStage1 0)

pico_add_extra_outputs(ZumoReflectanceStage1)
