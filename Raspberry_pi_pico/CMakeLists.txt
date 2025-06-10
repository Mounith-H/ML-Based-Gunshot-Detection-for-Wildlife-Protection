# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Set up project
project(gunshot_detector C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add NRF24L01 library
add_library(nrf24l01 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Pico_NRF24L01/NRF24/NRF24.cpp
)

# Set up includes for NRF24L01 library
target_include_directories(nrf24l01 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Pico_NRF24L01/NRF24
)

target_link_libraries(nrf24l01 PUBLIC
    pico_stdlib
    hardware_spi
)

# Add main executable first
add_executable(gunshot_detector
    Raspberry_pi_pico.cpp
    model_handler.c
    gps_handler.cpp
)

# Generate PIO header
pico_generate_pio_header(gunshot_detector ${CMAKE_CURRENT_LIST_DIR}/inmp441.pio)
pico_generate_pio_header(gunshot_detector ${CMAKE_CURRENT_LIST_DIR}/power_control.pio)

# Add minmea library
add_library(minmea STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/minmea/minmea.c
)

# Set up minmea includes
target_include_directories(minmea PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/minmea
    ${CMAKE_CURRENT_SOURCE_DIR}  # For time_utils.h
)

# Add compiler definitions for minmea
target_compile_definitions(minmea PRIVATE
    timegm=mktime  # Use mktime instead of timegm
)

# Set up includes and link libraries for the main executable
target_include_directories(gunshot_detector PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/minmea
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Pico_NRF24L01/NRF24
)

# Link libraries
target_link_libraries(gunshot_detector PRIVATE 
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_pio
    hardware_dma
    hardware_uart
    minmea
    nrf24l01
)

# Configure UART/USB settings and generate output files
pico_enable_stdio_uart(gunshot_detector 1)  # Enable UART for printf
pico_enable_stdio_usb(gunshot_detector 0)   # Disable USB
pico_add_extra_outputs(gunshot_detector)     # Generate bin/hex/uf2 files