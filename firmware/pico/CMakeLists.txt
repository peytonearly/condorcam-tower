cmake_minimum_required(VERSION 3.13)

# ---------------------------------------------------
# Modern Pico SDK (>=2.1) - no manual toolchain file
# ---------------------------------------------------
include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
project(pico_quad_serial C CXX ASM)

# Import and intialize the SDK
pico_sdk_init()

# Executable
add_executable(pico_quad_serial
    main.c
)

# PIO program
pico_generate_pio_header(pico_quad_serial ${CMAKE_CURRENT_LIST_DIR}/quadrature.pio)

# Link libraries
target_link_libraries(pico_quad_serial
    pico_stdlib
    hardware_pio
    hardware_irq
)

# Enable USB stdio
pico_enable_stdio_usb(pico_quad_serial 1)
pico_enable_stdio_uart(pico_quad_serial 0)

# Generate UF2 / BIN / ELF
pico_add_extra_outputs(pico_quad_serial)