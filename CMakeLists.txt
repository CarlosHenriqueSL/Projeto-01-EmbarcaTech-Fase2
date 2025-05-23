# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Trabalho01 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Trabalho01 Trabalho01.c lib/ssd1306.c lib/numeros.c)

pico_set_program_name(Trabalho01 "Trabalho01")
pico_set_program_version(Trabalho01 "0.1")

# Generate PIO header
pico_generate_pio_header(Trabalho01 ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Trabalho01 1)
pico_enable_stdio_usb(Trabalho01 1)

# Add the standard library to the build
target_link_libraries(Trabalho01
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Trabalho01 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Trabalho01 
        hardware_i2c
        hardware_pio
        hardware_irq
        hardware_adc
        hardware_pwm
        )

pico_add_extra_outputs(Trabalho01)

