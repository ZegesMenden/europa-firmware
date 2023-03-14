# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Users/camku/Documents/Pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(europa_test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(europa_test main.cpp )

pico_set_program_name(europa_test "europa_test")
pico_set_program_version(europa_test "0.1")

pico_enable_stdio_usb(europa_test 1)
pico_enable_stdio_uart(europa_test 0)

# Add the standard library to the build
target_link_libraries(europa_test

                      pico_stdlib
                      pico_multicore
                      hardware_spi
                      hardware_i2c
                      hardware_adc
                      hardware_pio
                      hardware_pwm
                      hardware_dma
                      hardware_timer
                      
                      )

# Add the standard include files to the build
target_include_directories(europa_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/include/drivers
)
# pico_generate_pio_header(europa_test ${CMAKE_CURRENT_LIST_DIR}/include/drivers/i2c.pio)

pico_add_extra_outputs(europa_test)

# example_auto_set_url(europa_test)

target_compile_definitions(europa_test PRIVATE PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

# pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
# target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)

# pico_set_boot_stage2(TARGET slower_boot2)