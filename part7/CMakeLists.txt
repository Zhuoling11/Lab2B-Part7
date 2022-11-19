
cmake_minimum_required(VERSION 3.13) # CMAKE (heading)
include(pico_sdk_import.cmake) # CMAKE -> PICO SDK (include)

project(piosequencer C CXX ASM) # CMAKE ("project")
pico_sdk_init() # PICO SDK (initialization)

add_executable(piosequencer) # CMAKE (target)

pico_generate_pio_header(piosequencer ${CMAKE_CURRENT_LIST_DIR}/piosequencer.pio) # PICO SDK (PIO)

target_sources(piosequencer PRIVATE piosequencer.c neopixel.c) # CMAKE -> C (compiler)
target_link_libraries(piosequencer PRIVATE pico_stdlib hardware_pio hardware_dma) # CMAKE -> C (linker)

pico_enable_stdio_usb(piosequencer 1) # PICO SDK (USB)
pico_enable_stdio_uart(piosequencer 0) # PICO SDK (USB)

pico_add_extra_outputs(piosequencer) # PICO SDK (UF2)
# example_auto_set_url(piosequencer)