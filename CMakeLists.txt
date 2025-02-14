# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(CWirelessMIDI C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(CWirelessMIDI src/main.c
        src/bluetooth_midi.c
        ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
        ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c)

pico_set_program_name(CWirelessMIDI "CWirelessMIDI")
pico_set_program_version(CWirelessMIDI "0.1")
target_include_directories(CWirelessMIDI PUBLIC src)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/ring_buffer_lib)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/pico-w-ble-midi-lib)
# Add bluetooth library
target_link_libraries(CWirelessMIDI 
        pico_cyw43_arch_none
        pico_btstack_cyw43
        pico_btstack_ble
        ble_midi_service_lib
        ring_buffer_lib
        )

# Add the standard library to the build
target_link_libraries(CWirelessMIDI
        pico_stdlib
        pico_multicore
        tinyusb_board
        tinyusb_host)

target_include_directories(CWirelessMIDI PRIVATE
        src) 

# Add USB PIO
add_subdirectory(lib/Pico-PIO-USB)
target_link_libraries(CWirelessMIDI
        pico_pio_usb)
# pico_enable_stdio_usb(CWirelessMIDI 1) 

pico_enable_stdio_uart(CWirelessMIDI 1)
pico_enable_stdio_usb(CWirelessMIDI 0)

# Add MIDI host library
add_subdirectory(lib/usb_midi_host)
target_link_libraries(CWirelessMIDI
        usb_midi_host_app_driver)        

# Add the standard include files to the build
target_include_directories(CWirelessMIDI PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        src
)

# pico_btstack_make_gatt_header(CWirelessMIDI PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/pico-w-ble-midi-lib/midi_service.gatt)
pico_btstack_make_gatt_header(CWirelessMIDI PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/bluetooth_midi.gatt ${CMAKE_CURRENT_LIST_DIR}/lib/pico-w-ble-midi-lib)


pico_add_extra_outputs(CWirelessMIDI)

