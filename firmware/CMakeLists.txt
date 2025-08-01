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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(cuppa C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(pico-icm20948/src)

add_library(CuppaLib src/fluid_sim.cpp src/estimator.cpp src/ukf.cpp src/utils.cpp src/timer.cpp src/utils.cpp src/imu_icm20948.cpp src/comms.cpp src/charlieplex.cpp mpu6500/example/driver_mpu6500_basic.c mpu6500/src/driver_mpu6500.c src/imu_mpu6500.cpp src/driver_mpu6500_interface.c src/button.cpp src/flip.cpp TLV493D-A1B6-3DMagnetic-Sensor/src/Tlv493d.cpp TLV493D-A1B6-3DMagnetic-Sensor/src/util/BusInterface.cpp TLV493D-A1B6-3DMagnetic-Sensor/src/util/RegMask.cpp DFRobot_ICM42688/DFRobot_ICM42688.cpp)
target_link_libraries(CuppaLib
    pico_stdlib
    hardware_spi
    hardware_i2c
    pico_stdio_usb
    pico-icm20948
)

target_include_directories(CuppaLib PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/eigen
        ${CMAKE_CURRENT_LIST_DIR}/eigen
        ${CMAKE_CURRENT_LIST_DIR}/pico-icm20948/src
        ${CMAKE_CURRENT_LIST_DIR}/mpu6500/example
        ${CMAKE_CURRENT_LIST_DIR}/mpu6500/src
        ${CMAKE_CURRENT_LIST_DIR}/TLV493D-A1B6-3DMagnetic-Sensor/src/util/
        ${CMAKE_CURRENT_LIST_DIR}/TLV493D-A1B6-3DMagnetic-Sensor/src/
        ${CMAKE_CURRENT_LIST_DIR}/DFRobot_ICM42688
)

add_executable(cuppa cuppa.cpp)

# enable usb output
pico_enable_stdio_usb(cuppa 1)
pico_enable_stdio_uart(cuppa 0)

pico_set_program_name(cuppa "julier_jewel")
pico_set_program_version(cuppa "0.1")

# Add the standard include files to the build
target_include_directories(cuppa PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/eigen
        ${CMAKE_CURRENT_LIST_DIR}/pico-icm20948/src
)

pico_add_extra_outputs(cuppa)

target_link_libraries(cuppa CuppaLib pico_stdlib pico_stdio_usb hardware_i2c pico-icm20948 pico_multicore pico_sync)


