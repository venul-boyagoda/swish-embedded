cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(swish-embedded)

target_sources(app PRIVATE 
    src/main.c
    src/bluetooth.c
    src/bno055_driver.c
    src/button.c
    src/onboard_imu.c
    src/quaternion_utils.c
    libs/bno055.c
    libs/mahony.c
)

target_include_directories(app PRIVATE 
    src
    libs
)
