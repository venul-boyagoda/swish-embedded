cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(swish-embedded)

target_sources(app PRIVATE 
    src/main.c
    libs/bno055.c
    libs/mahony.c
)

target_include_directories(app PRIVATE 
    src
    libs
)
