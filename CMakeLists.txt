cmake_minimum_required(VERSION 3.13.1)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C CXX)

INCLUDE_DIRECTORIES(include)

target_sources(app PRIVATE src/main.cpp src/gpio.cpp src/can.c src/pl455.cpp src/module_data.cpp src/slave.cpp src/master.cpp) 