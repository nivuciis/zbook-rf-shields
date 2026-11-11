# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rfm95_spi_check LANGUAGES C)

include_directories(./include)

file(GLOB_RECURSE SRC ${CMAKE_SOURCE_DIR} "src/*.c")
target_sources(app PRIVATE ${SRC})