# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
	${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensor/bmp390
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bmp-shuttle)

if(DEFINED DEBUG_PRINTF)
  zephyr_compile_definitions(DEBUG_PRINTF=${DEBUG_PRINTF})
endif()
if(DEFINED SAME_PRIO)
  zephyr_compile_definitions(SAME_PRIO)
endif()
if(DEFINED FORKS)
  zephyr_compile_definitions(FORKS=${FORKS})
endif()

target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include/zephyr/portability)
target_sources(app PRIVATE 
  src/main.c
  src/sensors/sensors.c
)

target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include/zephyr/portability)
target_include_directories(app PRIVATE
  inc
  src
  src/sensors
)

zephyr_library_include_directories(.)

