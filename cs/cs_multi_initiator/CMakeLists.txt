#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(channel_sound_ras_initiator)

FILE(GLOB app_sources src/*.c src/*.cpp)
# NORDIC SDK APP START
target_sources(app PRIVATE
  ${app_sources}
  # Sofia's trilateration (shared with the rest of the project, used as is)
  ${CMAKE_CURRENT_LIST_DIR}/../../trilateration/trilateration.cpp
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../trilateration)

if(CONFIG_IL_MAVLINK)
  set(MAVLINK_LIB ${CMAKE_CURRENT_LIST_DIR}/../../lib/mavlink)
  if(NOT EXISTS ${MAVLINK_LIB}/common/mavlink.h)
    message(FATAL_ERROR "lib/mavlink is empty: run 'git submodule update --init lib/mavlink'")
  endif()
  # The team's MAVLink packer, used as is
  target_sources(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../mavlink/mavlink_packer.cpp)
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../mavlink
    ${MAVLINK_LIB}
  )
  # MAVLink headers use packed structs; silence the noisy GCC warning
  target_compile_options(app PRIVATE -Wno-address-of-packed-member)
endif()
# NORDIC SDK APP END
