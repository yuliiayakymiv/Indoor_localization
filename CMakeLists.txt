# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(indoor_localization)

# --- Sources ---
target_sources(app PRIVATE
    src/main.cpp                      # Person 1: main loop
    trilateration/trilateration.cpp   # Person 1: 3D position math
    mavlink/mavlink_packer.cpp        # Person 2: MAVLink packing
)

# --- Include paths ---
target_include_directories(app PRIVATE
    trilateration
    mavlink
    lib/mavlink                       # c_library_v2 (git submodule, header-only)
)

# MAVLink headers use packed structs; silence the noisy GCC warning
target_compile_options(app PRIVATE -Wno-address-of-packed-member)
