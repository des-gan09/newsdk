#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.13.1)

# list(APPEND mcuboot_OVERLAY_CONFIG
#   "${CMAKE_CURRENT_SOURCE_DIR}/child_image/mcuboot.conf"
#   )

# list(APPEND netboot_OVERLAY_CONFIG
#   "${CMAKE_CURRENT_SOURCE_DIR}/child_image/netboot.conf"
#   )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# NORDIC SDK APP START
zephyr_library_include_directories(.)
zephyr_library_include_directories(../movus_lib)
target_sources(app PRIVATE src/main.c ../movus_lib/hal/hal_spi.c ../movus_lib/peripheral/lis3mdl.c )

# NORDIC SDK APP END



