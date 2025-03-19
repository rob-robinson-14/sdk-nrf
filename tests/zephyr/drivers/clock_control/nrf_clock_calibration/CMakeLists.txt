#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

set(KCONFIG_ROOT "\${ZEPHYR_BASE}/tests/drivers/clock_control/nrf_clock_calibration/Kconfig")

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_clock_calibration)

FILE(GLOB app_sources ${ZEPHYR_BASE}/tests/drivers/clock_control/nrf_clock_calibration/src/*.c)
target_sources(app PRIVATE ${app_sources})
