# Copyright (c) 2025 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

zephyr_library_sources(soc_preeng.c)

zephyr_include_directories(.)
add_subdirectory(${ZEPHYR_BASE}/soc/nordic/common ${CMAKE_BINARY_DIR}/common)
add_subdirectory(${ZEPHYR_BASE}/soc/nordic/nrf71 ${CMAKE_BINARY_DIR}/nrf71)
