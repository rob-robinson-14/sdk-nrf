# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

if(NOT DEFINED NRFX_DIR)
  set(NRFX_DIR ${ZEPHYR_BASE}/../modules/hal/nordic_internal/nrfx CACHE PATH "nrfx Directory")
endif()

zephyr_include_directories_ifdef(CONFIG_HAS_NRFX .)
add_subdirectory_ifdef(CONFIG_HAS_NRFX nrfx)
add_subdirectory(${ZEPHYR_BASE}/modules/hal_nordic ${CMAKE_CURRENT_BINARY_DIR}/modules/hal_nordic)
