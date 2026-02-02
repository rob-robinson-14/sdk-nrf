# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(
  preeng_board.c
  board.c
)

zephyr_include_directories(.)