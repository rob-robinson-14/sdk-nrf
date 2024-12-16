/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for Nordic Semiconductor nRF71 family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Nordic Semiconductor nRF71 family processor.
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#if defined(NRF_APPLICATION)
#include <cmsis_core.h>
#include <hal/nrf_glitchdet.h>
#endif
#include <soc/nrfx_coredep.h>

#include <system_nrf7120_enga.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

int nordicsemi_nrf71_init(void)
{
	/* Enable ICACHE */
	sys_cache_instr_enable();

	return 0;
}

void arch_busy_wait(uint32_t time_us)
{
	nrfx_coredep_delay_us(time_us);
}

SYS_INIT(nordicsemi_nrf71_init, PRE_KERNEL_1, 0);
