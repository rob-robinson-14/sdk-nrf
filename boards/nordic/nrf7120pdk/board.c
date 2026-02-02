/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @brief System/hardware module for Nordic Semiconductor nRF71 family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Nordic Semiconductor nRF71 pre-engineering family processor.
 */

#ifdef __NRF_TFM__
#include <zephyr/autoconf.h>
#endif

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#ifndef __NRF_TFM__
#include <zephyr/cache.h>
#endif

#if defined(NRF_APPLICATION)
#include <cmsis_core.h>
#include <hal/nrf_glitchdet.h>
#endif

#include <nrfx.h>
#include <lib/nrfx_coredep.h>

#include <hal/nrf_spu.h>
#include <hal/nrf_mpc.h>
#include <hal/nrf_lfxo.h>

static inline NRF_SPU_Type *spu_instance_from_peripheral_addr(uint32_t peripheral_addr)
{
	/* See the SPU chapter in the IPS for how this is calculated */

	uint32_t apb_bus_number = peripheral_addr & 0x00FC0000;

	return (NRF_SPU_Type *)(0x50000000 | apb_bus_number);
}

static void spu_peripheral_config_non_secure(const uint32_t periph_base_address, bool periph_lock)
{
	uint8_t periph_id = NRFX_PERIPHERAL_ID_GET(periph_base_address);

#if NRF_SPU_HAS_MEMORY
	/* ASSERT checking that this is not an explicit Secure peripheral */
	NRFX_ASSERT(
		(NRF_SPU->PERIPHID[periph_id].PERM & SPU_PERIPHID_PERM_SECUREMAPPING_Msk) !=
		(SPU_PERIPHID_PERM_SECUREMAPPING_Secure << SPU_PERIPHID_PERM_SECUREMAPPING_Pos));

	nrf_spu_peripheral_set(NRF_SPU, periph_id, 0 /* Non-Secure */, 0 /* Non-Secure DMA */,
			       periph_lock);
#else
	NRF_SPU_Type *nrf_spu = spu_instance_from_peripheral_addr(periph_base_address);

	uint8_t spu_id = NRFX_PERIPHERAL_ID_GET(nrf_spu);

	uint8_t index = periph_id - spu_id;

	nrf_spu_periph_perm_secattr_set(nrf_spu, index, false /* Non-Secure */);
	nrf_spu_periph_perm_dmasec_set(nrf_spu, index, false /* Non-Secure */);
	nrf_spu_periph_perm_lock_enable(nrf_spu, index);
#endif
}

void wifi_setup(void)
{
	/* EMU platform uses UART 20 for the Wi-Fi console */
	/* Wi-Fi VPR uses UART 20 (PORT 2 Pin 2 is for the TX) */
	spu_peripheral_config_non_secure(NRF_UARTE20_S_BASE, true);
	nrf_spu_feature_secattr_set(NRF_SPU00, NRF_SPU_FEATURE_GPIO_PIN, 2, 2,
				    SPU_FEATURE_GPIO_PIN_SECATTR_NonSecure);

	/* Set permission for TXD */
	nrf_spu_feature_secattr_set(NRF_SPU20, NRF_SPU_FEATURE_GPIO_PIN, 1, 4,
				    SPU_FEATURE_GPIO_PIN_SECATTR_NonSecure);
}

void board_early_init_hook(void)
{
    wifi_setup();
}