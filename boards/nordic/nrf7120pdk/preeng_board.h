/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file Configuration macros for the Nordic Semiconductor NRF71 pre-engineering family processors.
 */

#ifndef _NORDICSEMI_NRF7120PDK_H_
#define _NORDICSEMI_NRF7120PDK_H_

int configure_playout_capture(uint32_t rx_mode, uint32_t tx_mode, uint32_t rx_holdoff_length,
			      uint32_t rx_wrap_length, uint32_t back_to_back_mode);

#endif /* _NORDICSEMI_NRF7120PDK_H_ */
