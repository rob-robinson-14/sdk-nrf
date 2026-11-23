/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#if defined(CONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT)
#include <gpiote_nrfx.h>
#include <hal/nrf_gpio.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_gpiote.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <zephyr/sys/sys_io.h>
#endif

#if defined(CONFIG_RAM_POWER_DOWN_LIBRARY)
#include <ram_pwrdn.h>
#endif

#define IDLE_TIME K_SECONDS(CONFIG_SAMPLE_POWER_CONSUMPTION_IDLE_SECONDS)

#if defined(CONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT)
#define LATENCY_NODE       DT_PATH(zephyr_user)
#define LATENCY_EVENT_PIN  NRF_DT_GPIOS_TO_PSEL(LATENCY_NODE, latency_event_gpios)
#define LATENCY_ISR_PIN    NRF_DT_GPIOS_TO_PSEL(LATENCY_NODE, latency_isr_gpios)
#define LATENCY_THREAD_PIN NRF_DT_GPIOS_TO_PSEL(LATENCY_NODE, latency_thread_gpios)
#define LATENCY_GPIOTE_INST \
	NRF_DT_GPIOTE_INST(LATENCY_NODE, latency_event_gpios)
#define LATENCY_GPIOTE_NODE DT_NODELABEL(_CONCAT(gpiote, LATENCY_GPIOTE_INST))

#if defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
#define SREGS30_BASE 0x4010F000UL
#else
#define SREGS30_BASE 0x5010F000UL
#endif
#define SREGS30_PDSELECT_OFFSET      0x780UL
#define SREGS30_PDSELECT_PIN0_MASK   GENMASK(3, 0)
#define SREGS30_PDSELECT_PWRABOVEELV 4U << 4
#define SREGS30_PDSELECT_ADDRESS     (SREGS30_BASE + SREGS30_PDSELECT_OFFSET)

BUILD_ASSERT(DT_NODE_HAS_PROP(LATENCY_NODE, latency_event_gpios),
	     "latency-event-gpios must be defined");
BUILD_ASSERT(DT_NODE_HAS_PROP(LATENCY_NODE, latency_isr_gpios),
	     "latency-isr-gpios must be defined");
BUILD_ASSERT(DT_NODE_HAS_PROP(LATENCY_NODE, latency_thread_gpios),
	     "latency-thread-gpios must be defined");
BUILD_ASSERT(NRF_DT_GPIOTE_INST(LATENCY_NODE, latency_isr_gpios) == LATENCY_GPIOTE_INST,
	     "Latency GPIOs must use the same GPIOTE instance");
BUILD_ASSERT(NRF_DT_GPIOTE_INST(LATENCY_NODE, latency_thread_gpios) == LATENCY_GPIOTE_INST,
	     "Latency GPIOs must use the same GPIOTE instance");

K_SEM_DEFINE(latency_wakeup_sem, 0, 1);

static nrfx_gpiote_t *const latency_gpiote =
	&GPIOTE_NRFX_INST_BY_NODE(LATENCY_GPIOTE_NODE);
static int32_t latency_grtc_channel;
static nrfx_gppi_handle_t latency_gppi_handle;

static void latency_elv_status_output_configure(void)
{
	uint32_t pdselect = sys_read32(SREGS30_PDSELECT_ADDRESS);

	pdselect &= ~SREGS30_PDSELECT_PIN0_MASK;
	pdselect |= SREGS30_PDSELECT_PWRABOVEELV;
	sys_write32(pdselect, SREGS30_PDSELECT_ADDRESS);
}

static void latency_timeout_handler(int32_t id, uint64_t expire_time, void *user_data)
{
	ARG_UNUSED(id);
	ARG_UNUSED(expire_time);
	ARG_UNUSED(user_data);

	nrf_gpio_pin_set(LATENCY_ISR_PIN);
	k_sem_give(&latency_wakeup_sem);
}

static int latency_measurement_init(void)
{
	static const nrfx_gpiote_output_config_t output_config = {
		.drive = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull = NRF_GPIO_PIN_NOPULL,
	};
	uint8_t gpiote_channel;
	int err;

	latency_elv_status_output_configure();

	if (!nrfx_gpiote_init_check(latency_gpiote)) {
		err = nrfx_gpiote_init(latency_gpiote, 0);
		if (err < 0) {
			return err;
		}
	}

	err = nrfx_gpiote_channel_alloc(latency_gpiote, &gpiote_channel);
	if (err < 0) {
		return err;
	}

	const nrfx_gpiote_task_config_t task_config = {
		.task_ch = gpiote_channel,
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
	};

	err = nrfx_gpiote_output_configure(latency_gpiote, LATENCY_EVENT_PIN,
					   &output_config, &task_config);
	if (err < 0) {
		return err;
	}

	nrfx_gpiote_out_task_enable(latency_gpiote, LATENCY_EVENT_PIN);
	nrf_gpio_cfg_output(LATENCY_ISR_PIN);
	nrf_gpio_cfg_output(LATENCY_THREAD_PIN);
	nrf_gpio_pin_clear(LATENCY_ISR_PIN);
	nrf_gpio_pin_clear(LATENCY_THREAD_PIN);

	latency_grtc_channel = z_nrf_grtc_timer_chan_alloc();
	if (latency_grtc_channel < 0) {
		return latency_grtc_channel;
	}

	err = nrfx_gppi_conn_alloc(
		z_nrf_grtc_timer_compare_evt_address_get(latency_grtc_channel),
		nrfx_gpiote_set_task_address_get(latency_gpiote, LATENCY_EVENT_PIN),
		&latency_gppi_handle);
	if (err < 0) {
		return err;
	}

	nrfx_gppi_conn_enable(latency_gppi_handle);

	return 0;
}

static int latency_measurement_sleep(void)
{
	uint64_t target_time;
	int err;

	nrfx_gpiote_clr_task_trigger(latency_gpiote, LATENCY_EVENT_PIN);
	nrf_gpio_pin_clear(LATENCY_ISR_PIN);
	nrf_gpio_pin_clear(LATENCY_THREAD_PIN);

	target_time = z_nrf_grtc_timer_get_ticks(IDLE_TIME);
	err = z_nrf_grtc_timer_set(latency_grtc_channel, target_time,
				   latency_timeout_handler, NULL);
	if (err < 0) {
		return err;
	}

	k_sem_take(&latency_wakeup_sem, K_FOREVER);
	nrf_gpio_pin_set(LATENCY_THREAD_PIN);

	return 0;
}
#endif

static void configure_ram_retention(void)
{
#if defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_USED_ONLY)
	power_down_unused_ram();
#elif defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_64K) || \
	defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_128K) || \
	defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_256K) || \
	defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_512K)
	uintptr_t ram_start = DT_REG_ADDR(DT_CHOSEN(zephyr_sram));
	uintptr_t ram_end = ram_start + DT_REG_SIZE(DT_CHOSEN(zephyr_sram));

#if defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_64K)
	uintptr_t retained_size = 64U * 1024U;
#elif defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_128K)
	uintptr_t retained_size = 128U * 1024U;
#elif defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_256K)
	uintptr_t retained_size = 256U * 1024U;
#elif defined(CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_512K)
	uintptr_t retained_size = 512U * 1024U;
#endif

	power_down_ram(ram_start + retained_size, ram_end);
#endif
	/* CONFIG_SAMPLE_POWER_CONSUMPTION_RAM_RETAIN_FULL (default): no
	 * power-down call at all -- every byte of application RAM stays retained.
	 */
}

int main(void)
{
#if defined(CONFIG_SERIAL) || defined(CONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT)
	int err;
#endif

#if defined(CONFIG_SERIAL)
	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!device_is_ready(console)) {
		return 0;
	}

	printk("\nPower consumption demo ready.\n");
#endif

#if defined(CONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT)
	err = latency_measurement_init();
	if (err < 0) {
#if defined(CONFIG_SERIAL)
		printk("Failed to initialize latency measurement: %d\n", err);
#endif
		return 0;
	}
#endif

	configure_ram_retention();

#if defined(CONFIG_SERIAL)
	uint32_t wakeups = 0;
#endif

	while (true) {
#if defined(CONFIG_SERIAL)
		/* Suspend the console (UART) before sleeping -- otherwise it
		 * stays fully active (and keeps whatever clock it depends on
		 * requested) for the whole "idle" window, which is the
		 * single biggest cause of elevated idle current in a sample
		 * like this.
		 */
		err = pm_device_action_run(console, PM_DEVICE_ACTION_SUSPEND);
		if (err != 0) {
			printk("Failed to suspend console: %d\n", err);
			return 0;
		}
#endif

#if defined(CONFIG_SAMPLE_POWER_CONSUMPTION_LATENCY_MEASUREMENT)
		err = latency_measurement_sleep();
		if (err < 0) {
			return 0;
		}
#else
		k_sleep(IDLE_TIME);
#endif

#if defined(CONFIG_SERIAL)
		err = pm_device_action_run(console, PM_DEVICE_ACTION_RESUME);
		if (err != 0) {
			return 0;
		}

		wakeups++;
		printk("Woken up %u time(s)\n", wakeups);
#endif
	}

	return 0;
}
