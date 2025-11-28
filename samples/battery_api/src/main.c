#include <nrfx_saadc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#include <zephyr/sys/printk.h>

#define SIMPLE_MODE 0


/* Timer configuration */
#define TIMER_INST_IDX 			20
#define TIMER_INTERVAL_SECONDS 	5
#define SECONDS_TO_MS			1000
#define TIMER_CC_CHANNEL       	NRF_TIMER_CC_CHANNEL0
#define TIMER_CC_SHORT_CLEAR	NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK

/* SAADC configuration */
typedef enum
{
    SAADC_SAMPLING,     //< Triggers SAADC sampling task on external timer event.
	SAADC_START			//< Configures SAADC after previous samppling has ended.	
} gppi_channels_purpose_t;

/** @brief Array of GPPI channels for battery monitoring. */
static uint8_t m_gppi_channels[2];

#if SIMPLE_MODE
#define BUFFER_COUNT 1UL
#else
#define BUFFER_COUNT 2UL
#endif

#define BUFFER_SIZE 1UL
#define RESOLUTION NRF_SAADC_RESOLUTION_10BIT

/** @brief Samples buffer to store values from a single channel */
#if SIMPLE_MODE
static uint16_t m_sample_buffer_single[BUFFER_SIZE];
#else
static uint16_t m_sample_buffers[BUFFER_COUNT][BUFFER_SIZE];
#endif

static const struct adc_dt_spec vbatt_monitor = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);


/**
 * @brief Function for handling SAADC driver events.
 *
 * @param[in] p_event Pointer to an SAADC driver event.
 */
void saadc_handler(nrfx_saadc_evt_t const * p_event)
{
    static uint16_t buffer_index = 1;
    nrfx_err_t status;

    switch (p_event->type) {
    case NRFX_SAADC_EVT_CALIBRATEDONE:
        printk("SAADC calibration done\n");
        break;

    case NRFX_SAADC_EVT_READY:
        printk("SAADC ready\n");
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
#if SIMPLE_MODE
	status = nrfx_saadc_buffer_set(m_sample_buffer_single[0], BUFFER_SIZE);
#else
	status = nrfx_saadc_buffer_set(m_sample_buffers[(buffer_index++)%2], BUFFER_SIZE);
#endif
        NRFX_ASSERT(status == NRFX_SUCCESS);
        printk("NRFX_SAADC_EVT_BUF_REQ\n");
        break;

    case NRFX_SAADC_EVT_DONE:
        printk("SAADC done: %d samples\n", p_event->data.done.size);
        break;

    case NRFX_SAADC_EVT_LIMIT:
        /* Limit interrupt triggered */
        if (p_event->data.limit.limit_type == NRF_SAADC_LIMIT_LOW) {
            printk("IRQ LIMITL triggered on channel %d\n", p_event->data.limit.channel);
            /* Your limit handling code here */
        }
        break;

    case NRFX_SAADC_EVT_FINISHED:
        printk("SAADC finished\n");
        break;

    default:
        break;
    }
}

void timer_handler(nrf_timer_event_t event_type, void * p_context){

	printk("TIMER IRQ\n");
}

static int timer_setup(nrfx_timer_t *timer_inst)
{

	// IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)),
    //         IRQ_PRIO_LOWEST,
    //         NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_IDX),
    //         0, 0);

    nrfx_err_t status;
    uint32_t desired_ticks;

	uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst->p_reg);

	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_config.p_context = NULL; // vbatt_monitor_timer / mpsl timer

    status = nrfx_timer_init(timer_inst, &timer_config, timer_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_timer_clear(timer_inst);

    desired_ticks = nrfx_timer_ms_to_ticks(timer_inst, TIMER_INTERVAL_SECONDS * SECONDS_TO_MS);
    printk("Time to wait: %u ms", TIMER_INTERVAL_SECONDS * SECONDS_TO_MS);
    printk("Timer interval: %u seconds = %u ticks\n", TIMER_INTERVAL_SECONDS, desired_ticks);


	// Don't need timer IRQ
	nrfx_timer_extended_compare(timer_inst, TIMER_CC_CHANNEL, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);


	nrfx_timer_enable(timer_inst);
	printk("Timer status: %s \n", nrfx_timer_is_enabled(timer_inst) ? "enabled" : "disabled");

    return 0;
}

static int saadc_setup(void)
{

	int err;
	nrfx_err_t status;


	if (!adc_is_ready_dt(&vbatt_monitor)) {
		printk("ADC controller device %s not ready\n", vbatt_monitor.dev->name);
		return 0;
	}

	err = adc_channel_setup_dt(&vbatt_monitor);
	if (err < 0) {
		printk("Could not setup channel (%d)\n", err);
		return 0;
	}

	/* Initialize nrfx_saadc if not already done by Zephyr driver */
    status = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (status != NRFX_SUCCESS && status != NRFX_ERROR_ALREADY) {
        printk("nrfx_saadc_init failed: 0x%08x\n", status);
        return 0;
    } // See if this is required


	uint32_t channel_mask = nrfx_saadc_channels_configured_get();
#if SIMPLE_MODE
	status = nrfx_saadc_simple_mode_set(channel_mask,
                                          RESOLUTION,
                                          NRF_SAADC_OVERSAMPLE_DISABLED,
                                          saadc_handler);
	status = nrfx_saadc_buffer_set(m_sample_buffer_single, BUFFER_SIZE);
	NRFX_ASSERT(status == NRFX_SUCCESS);
#else
	
	nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	status = nrfx_saadc_advanced_mode_set(channel_mask,
                                          RESOLUTION,
                                          &adv_config,
                                          saadc_handler);

	status = nrfx_saadc_buffer_set(m_sample_buffers[0], BUFFER_SIZE);
    NRFX_ASSERT(status == NRFX_SUCCESS);
	status = nrfx_saadc_buffer_set(m_sample_buffers[1], BUFFER_SIZE);
	NRFX_ASSERT(status == NRFX_SUCCESS);
#endif
	
    NRFX_ASSERT(status == NRFX_SUCCESS);

	/* Set up limit interrupts */
    status = nrfx_saadc_limits_set(vbatt_monitor.channel_id, 80, 5000); //CONFIG_VBAT_MONITOR_LOWER_THRESHOLD_MV
    NRFX_ASSERT(status == NRFX_SUCCESS);

	status = nrfx_saadc_mode_trigger();
	NRFX_ASSERT(status == NRFX_SUCCESS);


	/* Start calibration (required before first use) */
    // status = nrfx_saadc_offset_calibrate(NULL);
    // NRFX_ASSERT(status == NRFX_SUCCESS);

	return 0;

}

static int gppi_setup(nrfx_timer_t *timer_inst)
{
	nrfx_err_t status;
	status = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_SAMPLING]);
	NRFX_ASSERT(status == NRFX_SUCCESS);
	status = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_START]);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_SAMPLING],
        nrfx_timer_compare_event_address_get(timer_inst, TIMER_CC_CHANNEL),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

	nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_START],
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

	nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_START]));
	nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_SAMPLING]));

	return 0;
}

int main(void){

	int err;
	#define TIMER_IDX 20
	nrfx_timer_t timer_dev = NRFX_TIMER_INSTANCE(TIMER_IDX);

	err = gppi_setup(&timer_dev);
	if (err != 0) {
		printk("GPPI setup failed: %d\n", err);
		return 0;
	}
	
	err = saadc_setup();
	if (err != 0) {
		printk("Timer setup failed: %d\n", err);
		return 0;
	}
	
	err = timer_setup(&timer_dev);
	if (err != 0) {
		printk("Timer setup failed: %d\n", err);
		return 0;
	}

	while(true)
	{
		k_sleep(K_MSEC(1000));
#if SIMPLE_MODE
		// printk("SAADC buffer value %u \n", (m_sample_buffer_single[0]));
#else
		// printk("SAADC buffer values %u, %u \n", *(m_sample_buffers[0]),*(m_sample_buffers[1]));
#endif
	}
}