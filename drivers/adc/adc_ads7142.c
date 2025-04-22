/* TI ADS7142 ADC
 *
 * Copyright (c) 2025 Ankitkumar Modi (ankit.modi912@gmail.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ads7142

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(adc_ads7142);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADS7142_RESOLUTION 12U

#define ADS7142_DATA_VALID_TIMEOUT 100

/* Opcodes for commands */
/* General */
#define ADS7142_OC_GENERAL          0x00
/* Single Register Read */
#define ADS7142_OC_SINGLE_REG_READ  0x10
/* Single Register Write */
#define ADS7142_OC_SINGLE_REG_WRITE 0x08
/* Single Bit Set */
#define ADS7142_OC_SET_BIT          0x18
/* Single Bit Clear */
#define ADS7142_OC_CLEAR_BIT        0x20
/* Block Register Read */
#define ADS7142_OC_BLOCK_READ       0x30
/* Block Register Write */
#define ADS7142_OC_BLOCK_WRITE      0x28

/* Registers */
/* Reset registers */
#define ADS7142_WKEY                  0x17
#define ADS7142_DEVICE_RESET          0x14
/* Functional mode select registers */
#define ADS7142_OFFSET_CAL            0x15
#define ADS7142_OPMODE_SEL            0x1C
/* Manual Mode with Channel 0 only */
#define ADS7142_OPMODE_SEL_MANUALCH0  (0)
#define ADS7142_OPMODE_SEL_MANUALCH0  (1)
/* Manual Mode with Auto Sequencing enabled */
#define ADS7142_OPMODE_SEL_MANUALSEQ  (4)
#define ADS7142_OPMODE_SEL_MANUALSEQ  (5)
/* Autonomous monitoring mode with AUTO sequencing enabled */
#define ADS7142_OPMODE_SEL_MONITORING (6)
/* High precision mode with AUTO sequencing enabled */
#define ADS7142_OPMODE_SEL_HIGHPREC   (7)

/* OPMode Status register */
#define ADS7142_OPMODE_STATUS                 0x00
#define ADS7142_OPMODE_STATUS_OPMODE_MSK      (3)
/* Device functional mode */
#define ADS7142_OPMODE_STATUS_OPMODE_MANUAL   (0)
#define ADS7142_OPMODE_STATUS_OPMODE_AUTO     (2)
#define ADS7142_OPMODE_STATUS_OPMODE_HIGHPREC (3)
/* High Speed Mode */
#define ADS7142_OPMODE_STATUS_HS_MODE         BIT(2)

/* Input config register */
#define ADS7142_CH_INPUT_CFG                   0x24
#define ADS7142_CH_INPUT_CFG_TCSE              (0)
#define ADS7142_CH_INPUT_CFG_SCSE              (1)
#define ADS7142_CH_INPUT_CFG_SCPD              (2)
/* Analog mux and sequencer registers */
#define ADS7142_AUTO_SEQ_CHEN                  0x20
#define ADS7142_AUTO_SEQ_CHEN_CH0              BIT(0)
#define ADS7142_AUTO_SEQ_CHEN_CH1              BIT(1)
#define ADS7142_START_SEQUENCE                 0x1E
#define ADS7142_START_SEQUENCE_SEQ_START       BIT(0)
#define ADS7142_ABORT_SEQUENCE                 0x1F
#define ADS7142_ABORT_SEQUENCE_SEQ_ABORT       BIT(0)
#define ADS7142_SEQUENCE_STATUS                0x04
#define ADS7142_SEQUENCE_STATUS_SEQ_ERR_ST_MSK (0x06)
#define ADS7142_SEQUENCE_STATUS_SEQ_DISABLED   (0x00)
#define ADS7142_SEQUENCE_STATUS_SEQ_ENABLED    (0x02)
#define ADS7142_SEQUENCE_STATUS_SEQ_ERROR      (0x06)
/* Oscillator and timing control registers */
#define ADS7142_OSC_SEL                        0x18
#define ADS7142_OSC_SEL_HSZ_LP                 BIT(0)
#define ADS7142_NCLK_SEL                       0x19
#define ADS7142_NCLK_SEL_MSK                   0xFF
/* Data buffer control register */
#define ADS7142_DATA_BUFFER_OPMODE             0x2C
#define ADS7142_DATA_BUFFER_OPMODE_STOP_BURST  (0)
#define ADS7142_DATA_BUFFER_OPMODE_START_BURST (1)
#define ADS7142_DATA_BUFFER_OPMODE_PRE_ALERT   (4)
#define ADS7142_DATA_BUFFER_OPMODE_POST_ALERT  (6)
#define ADS7142_DOUT_FORMAT_CFG                0x28
#define ADS7142_DOUT_FORMAT_CFG_12B            (0)
#define ADS7142_DOUT_FORMAT_CFG_12BCH          (1)
#define ADS7142_DOUT_FORMAT_CFG_12BCHDV        (2)
#define ADS7142_DATA_BUFFER_STATUS             0x01
/* Accumulator control register */
#define ADS7142_ACC_EN                         0x30
#define ADS7142_ACC_CH0_LSB                    0x08
#define ADS7142_ACC_CH0_MSB                    0x09
#define ADS7142_ACC_CH1_LSB                    0x0A
#define ADS7142_ACC_CH1_MSB                    0x0B
#define ADS7142_ACC_STATUS                     0x02
/* Digital window comparator registers */
#define ADS7142_ALERT_DWC_EN                   0x37
#define ADS7142_ALERT_DWC_EN_BLOCK_EN          BIT(0)
#define ADS7142_ALERT_CHEN                     0x34
#define ADS7142_DWC_HTH_CH0_LSB                0x38
#define ADS7142_DWC_HTH_CH0_MSB                0x39
#define ADS7142_DWC_LTH_CH0_LSB                0x3A
#define ADS7142_DWC_LTH_CH0_MSB                0x3B
#define ADS7142_DWC_HYS_CH0                    0x40
#define ADS7142_DWC_HTH_CH1_LSB                0x3C
#define ADS7142_DWC_HTH_CH1_MSB                0x3D
#define ADS7142_DWC_LTH_CH1_LSB                0x3E
#define ADS7142_DWC_LTH_CH1_MSB                0x3F
#define ADS7142_DWC_HYS_CH1                    0x41
#define ADS7142_PRE_ALT_EVT_CNT                0x36
#define ADS7142_ALT_TRIG_CHID                  0x03
#define ADS7142_ALT_LOW_FLAGS                  0x0C
#define ADS7142_ALT_LOW_FLAGS_CH0              BIT(0)
#define ADS7142_ALT_LOW_FLAGS_CH1              BIT(1)
#define ADS7142_ALT_HIGH_FLAGS                 0x0E
#define ADS7142_ALT_HIGH_FLAGS_CH0             BIT(0)
#define ADS7142_ALT_HIGH_FLAGS_CH1             BIT(1)

#define ADS7142_THRESHOLD_MSK  0xFFF
#define ADS7142_HYSTERESIS_MSK 0x3F

#define ADS7142_HSO_FREQ     20000000
#define ADS7142_HSO_NCLK_MIN 21
#define ADS7142_HSO_NCLK_MAX 255
#define ADS7142_HSO_MIN_SS   78431
#define ADS7142_HSO_MAX_SS   952380
#define ADS7142_LPO_FREQ     10504
#define ADS7142_LPO_NCLK_MIN 18
#define ADS7142_LPO_NCLK_MAX 255
#define ADS7142_LPO_MIN_SS   41
#define ADS7142_LPO_MAX_SS   583

#define ADS7142_CHANNEL_COUNT 2

struct ads7142_config {
	struct i2c_dt_spec bus;
	uint8_t channels;
	bool osc_sel;
	uint32_t n_clk;
	int buffer_mode;
};

struct ads7142_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t channels;
	struct k_thread thread;
	struct k_sem acq_sem;
	struct k_sem alert_sem;
	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_ADS7142_ACQUISITION_THREAD_STACK_SIZE);
};

struct ads7142_channel_config {
	bool alert_low;
	bool alert_high;
	int high_threshold;
	int low_threshold;
	int hysteresis;
};

struct ads7142_channel {
	struct ads7142_channel_config config;
	uint8_t channel;
};

struct ads7142_priv {
	struct k_mutex lock; /* For syncing access to device */
	struct ads7142_config config;
	struct ads7142_channel *channels;
	uint16_t *scan_data;
	bool irq_present;
	bool monitor_pending;
};

static const char *const ads7142_ain_names[] = {
	"AIN0",
	"AIN1",
};

#define ADS7142_BUFFM_NONE        0
#define ADS7142_BUFFM_STOP_BURST  1
#define ADS7142_BUFFM_START_BURST 2
#define ADS7142_BUFFM_PRE_ALERT   3
#define ADS7142_BUFFM_POST_ALERT  4

static const char *const ti_ads7142_buffer_modes[] = {
	[ADS7142_BUFFM_NONE] = "none",
	[ADS7142_BUFFM_STOP_BURST] = "stop_burst",
	[ADS7142_BUFFM_START_BURST] = "start_burst",
	[ADS7142_BUFFM_PRE_ALERT] = "pre_alert",
	[ADS7142_BUFFM_POST_ALERT] = "post_alert",
};

static int ads7142_read_reg(const struct device *dev, enum ads1119_reg reg_addr, uint8_t *reg_val)
{
	const struct ads7142_config *config = dev->config;
	uint8_t buf[2] = {ADS7142_OC_SINGLE_REG_READ, reg_addr};
	return i2c_reg_read_byte_dt(&config->bus, buf, reg_val);
}

static int ads7142_write_reg(const struct device *dev, uint8_t reg)
{
	const struct ads7142_config *config = dev->config;
	uint8_t buf[3] = {ADS7142_OC_SINGLE_REG_WRITE, reg, data};
	return i2c_reg_write_byte_dt(&config->bus, buf, sizeof(buf));
}

static int ads7142_init(const struct device *dev)
{
	const struct ads7142_config *config = dev->config;
	struct ads7142_data *data = dev->data;
	int rc;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	adc_context_init(&data->ctx);

	k_sem_init(&data->acq_sem, 0, 1);
	k_sem_init(&data->alert_sem, 0, 1);

	/* Reset the device */
	rc = ads7142_write_reg(dev, ADS7142_DEVICE_RESET, 0x01);
	if (rc < 0) {
		LOG_ERR("Failed to reset device");
		return rc;
	}

	/* Configure operational mode */
	rc = ads7142_write_reg(dev, ADS7142_OPMODE_SEL, ADS7142_OPMODE_SEL_MANUALCH0);
	if (rc < 0) {
		LOG_ERR("Failed to configure operational mode");
		return rc;
	}

#if CONFIG_ADC_ASYNC
	k_tid_t tid = k_thread_create(&data->thread, config->stack,
				      CONFIG_ADC_ADS7142_ACQUISITION_THREAD_STACK_SIZE,
				      ads7142_acquisition_thread, (void *)dev, NULL, NULL,
				      CONFIG_ADC_ADS7142_ASYNC_THREAD_INIT_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "adc_ads7142");
#endif
	adc_context_unlock_unconditionally(&data->ctx);

	LOG_INF("ADS7142 initialized successfully");
	return rc;
}

static DEVICE_API(adc, api) = {
	.channel_setup = ads7142_channel_setup,
	.read = ads7142_read,
	.ref_internal = ADS7142_REF_INTERNAL,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads7142_adc_read_async,
#endif
};

#define ADS7142_INIT(inst)                                                                         \
	static struct ads7142_data ads7142_data_##inst;                                            \
	static const struct ads7142_config ads7142_config_##inst = {                               \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.channels = ADS7142_CHANNEL_COUNT,                                                 \
		.osc_sel = false,                                                                  \
		.n_clk = ADS7142_HSO_NCLK_MIN,                                                     \
		.buffer_mode = ADS7142_BUFFM_NONE,                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, ads7142_init, NULL, &ads7142_data_##inst,                      \
			      &ads7142_config_##inst, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,       \
			      &ads7142_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADS7142_INIT)
