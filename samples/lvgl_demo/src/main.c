
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <lv_demos.h>
#include <lvgl.h>

#include "lv_sample.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_lcd0));
#define MAX_PERIOD (1 * 1000 * 100) // This is equivalent to 0.1 msec time period (1kHz frequency)

int main(void)
{
	const struct device *display_dev;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

	if (!pwm_is_ready_dt(&pwm_led0)) {
		LOG_ERR("PWM device not ready, aborting test");
		return 0;
	}

	uint32_t pulse_width = MAX_PERIOD;
	/* configure the PWM Pulse Period and Pulse Width */
	pwm_set_dt(&pwm_led0, MAX_PERIOD, pulse_width);

#if defined(CONFIG_LV_USE_DEMO_MUSIC)
	lv_demo_music();
#elif defined(CONFIG_LV_USE_DEMO_BENCHMARK)
	lv_demo_benchmark();
#elif defined(CONFIG_LV_USE_DEMO_STRESS)
	lv_demo_stress();
#elif defined(CONFIG_LV_USE_DEMO_WIDGETS)
	lv_demo_widgets();
#elif defined(CONFIG_LV_USE_DEMO_SAMPLE)
	lv_demo_sample();
#else
#error Enable one of the demos CONFIG_LV_USE_DEMO_MUSIC, CONFIG_LV_USE_DEMO_BENCHMARK ,\
	CONFIG_LV_USE_DEMO_STRESS, or CONFIG_LV_USE_DEMO_WIDGETS
#endif

	lv_task_handler();
	display_blanking_off(display_dev);

	uint32_t count = 10, iteration = 0;
	while (1) {
		k_msleep(lv_task_handler());

		if ((iteration % 1000) == 0) {
			pulse_width = (MAX_PERIOD * count) / 10;
			count--;
			if (count == 0) {
				count = 10;
			}

			LOG_INF("PWM Pulse Period: %d, PWM Pulse Width: %d", MAX_PERIOD,
				pulse_width);
		}
		iteration++;
		pwm_set_dt(&pwm_led0, MAX_PERIOD, pulse_width);
	}

	return 0;
}
