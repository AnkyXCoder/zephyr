/*
 * SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/class/usbd_uvc.h>
#include <zephyr/usb/class/usbh_uvc.h>
#include <zephyr/ztest.h>
#include <sample_usbd.h>

#include "../../../../../drivers/video/video_common.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

const struct video_format test_formats[] = {
	{.pixelformat = VIDEO_PIX_FMT_YUYV, .width = 640, .height = 480},
	{.pixelformat = VIDEO_PIX_FMT_YUYV, .width = 320, .height = 240},
	{.pixelformat = VIDEO_PIX_FMT_YUYV, .width = 160, .height = 120},
};

const struct device *const uvc_dev = DEVICE_DT_GET(DT_NODELABEL(uvc_device));
const struct device *const video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));

ZTEST(uvc_test, test_virtual_device_virtual_host)
{
	const struct device *uvc_dev;

	uvc_dev = device_get_binding("usbh_uvc_0");
	zassert_not_null(uvc_dev, "No USB host UVC instance available");
	LOG_INF("%s", uvc_dev->name);

	/* TODO: test the video devices here. */
}

static volatile enum usbh_uvc_dev_status last_status;
static volatile int status_cb_count;

static void test_status_cb(const struct device *dev, enum usbh_uvc_dev_status status)
{
	last_status = status;
	status_cb_count++;
	LOG_INF("Status callback: dev=%s status=%d count=%d", dev->name, status, status_cb_count);
}

ZTEST(uvc_test, test_status_cb_register_unregister)
{
	const struct device *host_uvc_dev;
	int ret;

	host_uvc_dev = device_get_binding("usbh_uvc_0");
	zassert_not_null(host_uvc_dev, "No USB host UVC instance available");

	/* Register a status callback */
	ret = usbh_uvc_set_status_cb(host_uvc_dev, test_status_cb);
	zassert_ok(ret, "Failed to register status callback");

	/* Unregister the callback by passing NULL */
	ret = usbh_uvc_set_status_cb(host_uvc_dev, NULL);
	zassert_ok(ret, "Failed to unregister status callback");

	/* Re-register the callback */
	ret = usbh_uvc_set_status_cb(host_uvc_dev, test_status_cb);
	zassert_ok(ret, "Failed to re-register status callback");
}

static struct usbd_context *test_usbd;

USBH_CONTROLLER_DEFINE(test_uhs_ctx, DEVICE_DT_GET(DT_NODELABEL(zephyr_uhc0)));

struct usbh_context *const uhs_ctx = &test_uhs_ctx;

void *uvc_test_enable(void)
{
	int ret;

	uvc_device_init(uvc_dev, video_dev);

	for (size_t i = 0; i < ARRAY_SIZE(test_formats); i++) {
		struct video_format fmt = test_formats[i];

		ret = video_estimate_fmt_size(&fmt);
		zassert_ok(ret);

		ret = uvc_device_add_format(uvc_dev, &fmt);
		zassert_ok(ret);
	}

	ret = uvc_device_enable(uvc_dev);
	zassert_ok(ret, "Failed to initialize UVC device class");

	ret = usbh_init(uhs_ctx);
	zassert_ok(ret, "Failed to initialize USB host");

	ret = usbh_enable(uhs_ctx);
	zassert_ok(ret, "Failed to enable USB host");

	ret = uhc_bus_reset(uhs_ctx->dev);
	zassert_ok(ret, "Failed to signal bus reset");

	ret = uhc_bus_resume(uhs_ctx->dev);
	zassert_ok(ret, "Failed to signal bus resume");

	ret = uhc_sof_enable(uhs_ctx->dev);
	zassert_ok(ret, "Failed to enable SoF generator");

	LOG_INF("Host controller enabled");

	test_usbd = sample_usbd_setup_device(NULL);
	zassert_not_null(test_usbd, "Failed to setup USB device");

	ret = usbd_init(test_usbd);
	zassert_ok(ret, "Failed to initialize device support");

	ret = usbd_enable(test_usbd);
	zassert_ok(ret, "Failed to enable device support");

	LOG_INF("Device support enabled");

	/* Allow the host time to reset the device. */
	k_msleep(200);

	return NULL;
}

void uvc_test_shutdown(void *f)
{
	int ret;

	ret = usbd_disable(test_usbd);
	zassert_ok(ret, "Failed to disable device support");

	ret = usbd_shutdown(test_usbd);
	zassert_ok(ret, "Failed to shutdown device support");

	ret = uvc_device_shutdown(uvc_dev);
	zassert_ok(ret, "Failed to shutdown UVC device class");

	LOG_INF("Device support disabled");

	ret = usbh_disable(uhs_ctx);
	zassert_ok(ret, "Failed to disable USB host");

	ret = usbh_shutdown(uhs_ctx);
	zassert_ok(ret, "Failed to shutdown host support");

	LOG_INF("Host controller disabled");
}

ZTEST_SUITE(uvc_test, NULL, uvc_test_enable, NULL, NULL, uvc_test_shutdown);
