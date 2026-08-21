/*
 * Copyright (c) 2026 eInfochips (An Arrow Company)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Host Video Class (UVC) public header
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBH_UVC_H
#define ZEPHYR_INCLUDE_USB_CLASS_USBH_UVC_H

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB Host Video Class (UVC) API
 * @defgroup usbh_uvc USB Host Video Class (UVC) API
 * @ingroup usb
 * @since 4.3
 * @version 0.1.0
 * @{
 */

/**
 * @brief UVC host device status codes
 *
 * Status codes reported by the registered status callback to indicate
 * changes in the USB host UVC device lifecycle.
 */
enum usbh_uvc_dev_status {
	/** UVC device connected and enumerated successfully */
	USBH_UVC_DEV_CONNECTED,
	/** UVC device disconnected from the USB bus */
	USBH_UVC_DEV_DISCONNECTED,
	/** Video streaming has started */
	USBH_UVC_DEV_STREAMING_STARTED,
	/** Video streaming has stopped */
	USBH_UVC_DEV_STREAMING_STOPPED,
};

/**
 * @brief Callback function signature for UVC host device status changes
 *
 * @param[in] dev    Pointer to the UVC host device instance
 * @param[in] status Status code indicating the event type
 */
typedef void (*usbh_uvc_status_cb_t)(const struct device *dev,
				     enum usbh_uvc_dev_status status);

/**
 * @brief Register a status callback for UVC host device events
 *
 * Register a callback function that will be invoked when the UVC host
 * device status changes (connection, disconnection, streaming start/stop).
 *
 * The callback is invoked in the context of the USB host stack thread.
 * The user should ensure that the callback execution does not block.
 *
 * Passing NULL as @p cb unregisters any previously registered callback.
 *
 * @param[in] dev Pointer to the UVC host device
 * @param[in] cb  Callback function, or NULL to unregister
 *
 * @return 0 on success, negative errno code on failure
 * @retval -ENODEV if the device is invalid
 */
int usbh_uvc_set_status_cb(const struct device *dev, usbh_uvc_status_cb_t cb);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBH_UVC_H */
