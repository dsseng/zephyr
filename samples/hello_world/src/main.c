/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/drivers/can.h"
#include "zephyr/ipc/ipc_service.h"
#include "zephyr/kernel/thread.h"
#include "zephyr/logging/log.h"
#include <stdio.h>

LOG_MODULE_REGISTER(can_ipc, CONFIG_CAN_LOG_LEVEL);
static struct ipc_ept ipc_ep;

static void ep_bound(void *priv)
{
	// LOG_WRN("BBBBB: bound");
	// FIXME: start/stop bus?
}

static void ep_rx(const void *data, size_t len, void *priv)
{
	LOG_WRN("BBBBB: rx %d", (len > 0 ? ((uint32_t *)data)[0] : 1337));
}

static struct ipc_ept_cfg ipc_ep_cfg = {
	.name = "can_ipc",
	.cb = {
		.bound    = ep_bound,
		.received = ep_rx,
	},
};

const struct device *const can_bus = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
CAN_MSGQ_DEFINE(rx_msgq, 2);

K_THREAD_STACK_DEFINE(rx_thread_stack, 512);
struct k_thread rx_thread_data;

void rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	const struct can_filter filter = {
		.flags = CAN_FILTER_IDE,
		.id = 0,
		.mask = 0
	};
	struct can_frame frame;
	int filter_id;

	filter_id = can_add_rx_filter_msgq(can_bus, &rx_msgq, &filter);

	LOG_WRN("BBBBB thread running");

	while (1) {
		k_msgq_get(&rx_msgq, &frame, K_FOREVER);

		LOG_WRN("BBBBB Frame received: %u\n", frame.id);

		ipc_service_send(&ipc_ep, &frame.id, 4);
	}
}

int main(void)
{
	const struct device *ipc_instance =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_can_ipc));
	int err;
	k_tid_t rx_tid;

	if (!device_is_ready(can_bus)) {
		LOG_ERR("BBBBB CAN bus %s not ready.", can_bus->name);
		return 0;
	}

	err = ipc_service_open_instance(ipc_instance);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("BBBBB IPC service instance initialization failed: %d\n", err);
	}

	err = ipc_service_register_endpoint(ipc_instance, &ipc_ep, &ipc_ep_cfg);
	if (err != 0) {
		LOG_ERR("BBBBB Registering endpoint failed: %d", err);
	}

	err = can_start(can_bus);
	if (err != 0) {
		LOG_ERR("BBBBB Error starting CAN controller: %d", err);
		return 0;
	}

	rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
				 K_THREAD_STACK_SIZEOF(rx_thread_stack),
				 rx_thread, NULL, NULL, NULL,
				 2, 0, K_NO_WAIT);
	if (rx_tid == NULL) {
		LOG_ERR("BBBBB Failed to start RX thread");
	}

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	LOG_WRN("BBBBB Hello World! %s\n", CONFIG_BOARD_TARGET);

	return 0;
}
