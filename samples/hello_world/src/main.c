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
#include <zephyr/drivers/can_ipc.h>
#include <stdio.h>

LOG_MODULE_REGISTER(can_ipc, CONFIG_CAN_LOG_LEVEL);
static struct ipc_ept ipc_ep;

static void ep_bound(void *priv)
{
	// LOG_WRN("BBBBB: bound");
	// FIXME: start/stop bus?
}

static void ep_unbound(void *priv)
{
	LOG_WRN("BBBBB: unbound");
	// FIXME: start/stop bus?
}

static void ep_error(const char *err, void *priv)
{
	LOG_WRN("BBBBB: IPC error %s", err);
}

const struct device *const can_bus = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static void ep_rx(const void *data, size_t len, void *priv)
{
	struct can_ipc_proto_frame *f;
	struct can_frame frame;
	int err;

	LOG_WRN("BBBBB: rx %d", (len > 0 ? ((uint32_t *)data)[0] : 1337));
	if (len != sizeof(struct can_ipc_proto_frame)) {
		LOG_ERR("Length %d is not equal to expected %d", len, sizeof(struct can_ipc_proto_frame));
		return;
	}

	f = (struct can_ipc_proto_frame *)data;
	LOG_WRN("BBBBB: rx id %x", f->id);
	can_ipc_to_frame(f, &frame);

	err = can_send(can_bus, &frame, K_MSEC(100), NULL, NULL);
	if (err != 0) {
		LOG_ERR("Failed to send to bus: %d", err);
	}
}

static struct ipc_ept_cfg ipc_ep_cfg = {
	.name = "can_ipc",
	.cb = {
		.bound    = ep_bound,
		.unbound  = ep_unbound,
		.received = ep_rx,
		.error    = ep_error,
	},
};

CAN_MSGQ_DEFINE(rx_msgq, 2);

K_THREAD_STACK_DEFINE(rx_thread_stack, 512);
struct k_thread rx_thread_data;

void rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	const struct can_filter filter = {
		.flags = 0,
		.id = 0,
		.mask = 0
	};
	const struct can_filter filter_ide = {
		.flags = CAN_FILTER_IDE,
		.id = 0,
		.mask = 0
	};
	struct can_frame frame;
	struct can_ipc_proto_frame *f;
	uint32_t size = sizeof(*f);
	int ret;

	can_add_rx_filter_msgq(can_bus, &rx_msgq, &filter);
	can_add_rx_filter_msgq(can_bus, &rx_msgq, &filter_ide);

	LOG_WRN("BBBBB thread running");

	while (1) {
		k_msgq_get(&rx_msgq, &frame, K_FOREVER);

		LOG_WRN("BBBBB Frame received: %u\n", frame.id);

		ret = ipc_service_get_tx_buffer(&ipc_ep, (void **)&f, &size, K_MSEC(100));
		if (ret != 0) {
			LOG_ERR("Error acquiring TX buffer: %d", ret);
		}

		can_frame_to_ipc(&frame, f);
		LOG_WRN("BBBBB sending id %x", f->id);

		ret = ipc_service_send(&ipc_ep, f, sizeof(*f));
		if (ret != 0) {
			LOG_ERR("Error sending IPC: %d", ret);
			continue;
		}
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

	err = can_start(can_bus);
	if (err != 0) {
		LOG_ERR("BBBBB Error starting CAN controller: %d", err);
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
