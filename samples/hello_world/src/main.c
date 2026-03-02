/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can_ipc.h>

LOG_MODULE_REGISTER(can_ipc_remote, CONFIG_CAN_LOG_LEVEL);
static struct ipc_ept ipc_ep;

static void ep_error(const char *err, void *priv)
{
	LOG_WRN("IPC error %s", err);
}

const struct device *const can_bus = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static void can_cb(const struct device *dev, int status, void *user_data) {
	struct can_ipc_proto_frame f;
	int err;

	f.flags = CAN_IPC_FRAME_IPC_SVC;
	f.id = CAN_IPC_ID_ACK;
	((int*)f.data)[0] = status;

	err = ipc_service_send(&ipc_ep, &f, sizeof(f));
	if (err < 0) {
		LOG_ERR("Failed to send ACK: %d", err);
	}
}

static void ep_rx(const void *data, size_t len, void *priv)
{
	struct can_ipc_proto_frame *f;
	struct can_frame frame;
	int err;

	if (len != sizeof(struct can_ipc_proto_frame)) {
		LOG_ERR("Length %d is not equal to expected %d", len, sizeof(struct can_ipc_proto_frame));
		return;
	}

	f = (struct can_ipc_proto_frame *)data;
	LOG_WRN("rx id %x", f->id);
	can_ipc_to_frame(f, &frame);

	err = can_send(can_bus, &frame, K_MSEC(100), can_cb, NULL);
	if (err != 0) {
		LOG_ERR("Failed to send to bus: %d", err);
	}
}

static struct ipc_ept_cfg ipc_ep_cfg = {
	.name = "can_ipc",
	.cb = {
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
	struct can_ipc_proto_frame f;
	int ret;

	can_add_rx_filter_msgq(can_bus, &rx_msgq, &filter);
	can_add_rx_filter_msgq(can_bus, &rx_msgq, &filter_ide);

	while (1) {
		k_msgq_get(&rx_msgq, &frame, K_FOREVER);
		LOG_DBG("Frame received: %u\n", frame.id);

		can_frame_to_ipc(&frame, &f);

		ret = ipc_service_send(&ipc_ep, &f, sizeof(f));
		if (ret < 0) {
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
		LOG_ERR("CAN bus %s not ready", can_bus->name);
		return 0;
	}

	err = can_start(can_bus);
	if (err != 0) {
		LOG_ERR("Error starting CAN controller: %d", err);
		return 0;
	}

	err = ipc_service_open_instance(ipc_instance);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("IPC service instance initialization failed: %d\n", err);
	}

	err = ipc_service_register_endpoint(ipc_instance, &ipc_ep, &ipc_ep_cfg);
	if (err != 0) {
		LOG_ERR("Registering endpoint failed: %d", err);
	}

	rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
				 K_THREAD_STACK_SIZEOF(rx_thread_stack),
				 rx_thread, NULL, NULL, NULL,
				 2, 0, K_NO_WAIT);
	if (rx_tid == NULL) {
		LOG_ERR("Failed to start RX thread");
	}

	return 0;
}
