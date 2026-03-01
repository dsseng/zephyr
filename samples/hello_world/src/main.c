/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/ipc/ipc_service.h"
#include "zephyr/logging/log.h"
#include <stdio.h>

LOG_MODULE_REGISTER(can_ipc, CONFIG_CAN_LOG_LEVEL);
static struct ipc_ept ipc_ep;

static void ep_bound(void *priv)
{
	LOG_WRN("BBBBB: bound");
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

int main(void)
{
	const struct device *ipc_instance =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_can_ipc));
	int err;

	err = ipc_service_open_instance(ipc_instance);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("IPC service instance initialization failed: %d\n", err);
	}

	err = ipc_service_register_endpoint(ipc_instance, &ipc_ep, &ipc_ep_cfg);
	if (err) {
		LOG_ERR("Registering endpoint failed with %d", err);
	}

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	LOG_WRN("BBBBB Hello World! %s\n", CONFIG_BOARD_TARGET);

	return 0;
}
