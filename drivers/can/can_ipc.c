/*
 * Copyright (c) 2026 Dmitrii Sharshakov
 *
 * Basec on can_loopback.c
 * Copyright (c) 2021 Vestas Wind Systems A/S
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_can_ipc

#include <stdbool.h>

#include <zephyr/drivers/can.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/can_ipc.h>

LOG_MODULE_REGISTER(can_ipc, CONFIG_CAN_LOG_LEVEL);

struct can_ipc_frame {
	struct can_frame frame;
	can_tx_callback_t cb;
	void *cb_arg;
};

struct can_ipc_filter {
	can_rx_callback_t rx_cb;
	void *cb_arg;
	struct can_filter filter;
};

struct can_ipc_config {
	const struct can_driver_config common;
	const struct device *ipc_instance;
	struct ipc_ept_cfg ipc_ep_cfg;
};

struct can_ipc_data {
	struct can_driver_data common;
	struct can_ipc_filter filters[CONFIG_CAN_IPC_MAX_FILTERS];
	struct k_mutex mtx;

	struct ipc_ept ep;

	K_KERNEL_STACK_MEMBER(tx_thread_stack,
		      CONFIG_CAN_IPC_TX_THREAD_STACK_SIZE);
};

static void receive_frame(const struct device *dev,
			  const struct can_frame *frame,
			  struct can_ipc_filter *filter)
{
	struct can_frame frame_tmp = *frame;

	LOG_DBG("Receiving %d bytes. Id: 0x%x, ID type: %s %s",
		frame->dlc, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");

	filter->rx_cb(dev, &frame_tmp, filter->cb_arg);
}

static int can_ipc_send(const struct device *dev,
			     const struct can_frame *frame,
			     k_timeout_t timeout, can_tx_callback_t callback,
			     void *user_data)
{
	struct can_ipc_data *data = dev->data;
	uint8_t max_dlc = CAN_MAX_DLC;
	struct can_ipc_proto_frame *f;
	uint32_t size = sizeof(*f);
	int ret;

	LOG_DBG("Sending %d bytes on %s. Id: 0x%x, ID type: %s %s",
		frame->dlc, dev->name, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (frame->dlc > max_dlc) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, max_dlc);
		return -EINVAL;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	ret = ipc_service_get_tx_buffer(&data->ep, (void **)&f, &size, K_MSEC(100));
	if (ret != 0) {
		return ret;
	}

	can_frame_to_ipc(frame, f);

	ret = ipc_service_send(&data->ep, f, sizeof(*f));
	if (ret != 0) {
		return ret;
	}

	return 0;
}


static inline int get_free_filter(struct can_ipc_filter *filters)
{
	for (int i = 0; i < CONFIG_CAN_IPC_MAX_FILTERS; i++) {
		if (filters[i].rx_cb == NULL) {
			return i;
		}
	}

	return -ENOSPC;
}

static int can_ipc_add_rx_filter(const struct device *dev, can_rx_callback_t cb,
				      void *cb_arg, const struct can_filter *filter)
{
	struct can_ipc_data *data = dev->data;
	struct can_ipc_filter *loopback_filter;
	int filter_id;

	LOG_DBG("Setting filter ID: 0x%x, mask: 0x%x", filter->id, filter->mask);

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mtx, K_FOREVER);
	filter_id = get_free_filter(data->filters);

	if (filter_id < 0) {
		LOG_ERR("No free filter left");
		k_mutex_unlock(&data->mtx);
		return filter_id;
	}

	loopback_filter = &data->filters[filter_id];

	loopback_filter->rx_cb = cb;
	loopback_filter->cb_arg = cb_arg;
	loopback_filter->filter = *filter;
	k_mutex_unlock(&data->mtx);

	LOG_DBG("Filter added. ID: %d", filter_id);

	return filter_id;
}

static void can_ipc_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_ipc_data *data = dev->data;

	if (filter_id < 0 || filter_id >= ARRAY_SIZE(data->filters)) {
		LOG_ERR("filter ID %d out-of-bounds", filter_id);
		return;
	}

	LOG_DBG("Remove filter ID: %d", filter_id);
	k_mutex_lock(&data->mtx, K_FOREVER);
	data->filters[filter_id].rx_cb = NULL;
	k_mutex_unlock(&data->mtx);
}

static int can_ipc_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL;// | CAN_MODE_LOOPBACK;

	return 0;
}

static int can_ipc_start(const struct device *dev)
{
	struct can_ipc_data *data = dev->data;
	// const struct can_ipc_config *config = dev->config;
	// int err;

	if (data->common.started) {
		return -EALREADY;
	}

	// err = ipc_service_open_instance(config->ipc_instance);
	// if (err && (err != -EALREADY)) {
	// 	LOG_ERR("Failed to open IPC instance: %d\n", err);
	// 	return err;
	// }

	// err = ipc_service_register_endpoint(config->ipc_instance, &data->ep, &config->ipc_ep_cfg);
	// if (err != 0) {
	// 	LOG_ERR("Failed to register EP: %d", err);
	// 	return err;
	// }

	data->common.started = true;

	return 0;
}

static int can_ipc_stop(const struct device *dev)
{
	struct can_ipc_data *data = dev->data;
	// const struct can_ipc_config *config = dev->config;
	// int err;

	if (!data->common.started) {
		return -EALREADY;
	}

	// err = ipc_service_deregister_endpoint(&data->ep);
	// if (err != 0) {
	// 	LOG_ERR("Failed to deregister EP: %d", err);
	// 	return err;
	// }

	// err = ipc_service_close_instance(config->ipc_instance);
	// if (err != 0) {
	// 	LOG_ERR("Failed to close IPC instance: %d\n", err);
	// 	return err;
	// }

	data->common.started = false;

	return 0;
}

static int can_ipc_set_mode(const struct device *dev, can_mode_t mode)
{
	struct can_ipc_data *data = dev->data;

	if (data->common.started) {
		return -EBUSY;
	}

	// if ((mode & ~(CAN_MODE_LOOPBACK)) != 0) {
	if (mode != CAN_MODE_NORMAL) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	data->common.mode = mode;

	return 0;
}

static int can_ipc_set_timing(const struct device *dev,
				   const struct can_timing *timing)
{
	struct can_ipc_data *data = dev->data;

	ARG_UNUSED(timing);

	if (data->common.started) {
		return -EBUSY;
	}

	return 0;
}

static int can_ipc_get_state(const struct device *dev, enum can_state *state,
				  struct can_bus_err_cnt *err_cnt)
{
	struct can_ipc_data *data = dev->data;

	if (state != NULL) {
		if (data->common.started) {
			*state = CAN_STATE_ERROR_ACTIVE;
		} else {
			*state = CAN_STATE_STOPPED;
		}
	}

	if (err_cnt) {
		err_cnt->tx_err_cnt = 0;
		err_cnt->rx_err_cnt = 0;
	}

	return 0;
}

static void can_ipc_set_state_change_callback(const struct device *dev,
						   can_state_change_callback_t cb,
						   void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
}

static int can_ipc_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

	/* Recommended CAN clock from CiA 601-3 */
	*rate = MHZ(80);

	return 0;
}

static int can_ipc_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	return CONFIG_CAN_IPC_MAX_FILTERS;
}

static DEVICE_API(can, can_ipc_driver_api) = {
	.get_capabilities = can_ipc_get_capabilities,
	.start = can_ipc_start,
	.stop = can_ipc_stop,
	.set_mode = can_ipc_set_mode,
	.set_timing = can_ipc_set_timing,
	.send = can_ipc_send,
	.add_rx_filter = can_ipc_add_rx_filter,
	.remove_rx_filter = can_ipc_remove_rx_filter,
	.get_state = can_ipc_get_state,
	.set_state_change_callback = can_ipc_set_state_change_callback,
	.get_core_clock = can_ipc_get_core_clock,
	.get_max_filters = can_ipc_get_max_filters,
	/* Recommended configuration ranges from CiA 601-2 */
	.timing_min = {
		.sjw = 1,
		.prop_seg = 0,
		.phase_seg1 = 2,
		.phase_seg2 = 2,
		.prescaler = 1
	},
	.timing_max = {
		.sjw = 128,
		.prop_seg = 0,
		.phase_seg1 = 256,
		.phase_seg2 = 128,
		.prescaler = 32
	},
};

static int can_ipc_init(const struct device *dev)
{
	struct can_ipc_data *data = dev->data;
	const struct can_ipc_config *config = dev->config;
	int err;

	k_mutex_init(&data->mtx);

	for (int i = 0; i < CONFIG_CAN_IPC_MAX_FILTERS; i++) {
		data->filters[i].rx_cb = NULL;
	}

	err = ipc_service_open_instance(config->ipc_instance);
	if (err && (err != -EALREADY)) {
		LOG_ERR("Failed to open IPC instance: %d\n", err);
		return err;
	}

	err = ipc_service_register_endpoint(config->ipc_instance, &data->ep, &config->ipc_ep_cfg);
	if (err != 0) {
		LOG_ERR("Failed to register EP: %d", err);
		return err;
	}

	return 0;
}

static void can_ipc_bound(void *priv)
{
	const struct device *dev = priv;

	LOG_WRN("%s: AAAAA: bound", dev->name);
}

static void can_ipc_unbound(void *priv)
{
	const struct device *dev = priv;

	LOG_WRN("%s: AAAAA: unbound", dev->name);
}

static void can_ipc_error(const char *err, void *priv)
{
	LOG_WRN("BBBBB: IPC error %s", err);
}

static void can_ipc_rx(const void *pkt, size_t len, void *priv)
{
	const struct device *dev = priv;
	struct can_ipc_data *data = dev->data;
	struct can_frame frame;
	struct can_ipc_proto_frame *f;
	struct can_ipc_filter *filter;

	if (len != sizeof(struct can_ipc_proto_frame)) {
		LOG_ERR("Length %d is not equal to expected %d", len, sizeof(struct can_ipc_proto_frame));
		return;
	}

	f = (struct can_ipc_proto_frame *)pkt;
	LOG_WRN("%s: AAAAA: rx id %x", dev->name, f->id);

	can_ipc_to_frame(f, &frame);

	k_mutex_lock(&data->mtx, K_FOREVER);

	for (int i = 0; i < CONFIG_CAN_IPC_MAX_FILTERS; i++) {
		filter = &data->filters[i];
		if (filter->rx_cb != NULL &&
			can_frame_matches_filter(&frame, &filter->filter)) {
			receive_frame(dev, &frame, filter);
		}
	}

	k_mutex_unlock(&data->mtx);
}

#define CAN_IPC_MAX_BITRATE 1000000

#define CAN_IPC_INIT(inst)									\
	static const struct can_ipc_config can_ipc_config_##inst = {			\
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, CAN_IPC_MAX_BITRATE),	\
		.ipc_instance = DEVICE_DT_GET(DT_INST_PARENT(inst)), \
		.ipc_ep_cfg = { \
			.name = "can_ipc", \
			.priv = (void *)(DEVICE_DT_INST_GET(inst)), \
			.cb = { \
				.bound    = can_ipc_bound, \
				.unbound  = can_ipc_unbound, \
				.received = can_ipc_rx, \
				.error    = can_ipc_error, \
			}, \
		}, \
	};											\
												\
	static struct can_ipc_data can_ipc_data_##inst; \
												\
	CAN_DEVICE_DT_INST_DEFINE(inst, can_ipc_init, NULL,				\
				  &can_ipc_data_##inst,					\
				  &can_ipc_config_##inst,					\
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,			\
				  &can_ipc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_IPC_INIT)
