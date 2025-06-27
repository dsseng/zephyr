/*
 * Notice: this licensing information only applies to this file with glue code.
 * For the actual implementation license and author data please refer to the can2040 project.
 *
 * Copyright (c) 2025 Dmitrii Sharshakov <d3dx12.xx@gmail.com>
 *
 * Parts based on can_loopback.c, which is:
 * Copyright (c) 2021 Vestas Wind Systems A/S
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h"
#include "zephyr/drivers/pinctrl.h"
#include <stdint.h>
#define DT_DRV_COMPAT can2040_can2040

#include <stdbool.h>

#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include "can2040_ll.h"

#define CONFIG_CAN_MAX_FILTER 16
#define CONFIG_CAN_CAN2040_TX_THREAD_STACK_SIZE 512
#define CONFIG_CAN_CAN2040_TX_THREAD_PRIORITY 2
#define CONFIG_CAN_CAN2040_TX_MSGQ_SIZE 16

LOG_MODULE_REGISTER(can_can2040, CONFIG_CAN_LOG_LEVEL);

struct can_can2040_frame {
	struct can_frame frame;
	can_tx_callback_t cb;
	void *cb_arg;
};

struct can_can2040_filter {
	can_rx_callback_t rx_cb;
	void *cb_arg;
	struct can_filter filter;
};

struct can_can2040_config {
	const struct can_driver_config common;
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;
	const uint32_t clk_freq;
	void (*irq_enable_func)(const struct device *dev);
};

struct can_can2040_data {
	struct can2040 can2040;

	struct can_driver_data common;
	struct can_can2040_filter filters[CONFIG_CAN_MAX_FILTER];
	struct k_mutex mtx;
	struct k_msgq tx_msgq;
	char msgq_buffer[CONFIG_CAN_CAN2040_TX_MSGQ_SIZE * sizeof(struct can_can2040_frame)];
	struct k_thread tx_thread_data;

	K_KERNEL_STACK_MEMBER(tx_thread_stack,
		      CONFIG_CAN_CAN2040_TX_THREAD_STACK_SIZE);
};

static void receive_frame(const struct device *dev,
			  const struct can_frame *frame,
			  struct can_can2040_filter *filter)
{
	struct can_frame frame_tmp = *frame;

	LOG_DBG("Receiving %d bytes. Id: 0x%x, ID type: %s %s",
		frame->dlc, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");

	filter->rx_cb(dev, &frame_tmp, filter->cb_arg);
}

static void tx_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct can_can2040_data *data = dev->data;
	struct can_can2040_frame frame;
	struct can_can2040_filter *filter;
	int ret;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_DBG("... Starting TX thread for %s", dev->name);

	while (1) {
		ret = k_msgq_get(&data->tx_msgq, &frame, K_FOREVER);
		if (ret < 0) {
			LOG_DBG("Pend on TX queue returned without valid frame (err %d)", ret);
			continue;
		}
		LOG_DBG("TX frame received: %d bytes. Id: 0x%x, ID type: %s %s",
   frame.frame.dlc, frame.frame.id,
   (frame.frame.flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
   (frame.frame.flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");
		frame.cb(dev, 0, frame.cb_arg);

		// if ((data->common.mode & CAN_MODE_LOOPBACK) == 0U) {
		// 	continue;
		// }

		k_mutex_lock(&data->mtx, K_FOREVER);

		for (int i = 0; i < CONFIG_CAN_MAX_FILTER; i++) {
			filter = &data->filters[i];
			if (filter->rx_cb != NULL &&
			    can_frame_matches_filter(&frame.frame, &filter->filter)) {
				LOG_DBG("tx->receive");
				receive_frame(dev, &frame.frame, filter);
			}
		}

		k_mutex_unlock(&data->mtx);
	}
}

static int can_can2040_send(const struct device *dev,
			     const struct can_frame *frame,
			     k_timeout_t timeout, can_tx_callback_t callback,
			     void *user_data)
{
	struct can_can2040_data *data = dev->data;
	int ret;

	LOG_DBG("Sending %d bytes on %s. Id: 0x%x, ID type: %s %s",
		frame->dlc, dev->name, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
		return -EINVAL;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	struct can2040_msg tx;
	tx.dlc = frame->dlc;
	tx.id = frame->id;
	tx.id |= (frame->flags & CAN_FRAME_IDE) ? CAN2040_ID_EFF : 0;
	tx.id |= (frame->flags & CAN_FRAME_RTR) ? CAN2040_ID_RTR : 0;
	LOG_DBG("TX frame: id=0x%x, dlc=%d, flags=0x%02x",
		tx.id, tx.dlc, frame->flags);
	memcpy(tx.data, frame->data, frame->dlc);

	ret = can2040_transmit(&data->can2040, &tx);
	if (ret < 0) {
		LOG_DBG("TX queue full (err %d)", ret);
		return -EAGAIN;
	}

	return 0;
}


static inline int get_free_filter(struct can_can2040_filter *filters)
{
	for (int i = 0; i < CONFIG_CAN_MAX_FILTER; i++) {
		if (filters[i].rx_cb == NULL) {
			return i;
		}
	}

	return -ENOSPC;
}

static int can_can2040_add_rx_filter(const struct device *dev, can_rx_callback_t cb,
				      void *cb_arg, const struct can_filter *filter)
{
	struct can_can2040_data *data = dev->data;
	struct can_can2040_filter *loopback_filter;
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

static void can_can2040_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_can2040_data *data = dev->data;

	if (filter_id < 0 || filter_id >= ARRAY_SIZE(data->filters)) {
		LOG_ERR("filter ID %d out-of-bounds", filter_id);
		return;
	}

	LOG_DBG("Remove filter ID: %d", filter_id);
	k_mutex_lock(&data->mtx, K_FOREVER);
	data->filters[filter_id].rx_cb = NULL;
	k_mutex_unlock(&data->mtx);
}

static int can_can2040_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK;

	return 0;
}

// Called from an ISR
static void
can_can2040_callback(struct can2040 *can2040_dev, uint32_t notify, struct can2040_msg *msg)
{
	const struct device *dev = can2040_dev->rx_cb_user_data;
	LOG_DBG("can2040_cb (%s): notify=0x%08x, msg->id=0x%08x, msg->dlc=%d", dev->name, notify, msg->id, msg->dlc);
}

static void can_can2040_pio_irq_handler(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;
	// LOG_DBG("IRQ %s %p", dev->name, (void *)&data->can2040);
	can2040_pio_irq_handler(&data->can2040);
}

static int can_can2040_start(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;
	const struct can_can2040_config *config = dev->config;

	if (data->common.started) {
		return -EALREADY;
	}

	memset(&data->can2040, 0, sizeof(data->can2040));
	data->can2040.pio_hw = pio_rpi_pico_get_pio(config->piodev);
	data->can2040.rx_cb_user_data = (void *)dev;
	can2040_callback_config(&data->can2040, can_can2040_callback);

	config->irq_enable_func(dev);

	LOG_DBG("Enable device %s %p, clock: %u", dev->name, (void *)&data->can2040, config->clk_freq);

	// FIXME: use DT clock and pinctrl, as well as CAN API for bitrate
	can2040_ll_data_state_clear_bits(&data->can2040);
	data->can2040.gpio_rx = 4;
	data->can2040.gpio_tx = 5;
	can2040_ll_pio_set_clkdiv(&data->can2040, config->clk_freq, 1000000);
	can2040_ll_pio_sm_setup(&data->can2040);
	can2040_ll_data_state_go_discard(&data->can2040);

	data->common.started = true;

	return pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
}

static int can_can2040_stop(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;

	if (!data->common.started) {
		return -EALREADY;
	}

	data->common.started = false;

	k_msgq_purge(&data->tx_msgq);

	return 0;
}

static int can_can2040_set_mode(const struct device *dev, can_mode_t mode)
{
	struct can_can2040_data *data = dev->data;

	if (data->common.started) {
		return -EBUSY;
	}

#ifdef CONFIG_CAN_FD_MODE
	if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_FD)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}
#else
	if ((mode & ~(CAN_MODE_LOOPBACK)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}
#endif /* CONFIG_CAN_FD_MODE */

	data->common.mode = mode;

	return 0;
}

static int can_can2040_set_timing(const struct device *dev,
				   const struct can_timing *timing)
{
	struct can_can2040_data *data = dev->data;

	ARG_UNUSED(timing);

	if (data->common.started) {
		return -EBUSY;
	}

	return 0;
}

static int can_can2040_get_state(const struct device *dev, enum can_state *state,
				  struct can_bus_err_cnt *err_cnt)
{
	struct can_can2040_data *data = dev->data;

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

static void can_can2040_set_state_change_callback(const struct device *dev,
						   can_state_change_callback_t cb,
						   void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
}

static int can_can2040_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

	/* Recommended CAN clock from CiA 601-3 */
	*rate = MHZ(80);

	return 0;
}

static int can_can2040_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	return CONFIG_CAN_MAX_FILTER;
}

static DEVICE_API(can, can_can2040_driver_api) = {
	.get_capabilities = can_can2040_get_capabilities,
	.start = can_can2040_start,
	.stop = can_can2040_stop,
	.set_mode = can_can2040_set_mode,
	.set_timing = can_can2040_set_timing,
	.send = can_can2040_send,
	.add_rx_filter = can_can2040_add_rx_filter,
	.remove_rx_filter = can_can2040_remove_rx_filter,
	.get_state = can_can2040_get_state,
	.set_state_change_callback = can_can2040_set_state_change_callback,
	.get_core_clock = can_can2040_get_core_clock,
	.get_max_filters = can_can2040_get_max_filters,
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

static int can_can2040_init(const struct device *dev)
{
	struct can_can2040_data *data = dev->data;
	k_tid_t tx_tid;

	k_mutex_init(&data->mtx);

	for (int i = 0; i < CONFIG_CAN_MAX_FILTER; i++) {
		data->filters[i].rx_cb = NULL;
	}

	k_msgq_init(&data->tx_msgq, data->msgq_buffer, sizeof(struct can_can2040_frame),
		    CONFIG_CAN_CAN2040_TX_MSGQ_SIZE);

	tx_tid = k_thread_create(&data->tx_thread_data, data->tx_thread_stack,
				 K_KERNEL_STACK_SIZEOF(data->tx_thread_stack),
				 tx_thread, (void *)dev, NULL, NULL,
				 CONFIG_CAN_CAN2040_TX_THREAD_PRIORITY,
				 0, K_NO_WAIT);
	if (!tx_tid) {
		LOG_ERR("ERROR spawning tx thread");
		return -1;
	}

	k_thread_name_set(tx_tid, dev->name);

	return 0;
}

#define CAN_CAN2040_MAX_BITRATE 1000000

#define CAN_CAN2040_INIT(inst)									\
	PINCTRL_DT_INST_DEFINE(inst);								\
	static void can_can2040_irq_enable_func_##n(const struct device *dev)		\
	{										\
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST_PARENT(inst), irq0, irq),\
				DT_IRQ_BY_NAME(DT_INST_PARENT(inst), irq0, priority),\
				can_can2040_pio_irq_handler,					\
			    DEVICE_DT_INST_GET(inst),					\
			    0);								\
											\
		irq_enable(DT_IRQ_BY_NAME(DT_INST_PARENT(inst), irq0, irq));\
	}					 	\
	\
	static const struct can_can2040_config can_can2040_config_##inst = {			\
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, CAN_CAN2040_MAX_BITRATE),	\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(inst)),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), \
		.clk_freq = 150000000, /* FIXME */ \
		.irq_enable_func = can_can2040_irq_enable_func_##n,			\
	};											\
												\
	static struct can_can2040_data can_can2040_data_##inst;				\
												\
	CAN_DEVICE_DT_INST_DEFINE(inst, can_can2040_init, NULL,				\
				  &can_can2040_data_##inst,					\
				  &can_can2040_config_##inst,					\
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,			\
				  &can_can2040_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_CAN2040_INIT)
