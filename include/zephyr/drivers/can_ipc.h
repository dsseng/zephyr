/*
 * Copyright (c) 2026 Dmitrii Sharshakov
 *
 * Partially based on include/zephyr/drivers/can.h
 * Copyright (c) 2021 Vestas Wind Systems A/S
 * Copyright (c) 2018 Karsten Koenig
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CAN_IPC_H_
#define ZEPHYR_INCLUDE_DRIVERS_CAN_IPC_H_

#include "zephyr/drivers/can.h"
#include <stdint.h>

/** Frame uses extended (29-bit) CAN ID */
#define CAN_IPC_FRAME_IDE BIT(0)

/** Frame is a Remote Transmission Request (RTR) */
#define CAN_IPC_FRAME_RTR BIT(1)

struct can_ipc_proto_frame {
	/** Standard (11-bit) or extended (29-bit) CAN identifier. */
	uint32_t id;
	/** Data Length Code (DLC) indicating data length in bytes. */
	uint8_t dlc;
	/** Flags. @see @ref CAN_IPC_FRAME_*. */
	uint8_t flags;
	/** The frame payload data. */
	/** Payload data accessed as unsigned 8 bit values. */
	uint8_t data[8];
} __attribute__((__packed__));

static inline void can_frame_to_ipc(
    const struct can_frame *from,
    struct can_ipc_proto_frame *to
) {
    to->id = from->id;
    to->flags = 0;
    if (from->flags & CAN_FRAME_IDE) {
        to->flags |= CAN_IPC_FRAME_IDE;
    }
    if (from->flags & CAN_FRAME_RTR) {
        to->flags |= CAN_IPC_FRAME_RTR;
    }
    to->dlc = from->dlc;
    memcpy(to->data, from->data, sizeof(to->data));
}

static inline void can_ipc_to_frame(
    const struct can_ipc_proto_frame *from,
    struct can_frame *to
) {
	to->id = from->id;
	to->dlc = from->dlc;
	memcpy(to->data, from->data, sizeof(from->data));
	to->flags = 0;
	if (from->flags & CAN_IPC_FRAME_IDE) {
		to->flags |= CAN_FRAME_IDE;
	}
	if (from->flags & CAN_IPC_FRAME_RTR) {
		to->flags |= CAN_FRAME_RTR;
	}
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CAN_IPC_H_ */