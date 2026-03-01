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

#endif /* ZEPHYR_INCLUDE_DRIVERS_CAN_IPC_H_ */