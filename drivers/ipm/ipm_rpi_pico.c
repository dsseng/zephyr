/*
 * Copyright (c) 2025 Dan Collins
 * Copyright (c) 2025 Dmitrii Sharshakov <d3dx12.xx@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT raspberrypi_pico_sio_fifo


#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/ipm.h>

#include <zephyr/logging/log.h>
#include "hardware/structs/sio.h"
#include "hardware/structs/psm.h"

LOG_MODULE_REGISTER(ipm_rpi_pico, CONFIG_IPM_LOG_LEVEL);

struct rpi_pico_mailbox_config {
	sio_hw_t *const sio_regs;
	psm_hw_t *const psm_regs;
};

struct rpi_pico_mailbox_data {
	ipm_callback_t cb;
	void *user_data;
};

static struct rpi_pico_mailbox_config rpi_pico_mailbox_config = {
	.sio_regs = (sio_hw_t *)DT_INST_REG_ADDR_BY_NAME(0, sio),
	.psm_regs = (psm_hw_t *)DT_INST_REG_ADDR_BY_NAME(0, psm),
};
static struct rpi_pico_mailbox_data rpi_pico_mailbox_data;

static inline void mailbox_put_blocking(uint32_t value)
{
	while (!(rpi_pico_mailbox_config.sio_regs->fifo_st & SIO_FIFO_ST_RDY_BITS)) {
		;
	}

	rpi_pico_mailbox_config.sio_regs->fifo_wr = value;

	/* Inform other CPU about FIFO update. */
	__SEV();
}

static inline uint32_t mailbox_pop_blocking(void)
{
	while (!(rpi_pico_mailbox_config.sio_regs->fifo_st & SIO_FIFO_ST_VLD_BITS)) {
		__WFE();
	}

	return rpi_pico_mailbox_config.sio_regs->fifo_rd;
}

static inline void mailbox_flush(void)
{
	/* Read all valid data from the mailbox. */
	while (rpi_pico_mailbox_config.sio_regs->fifo_st & SIO_FIFO_ST_VLD_BITS) {
		(void)rpi_pico_mailbox_config.sio_regs->fifo_rd;
	}
}

static int rpi_pico_mailbox_send(const struct device *dev, int wait, uint32_t id, const void *data, int size)
{
	ARG_UNUSED(data);

	if (size != 0) {
		return -EMSGSIZE;
	}

	if (id > UINT32_MAX) {
		return -EINVAL;
	}

	struct rpi_pico_mailbox_config *config = (struct rpi_pico_mailbox_config *)dev->config;
	if (!(config->sio_regs->fifo_st & SIO_FIFO_ST_RDY_BITS) && !wait) {
		LOG_ERR("Mailbox FIFO is full, cannot send message");
		return -EBUSY;
	}

	mailbox_put_blocking(id);

    return 0;
}

static void rpi_pico_mailbox_register_callback(const struct device *dev, ipm_callback_t cb, void *user_data)
{
	struct rpi_pico_mailbox_data *data = dev->data;

	data->cb = cb;
	data->user_data = user_data;
}

static int rpi_pico_mailbox_max_data_size_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* FIFO mailbox allows a single 32 bit value to be sent - and we
	 * use that as the channel identifier. */
	return 0;
}

static unsigned int rpi_pico_mailbox_max_id_val_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* FIFO mailbox allows a single 32 bit value to be sent - and we
	 * use that as the channel identifier. */
	return UINT32_MAX;
}

static int rpi_pico_mailbox_set_enabled(const struct device *dev, int enable)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(enable);
	// FIXME: enable IRQ here?
	return 0;
}

static inline bool address_in_range(uint32_t addr, uint32_t base, uint32_t size)
{
	return addr >= base && addr < base + size;
}

static int rpi_pico_reset_cpu1(void)
{
	uint32_t val;

	/* Power off, and wait for it to take effect. */
	hw_set_bits(&rpi_pico_mailbox_config.psm_regs->frce_off, PSM_FRCE_OFF_PROC1_BITS);
	while (!(rpi_pico_mailbox_config.psm_regs->frce_off & PSM_FRCE_OFF_PROC1_BITS)) {
		;
	}

	/* Power back on, and we can wait for a '0' in the FIFO to know
	 * that it has come back.
	 */
	hw_clear_bits(&rpi_pico_mailbox_config.psm_regs->frce_off, PSM_FRCE_OFF_PROC1_BITS);
	val = mailbox_pop_blocking();

	return val == 0 ? 0 : -EIO;
}

static void rpi_pico_boot_cpu1(uint32_t vector_table_addr, uint32_t stack_ptr, uint32_t pc)
{
	/* We synchronise with CPU1 and then we can hand over the memory addresses. */
	uint32_t cmds[] = {0, 0, 1, vector_table_addr, stack_ptr, pc};
	uint32_t seq = 0;

	do {
		uint32_t cmd = cmds[seq], rsp;

		if (cmd == 0) {
			mailbox_flush();
			__SEV();
		}

		mailbox_put_blocking(cmd);
		rsp = mailbox_pop_blocking();

		seq = (cmd == rsp) ? seq + 1 : 0;
	} while (seq < ARRAY_SIZE(cmds));
}

static void rpi_pico_mailbox_isr(const struct device *dev) {
	// Clear status
    rpi_pico_mailbox_config.sio_regs->fifo_st = 0xff;

	while (rpi_pico_mailbox_config.sio_regs->fifo_st & SIO_FIFO_ST_VLD_BITS) {
		uint32_t msg = rpi_pico_mailbox_config.sio_regs->fifo_rd;
		LOG_DBG("Received message: 0x%08x", msg);
		struct rpi_pico_mailbox_data *data = dev->data;
		if (data->cb) {
			// Only send the channel ID to the callback, no data.
			data->cb(dev, data->user_data, msg, 0);
		}
	}
}

static int rpi_pico_mailbox_init(const struct device *dev)
{
// FIXME: replace with DT reference to the cpu1 boot image
#if CONFIG_SOC_RP2350A_BOOT_CPU1
	/* Flash addresses are defined in our partition table from the device tree. */
	uint32_t cpu1_flash_base = DT_REG_ADDR(DT_NODELABEL(sram0_cpu1));
	uint32_t cpu1_flash_size = DT_REG_SIZE(DT_NODELABEL(sram0_cpu1));

	uint32_t *cpu1_vector_table = (uint32_t *)cpu1_flash_base;
	uint32_t cpu1_stack_ptr = cpu1_vector_table[0];
	uint32_t cpu1_reset_ptr = cpu1_vector_table[1];

	/* RAM address used to check arguments. */
	uint32_t sram0_start = DT_REG_ADDR(DT_NODELABEL(sram0));
	uint32_t sram0_size = DT_REG_SIZE(DT_NODELABEL(sram0));

	// FIXME: make these checks optional and disabled by default
	/* Stack pointer shall point within RAM. */
	if (!address_in_range(cpu1_stack_ptr, sram0_start, sram0_size)) {
		LOG_ERR("CPU1 stack pointer invalid.");
		return -EINVAL;
	}

	/* Reset pointer shall point to code within the CPU1 partition. */
	if (!address_in_range(cpu1_reset_ptr, cpu1_flash_base, cpu1_flash_size)) {
		LOG_ERR("CPU1 reset pointer invalid.");
		return -EINVAL;
	}

	LOG_WRN("CPU1 vector table at 0x%p", cpu1_vector_table);
	LOG_WRN("CPU1 stack pointer at 0x%08x", cpu1_stack_ptr);
	LOG_WRN("CPU1 reset pointer at 0x%08x", cpu1_reset_ptr);

	if (rpi_pico_reset_cpu1() != 0) {
		LOG_ERR("CPU1 reset failed.");
		return -EIO;
	}

	rpi_pico_boot_cpu1((uint32_t)cpu1_vector_table, cpu1_stack_ptr, cpu1_reset_ptr);

#endif /* CONFIG_SOC_RP2350A_BOOT_CPU1 */

	// Initialize mbox for communication.
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, sio_irq_fifo, irq),
		DT_INST_IRQ_BY_NAME(0, sio_irq_fifo, priority),
		rpi_pico_mailbox_isr, DEVICE_DT_INST_GET(0), 0);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static DEVICE_API(ipm, rpi_pico_mailbox_driver_api) = {
	.send = rpi_pico_mailbox_send,
	.register_callback = rpi_pico_mailbox_register_callback,
	.max_data_size_get = rpi_pico_mailbox_max_data_size_get,
	.max_id_val_get = rpi_pico_mailbox_max_id_val_get,
	.set_enabled = rpi_pico_mailbox_set_enabled,
};

DEVICE_DT_INST_DEFINE(0,
	&rpi_pico_mailbox_init,
	NULL,
	&rpi_pico_mailbox_data, &rpi_pico_mailbox_config,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
	&rpi_pico_mailbox_driver_api);
