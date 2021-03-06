/*
 * Copyright (c) 2011-2014 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This driver provides the core support for a single RMI4-based device.
 *
 * The RMI4 specification can be found here (URL split for line length):
 *
 * http://www.synaptics.com/sites/default/files/
 *      511-000136-01-Rev-E-RMI4-Interfacing-Guide.pdf
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kconfig.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <uapi/linux/input.h>
#include "rmi_bus.h"
#include "rmi_driver.h"

#define HAS_NONSTANDARD_PDT_MASK 0x40
#define RMI4_MAX_PAGE 0xff
#define RMI4_PAGE_SIZE 0x100
#define RMI4_PAGE_MASK 0xFF00

#define RMI_DEVICE_RESET_CMD	0x01
#define DEFAULT_RESET_DELAY_MS	100

#define DEFAULT_POLL_INTERVAL_MS	13

static irqreturn_t rmi_irq_thread(int irq, void *p)
{
	struct rmi_transport_dev *xport = p;
	struct rmi_device *rmi_dev = xport->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_driver_data *data;

	data = dev_get_drvdata(&rmi_dev->dev);

	data->attn_count++;
	if (driver && driver->irq_handler && rmi_dev)
		driver->irq_handler(rmi_dev, irq);

	return IRQ_HANDLED;
}

static void rmi_poll_work(struct work_struct *work)
{
	struct rmi_driver_data *data =
			container_of(work, struct rmi_driver_data, poll_work);
	struct rmi_device *rmi_dev = data->rmi_dev;

	rmi_process_interrupt_requests(rmi_dev);
}

/*
 * This is the timer function for polling - it simply has to schedule work
 * and restart the timer.
 */
static enum hrtimer_restart rmi_poll_timer(struct hrtimer *timer)
{
	struct rmi_driver_data *data =
			container_of(timer, struct rmi_driver_data, poll_timer);

	if (!data->enabled)
		return HRTIMER_NORESTART;
	if (!work_pending(&data->poll_work))
		schedule_work(&data->poll_work);
	hrtimer_start(&data->poll_timer, data->poll_interval, HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static int enable_polling(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	dev_dbg(&rmi_dev->dev, "Polling enabled.\n");
	INIT_WORK(&data->poll_work, rmi_poll_work);
	hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->poll_timer.function = rmi_poll_timer;
	hrtimer_start(&data->poll_timer, data->poll_interval, HRTIMER_MODE_REL);

	return 0;
}

static void disable_polling(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	dev_dbg(&rmi_dev->dev, "Polling disabled.\n");
	hrtimer_cancel(&data->poll_timer);
	cancel_work_sync(&data->poll_work);
}

static void disable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	if (!data->enabled)
		return;

	if (data->polling)
		disable_polling(rmi_dev);

	if (rmi_dev->xport->ops->disable_device)
		rmi_dev->xport->ops->disable_device(rmi_dev->xport);

	if (rmi_dev->xport->irq > 0) {
		disable_irq(rmi_dev->xport->irq);
		free_irq(rmi_dev->xport->irq, rmi_dev->xport);
	}

	data->enabled = false;
}

static void rmi_free_function_list(struct rmi_device *rmi_dev)
{
	struct rmi_function *fn, *tmp;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	data->f01_container = NULL;

	/* Doing it in the reverse order so F01 will be removed last */
	list_for_each_entry_safe_reverse(fn, tmp,
					 &data->function_list, node) {
		list_del(&fn->node);
		rmi_unregister_function(fn);
	}
}

static int reset_one_function(struct rmi_function *fn)
{
	struct rmi_function_handler *fh;
	int retval = 0;

	if (!fn || !fn->dev.driver)
		return 0;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fh->reset) {
		retval = fh->reset(fn);
		if (retval < 0)
			dev_err(&fn->dev, "Reset failed with code %d.\n",
				retval);
	}

	return retval;
}

static int configure_one_function(struct rmi_function *fn)
{
	struct rmi_function_handler *fh;
	int retval = 0;

	if (!fn || !fn->dev.driver)
		return 0;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fh->config) {
		retval = fh->config(fn);
		if (retval < 0)
			dev_err(&fn->dev, "Config failed with code %d.\n",
				retval);
	}

	return retval;
}

static int rmi_driver_process_reset_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *entry;
	int retval;

	list_for_each_entry(entry, &data->function_list, node) {
		retval = reset_one_function(entry);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int rmi_driver_process_config_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *entry;
	int retval;

	list_for_each_entry(entry, &data->function_list, node) {
		retval = configure_one_function(entry);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static void process_one_interrupt(struct rmi_driver_data *data,
				  struct rmi_function *fn)
{
	struct rmi_function_handler *fh;

	if (!fn || !fn->dev.driver)
		return;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fn->irq_mask && fh->attention) {
		bitmap_and(data->fn_irq_bits, data->irq_status, fn->irq_mask,
				         data->irq_count);
		if (!bitmap_empty(data->fn_irq_bits, data->irq_count))
			fh->attention(fn, data->fn_irq_bits);
	}
}

int rmi_process_interrupt_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;
	struct rmi_function *entry;
	int error;

	if (!data || !data->f01_container || !data->irq_status)
		return 0;

	if (!rmi_dev->xport->attn_data) {
		error = rmi_read_block(rmi_dev,
				data->f01_container->fd.data_base_addr + 1,
				data->irq_status, data->num_of_irq_regs);
		if (error < 0) {
			dev_err(dev, "Failed to read irqs, code=%d\n", error);
			return error;
		}
	}

	mutex_lock(&data->irq_mutex);
	bitmap_and(data->irq_status, data->irq_status, data->current_irq_mask,
	       data->irq_count);
	/*
	 * At this point, irq_status has all bits that are set in the
	 * interrupt status register and are enabled.
	 */
	mutex_unlock(&data->irq_mutex);

	/*
	 * It would be nice to be able to use irq_chip to handle these
	 * nested IRQs.  Unfortunately, most of the current customers for
	 * this driver are using older kernels (3.0.x) that don't support
	 * the features required for that.  Once they've shifted to more
	 * recent kernels (say, 3.3 and higher), this should be switched to
	 * use irq_chip.
	 */
	list_for_each_entry(entry, &data->function_list, node)
		if (entry->irq_mask)
			process_one_interrupt(data, entry);

	if (data->input)
		input_sync(data->input);

	return 0;
}
EXPORT_SYMBOL_GPL(rmi_process_interrupt_requests);

static int enable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_transport_dev *xport;
	int retval = 0;

	if (data->enabled)
		return 0;

	if (rmi_dev->xport->ops->enable_device) {
		retval = rmi_dev->xport->ops->enable_device(rmi_dev->xport);
		if (retval)
			return retval;
	}

	retval = rmi_driver_process_config_requests(rmi_dev);
	if (retval < 0)
		return retval;

	xport = rmi_dev->xport;
	if (xport->irq) {
		retval = request_threaded_irq(xport->irq,
				xport->hard_irq ? xport->hard_irq : NULL,
				xport->irq_thread ?
					xport->irq_thread : rmi_irq_thread,
				xport->irq_flags,
				dev_name(&rmi_dev->dev), xport);
		if (retval)
			return retval;
	} else if (data->polling) {
		retval = enable_polling(rmi_dev);
		if (retval < 0)
			return retval;
	}

	data->enabled = true;

	return rmi_process_interrupt_requests(rmi_dev);
}

/**
 * rmi_driver_set_input_params - set input device id and other data.
 *
 * @rmi_dev: Pointer to an RMI device
 * @input: Pointer to input device
 *
 */
static int rmi_driver_set_input_params(struct rmi_device *rmi_dev,
				struct input_dev *input)
{
	input->dev.parent = &rmi_dev->dev;
	input->name = SYNAPTICS_INPUT_DEVICE_NAME;
	input->id.vendor  = SYNAPTICS_VENDOR_ID;
	input->id.bustype = BUS_RMI;
	return 0;
}

static void rmi_driver_set_input_name(struct rmi_device *rmi_dev,
				struct input_dev *input)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	char *device_name = rmi_f01_get_product_ID(data->f01_container);
	char *name;

	if (!device_name)
		return;

	name = devm_kasprintf(&rmi_dev->dev, GFP_KERNEL,
			      "Synaptics %s", device_name);
	if (!name)
		return;

	input->name = name;
}


static int rmi_driver_set_irq_bits(struct rmi_device *rmi_dev,
				   unsigned long *mask)
{
	int error = 0;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);
	bitmap_or(data->new_irq_mask,
		  data->current_irq_mask, mask, data->irq_count);
	/* FIXME: convert to on-wire endianness before writing */
	error = rmi_write_block(rmi_dev,
			data->f01_container->fd.control_base_addr + 1,
			data->new_irq_mask, data->num_of_irq_regs);
	if (error < 0) {
		dev_err(dev, "%s: Failed to change enabled interrupts!",
							__func__);
		goto error_unlock;
	}
	bitmap_copy(data->current_irq_mask, data->new_irq_mask,
		    data->num_of_irq_regs);

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return error;
}

static int rmi_driver_clear_irq_bits(struct rmi_device *rmi_dev,
				     unsigned long *mask)
{
	int error = 0;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);
	bitmap_andnot(data->new_irq_mask,
		  data->current_irq_mask, mask, data->irq_count);
	/* FIXME: convert to on-wire endianness before writing */
	error = rmi_write_block(rmi_dev,
			data->f01_container->fd.control_base_addr + 1,
			data->new_irq_mask, data->num_of_irq_regs);
	if (error < 0) {
		dev_err(dev, "%s: Failed to change enabled interrupts!",
							__func__);
		goto error_unlock;
	}
	bitmap_copy(data->current_irq_mask, data->new_irq_mask,
		    data->num_of_irq_regs);

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return error;
}

static int rmi_driver_irq_handler(struct rmi_device *rmi_dev, int irq)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	might_sleep();
	/*
	 * Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->f01_container) {
		dev_dbg(&rmi_dev->dev,
			 "Not ready to handle interrupts yet!\n");
		return 0;
	}

	return rmi_process_interrupt_requests(rmi_dev);
}

static int rmi_driver_reset_handler(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	int error;

	/*
	 * Can get called before the driver is fully ready to deal with
	 * this situation.
	 */
	if (!data || !data->f01_container) {
		dev_warn(&rmi_dev->dev,
			 "Not ready to handle reset yet!\n");
		return 0;
	}

	error = rmi_read_block(rmi_dev,
			       data->f01_container->fd.control_base_addr + 1,
			       data->current_irq_mask, data->num_of_irq_regs);
	/* FIXME: convert from on-wire endianness before using */
	if (error < 0) {
		dev_err(&rmi_dev->dev, "%s: Failed to read current IRQ mask.\n",
			__func__);
		return error;
	}

	error = rmi_driver_process_reset_requests(rmi_dev);
	if (error < 0)
		return error;

	error = rmi_driver_process_config_requests(rmi_dev);
	if (error < 0)
		return error;

	return 0;
}

int rmi_read_pdt_entry(struct rmi_device *rmi_dev, struct pdt_entry *entry,
			u16 pdt_address)
{
	u8 buf[RMI_PDT_ENTRY_SIZE];
	int error;

	error = rmi_read_block(rmi_dev, pdt_address, buf, RMI_PDT_ENTRY_SIZE);
	if (error) {
		dev_err(&rmi_dev->dev, "Read PDT entry at %#06x failed, code: %d.\n",
				pdt_address, error);
		return error;
	}

	entry->page_start = pdt_address & RMI4_PAGE_MASK;
	entry->query_base_addr = buf[0];
	entry->command_base_addr = buf[1];
	entry->control_base_addr = buf[2];
	entry->data_base_addr = buf[3];
	entry->interrupt_source_count = buf[4] & RMI_PDT_INT_SOURCE_COUNT_MASK;
	entry->function_version = (buf[4] & RMI_PDT_FUNCTION_VERSION_MASK) >> 5;
	entry->function_number = buf[5];

	return 0;
}
EXPORT_SYMBOL_GPL(rmi_read_pdt_entry);

static void rmi_driver_copy_pdt_to_fd(const struct pdt_entry *pdt,
				      struct rmi_function_descriptor *fd)
{
	fd->query_base_addr = pdt->query_base_addr + pdt->page_start;
	fd->command_base_addr = pdt->command_base_addr + pdt->page_start;
	fd->control_base_addr = pdt->control_base_addr + pdt->page_start;
	fd->data_base_addr = pdt->data_base_addr + pdt->page_start;
	fd->function_number = pdt->function_number;
	fd->interrupt_source_count = pdt->interrupt_source_count;
	fd->function_version = pdt->function_version;
}

#define RMI_SCAN_CONTINUE	0
#define RMI_SCAN_DONE		1

static int rmi_scan_pdt_page(struct rmi_device *rmi_dev,
			     int page,
			     void *ctx,
			     int (*callback)(struct rmi_device *rmi_dev,
					     void *ctx,
					     const struct pdt_entry *entry))
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct pdt_entry pdt_entry;
	u16 page_start = RMI4_PAGE_SIZE * page;
	u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
	u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;
	u16 addr;
	int error;
	int retval;

	for (addr = pdt_start; addr >= pdt_end; addr -= RMI_PDT_ENTRY_SIZE) {
		error = rmi_read_pdt_entry(rmi_dev, &pdt_entry, addr);
		if (error)
			return error;

		if (RMI4_END_OF_PDT(pdt_entry.function_number))
			break;

		retval = callback(rmi_dev, ctx, &pdt_entry);
		if (retval != RMI_SCAN_CONTINUE)
			return retval;
	}

	return (data->f01_bootloader_mode || addr == pdt_start) ?
					RMI_SCAN_DONE : RMI_SCAN_CONTINUE;
}

static int rmi_scan_pdt(struct rmi_device *rmi_dev, void *ctx,
			int (*callback)(struct rmi_device *rmi_dev,
					void *ctx,
					const struct pdt_entry *entry))
{
	int page;
	int retval = RMI_SCAN_DONE;

	for (page = 0; page <= RMI4_MAX_PAGE; page++) {
		retval = rmi_scan_pdt_page(rmi_dev, page, ctx, callback);
		if (retval != RMI_SCAN_CONTINUE)
			break;
	}

	return retval < 0 ? retval : 0;
}


int rmi_read_register_desc(struct rmi_device *d, u16 addr,
				struct rmi_register_descriptor *rdesc)
{
	int ret;
	u8 size_presence_reg;
	u8 buf[35];
	int presense_offset = 1;
	u8 *struct_buf;
	int reg;
	int offset = 0;
	int map_offset = 0;
	int i;
	int b;

	/*
	 * The first register of the register descriptor is the size of
	 * the register descriptor's presense register.
	 */
	ret = rmi_read(d, addr, &size_presence_reg);
	if (ret)
		return ret;
	++addr;

	if (size_presence_reg < 0 || size_presence_reg > 35)
		/* sanity check the value */
		return -EIO;

	memset(buf, 0, sizeof(buf));

	/*
	 * The presence register contains the size of the register structure
	 * and a bitmap which identified which packet registers are present
	 * for this particular register type (ie query, control, or data).
	 */
	ret = rmi_read_block(d, addr, buf, size_presence_reg);
	if (ret)
		return ret;
	++addr;

	if (buf[0] == 0) {
		presense_offset = 3;
		rdesc->struct_size = buf[1] | (buf[2] << 8);
	} else {
		rdesc->struct_size = buf[0];
	}

	for (i = presense_offset; i < size_presence_reg; i++) {
		for (b = 0; b < 8; b++) {
			if (buf[i] & (0x1 << b))
				bitmap_set(rdesc->presense_map, map_offset, 1);
			++map_offset;
		}
	}

	rdesc->num_registers = bitmap_weight(rdesc->presense_map,
						RMI_REG_DESC_PRESENSE_BITS);

	rdesc->registers = devm_kzalloc(&d->dev, rdesc->num_registers *
				sizeof(struct rmi_register_desc_item),
				GFP_KERNEL);
	if (!rdesc->registers)
		return -ENOMEM;

	/*
	 * Allocate a temporary buffer to hold the register structure.
	 * I'm not using devm_kzalloc here since it will not be retained
	 * after exiting this function
	 */
	struct_buf = kzalloc(rdesc->struct_size, GFP_KERNEL);
	if (!struct_buf)
		return -ENOMEM;

	/*
	 * The register structure contains information about every packet
	 * register of this type. This includes the size of the packet
	 * register and a bitmap of all subpackets contained in the packet
	 * register.
	 */
	ret = rmi_read_block(d, addr, struct_buf, rdesc->struct_size);
	if (ret)
		goto free_struct_buff;

	reg = find_first_bit(rdesc->presense_map, RMI_REG_DESC_PRESENSE_BITS);
	map_offset = 0;
	for (i = 0; i < rdesc->num_registers; i++) {
		struct rmi_register_desc_item *item = &rdesc->registers[i];
		int reg_size = struct_buf[offset];

		++offset;
		if (reg_size == 0) {
			reg_size = struct_buf[offset] |
					(struct_buf[offset + 1] << 8);
			offset += 2;
		}

		if (reg_size == 0) {
			reg_size = struct_buf[offset] |
					(struct_buf[offset + 1] << 8) |
					(struct_buf[offset + 2] << 16) |
					(struct_buf[offset + 3] << 24);
			offset += 4;
		}

		item->reg = reg;
		item->reg_size = reg_size;

		do {
			for (b = 0; b < 7; b++) {
				if (struct_buf[offset] & (0x1 << b))
					bitmap_set(item->subpacket_map,
						map_offset, 1);
				++map_offset;
			}
		} while (struct_buf[offset++] & 0x80);

		item->num_subpackets = bitmap_weight(item->subpacket_map,
						RMI_REG_DESC_SUBPACKET_BITS);

		dev_dbg(&d->dev, "%s: reg: %d reg size: %ld subpackets: %d\n",
			__func__, item->reg, item->reg_size,
			item->num_subpackets);

		reg = find_next_bit(rdesc->presense_map,
				RMI_REG_DESC_PRESENSE_BITS, reg + 1);
	}

free_struct_buff:
	kfree(struct_buf);
	return ret;
}
EXPORT_SYMBOL_GPL(rmi_read_register_desc);

const struct rmi_register_desc_item *rmi_get_register_desc_item(
				struct rmi_register_descriptor *rdesc, u16 reg)
{
	const struct rmi_register_desc_item *item;
	int i;

	for (i = 0; i < rdesc->num_registers; i++) {
		item = &rdesc->registers[i];
		if (item->reg == reg)
			return item;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(rmi_get_register_desc_item);

size_t rmi_register_desc_calc_size(struct rmi_register_descriptor *rdesc)
{
	const struct rmi_register_desc_item *item;
	int i;
	size_t size = 0;

	for (i = 0; i < rdesc->num_registers; i++) {
		item = &rdesc->registers[i];
		size += item->reg_size;
	}
	return size;
}
EXPORT_SYMBOL_GPL(rmi_register_desc_calc_size);

/* Compute the register offset relative to the base address */
int rmi_register_desc_calc_reg_offset(
		struct rmi_register_descriptor *rdesc, u16 reg)
{
	const struct rmi_register_desc_item *item;
	int offset = 0;
	int i;

	for (i = 0; i < rdesc->num_registers; i++) {
		item = &rdesc->registers[i];
		if (item->reg == reg)
			return offset;
		++offset;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(rmi_register_desc_calc_reg_offset);

bool rmi_register_desc_has_subpacket(const struct rmi_register_desc_item *item,
	u8 subpacket)
{
	return find_next_bit(item->subpacket_map, RMI_REG_DESC_PRESENSE_BITS,
				subpacket) == subpacket;
}

/* Indicates that flash programming is enabled (bootloader mode). */
#define RMI_F01_STATUS_BOOTLOADER(status)	(!!((status) & 0x40))

/*
 * Given the PDT entry for F01, read the device status register to determine
 * if we're stuck in bootloader mode or not.
 *
 */
static int rmi_check_bootloader_mode(struct rmi_device *rmi_dev,
				     const struct pdt_entry *pdt)
{
	int error;
	u8 device_status;

	error = rmi_read(rmi_dev, pdt->data_base_addr + pdt->page_start,
			 &device_status);
	if (error) {
		dev_err(&rmi_dev->dev,
			"Failed to read device status: %d.\n", error);
		return error;
	}

	return RMI_F01_STATUS_BOOTLOADER(device_status);
}

static int rmi_count_irqs(struct rmi_device *rmi_dev,
			 void *ctx, const struct pdt_entry *pdt)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	int *irq_count = ctx;

	*irq_count += pdt->interrupt_source_count;
	if (pdt->function_number == 0x01) {
		data->f01_bootloader_mode =
			rmi_check_bootloader_mode(rmi_dev, pdt);
		if (data->f01_bootloader_mode)
			dev_warn(&rmi_dev->dev,
				"WARNING: RMI4 device is in bootloader mode!\n");
	}

	return RMI_SCAN_CONTINUE;
}

static int rmi_initial_reset(struct rmi_device *rmi_dev,
			     void *ctx, const struct pdt_entry *pdt)
{
	int error;

	if (pdt->function_number == 0x01) {
		u16 cmd_addr = pdt->page_start + pdt->command_base_addr;
		u8 cmd_buf = RMI_DEVICE_RESET_CMD;
		const struct rmi_device_platform_data *pdata =
				rmi_get_platform_data(rmi_dev);

		if (rmi_dev->xport->ops->reset) {
			if (rmi_dev->xport->ops->reset(rmi_dev->xport,
						       cmd_addr))
				return error;

			return RMI_SCAN_DONE;
		}

		error = rmi_write_block(rmi_dev, cmd_addr, &cmd_buf, 1);
		if (error) {
			dev_err(&rmi_dev->dev,
				"Initial reset failed. Code = %d.\n", error);
			return error;
		}

		mdelay(pdata->reset_delay_ms ?: DEFAULT_RESET_DELAY_MS);

		return RMI_SCAN_DONE;
	}

	/* F01 should always be on page 0. If we don't find it there, fail. */
	return pdt->page_start == 0 ? RMI_SCAN_CONTINUE : -ENODEV;
}

static int rmi_create_function(struct rmi_device *rmi_dev,
			       void *ctx, const struct pdt_entry *pdt)
{
	struct device *dev = &rmi_dev->dev;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	int *current_irq_count = ctx;
	struct rmi_function *fn;
	int i;
	int error;

	dev_dbg(dev, "Initializing F%02X.\n", pdt->function_number);

	fn = kzalloc(sizeof(struct rmi_function) +
			BITS_TO_LONGS(data->irq_count) * sizeof(unsigned long),
		     GFP_KERNEL);
	if (!fn) {
		dev_err(dev, "Failed to allocate memory for F%02X\n",
			pdt->function_number);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&fn->node);
	rmi_driver_copy_pdt_to_fd(pdt, &fn->fd);

	fn->rmi_dev = rmi_dev;

	fn->num_of_irqs = pdt->interrupt_source_count;
	fn->irq_pos = *current_irq_count;
	*current_irq_count += fn->num_of_irqs;

	for (i = 0; i < fn->num_of_irqs; i++)
		set_bit(fn->irq_pos + i, fn->irq_mask);

	error = rmi_register_function(fn);
	if (error)
		goto err_put_fn;

	if (pdt->function_number == 0x01)
		data->f01_container = fn;

	list_add_tail(&fn->node, &data->function_list);

	return RMI_SCAN_CONTINUE;

err_put_fn:
	put_device(&fn->dev);
	return error;
}

#ifdef CONFIG_PM_SLEEP
static int rmi_driver_suspend(struct device *dev)
{
	struct rmi_driver_data *data;
	int retval = 0;
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	data = dev_get_drvdata(&rmi_dev->dev);

	mutex_lock(&data->suspend_mutex);

	if (data->pre_suspend) {
		retval = data->pre_suspend(data->pm_data);
		if (retval)
			goto exit;
	}

	disable_sensor(rmi_dev);

	if (data->post_suspend)
		retval = data->post_suspend(data->pm_data);

	regulator_bulk_disable(ARRAY_SIZE(data->supplies), data->supplies);

exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

static int rmi_driver_resume(struct device *dev)
{
	struct rmi_driver_data *data;
	int retval = 0;
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	data = dev_get_drvdata(&rmi_dev->dev);
	mutex_lock(&data->suspend_mutex);

	retval = regulator_bulk_enable(ARRAY_SIZE(data->supplies),
				       data->supplies);
	if (retval)
		goto exit;

	if (data->pre_resume) {
		retval = data->pre_resume(data->pm_data);
		if (retval)
			goto exit;
	}

	retval = enable_sensor(rmi_dev);
	if (retval)
		goto exit;


	if (data->post_resume) {
		retval = data->post_resume(data->pm_data);
		if (retval)
			goto exit;
	}

	data->suspended = false;
exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(rmi_driver_pm, rmi_driver_suspend, rmi_driver_resume);

static int rmi_driver_remove(struct device *dev)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	disable_sensor(rmi_dev);
	regulator_bulk_disable(ARRAY_SIZE(data->supplies), data->supplies);
	rmi_free_function_list(rmi_dev);

	kfree(data->irq_status);
	kfree(data);

	return 0;
}

#ifdef CONFIG_OF
static int rmi_driver_of_probe(struct device *dev,
				struct rmi_device_platform_data *pdata)
{
	int retval;

	retval = rmi_of_property_read_u32(dev, &pdata->poll_interval_ms,
					"syna,poll-interval-ms", 1);
	if (retval)
		return retval;

	retval = rmi_of_property_read_u32(dev, &pdata->reset_delay_ms,
					"syna,reset-delay-ms", 1);
	if (retval)
		return retval;

	pdata->unified_input = of_property_read_bool(dev->of_node,
					"syna,unified_input");

	return 0;
}
#else
static inline int rmi_driver_of_probe(struct device *dev,
					struct rmi_device_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int rmi_driver_probe(struct device *dev)
{
	struct rmi_driver *rmi_driver;
	struct rmi_driver_data *data;
	struct rmi_device_platform_data *pdata;
	struct rmi_device *rmi_dev;
	size_t size;
	void *irq_memory;
	int irq_count;
	int retval;

	dev_dbg(dev, "%s: Starting probe.\n", __func__);

	if (!rmi_is_physical_device(dev)) {
		dev_dbg(dev, "Not a physical device.\n");
		return -ENODEV;
	}

	rmi_dev = to_rmi_device(dev);
	rmi_driver = to_rmi_driver(dev->driver);
	rmi_dev->driver = rmi_driver;

	pdata = rmi_get_platform_data(rmi_dev);

	if (rmi_dev->xport->dev->of_node) {
		retval = rmi_driver_of_probe(rmi_dev->xport->dev, pdata);
		if (retval)
			return retval;
	}

	data = kzalloc(sizeof(struct rmi_driver_data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s: Failed to allocate driver data.\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&data->function_list);
	data->rmi_dev = rmi_dev;
	dev_set_drvdata(&rmi_dev->dev, data);

	data->supplies[0].supply = "vdd";
	data->supplies[0].optional = true;
	data->supplies[1].supply = "vio";
	data->supplies[1].optional = true;
	retval = devm_regulator_bulk_get(rmi_dev->xport->dev,
					 ARRAY_SIZE(data->supplies),
					 data->supplies);
	if (retval < 0)
		return retval;

	retval = regulator_bulk_enable(ARRAY_SIZE(data->supplies),
				       data->supplies);
	if (retval < 0)
		return retval;

	/*
	 * Right before a warm boot, the sensor might be in some unusual state,
	 * such as F54 diagnostics, or F34 bootloader mode after a firmware
	 * or configuration update.  In order to clear the sensor to a known
	 * state and/or apply any updates, we issue a initial reset to clear any
	 * previous settings and force it into normal operation.
	 *
	 * We have to do this before actually building the PDT because
	 * the reflash updates (if any) might cause various registers to move
	 * around.
	 *
	 * For a number of reasons, this initial reset may fail to return
	 * within the specified time, but we'll still be able to bring up the
	 * driver normally after that failure.  This occurs most commonly in
	 * a cold boot situation (where then firmware takes longer to come up
	 * than from a warm boot) and the reset_delay_ms in the platform data
	 * has been set too short to accomodate that.  Since the sensor will
	 * eventually come up and be usable, we don't want to just fail here
	 * and leave the customer's device unusable.  So we warn them, and
	 * continue processing.
	 */
	retval = rmi_scan_pdt(rmi_dev, NULL, rmi_initial_reset);
	if (retval < 0)
		dev_warn(dev, "RMI initial reset failed! Continuing in spite of this.\n");

	retval = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION, &data->pdt_props);
	if (retval < 0) {
		/*
		 * we'll print out a warning and continue since
		 * failure to get the PDT properties is not a cause to fail
		 */
		dev_warn(dev, "Could not read PDT properties from %#06x (code %d). Assuming 0x00.\n",
			 PDT_PROPERTIES_LOCATION, retval);
	}

	/*
	 * We need to count the IRQs and allocate their storage before scanning
	 * the PDT and creating the function entries, because adding a new
	 * function can trigger events that result in the IRQ related storage
	 * being accessed.
	 */
	dev_dbg(dev, "Counting IRQs.\n");
	irq_count = 0;
	retval = rmi_scan_pdt(rmi_dev, &irq_count, rmi_count_irqs);
	if (retval < 0) {
		dev_err(dev, "IRQ counting failed with code %d.\n", retval);
		goto err_free_mem;
	}
	data->irq_count = irq_count;
	data->num_of_irq_regs = (data->irq_count + 7) / 8;

	mutex_init(&data->irq_mutex);

	size = BITS_TO_LONGS(data->irq_count) * sizeof(unsigned long);
	irq_memory = kzalloc(size * 4, GFP_KERNEL);
	if (!irq_memory) {
		dev_err(dev, "Failed to allocate memory for irq masks.\n");
		goto err_free_mem;
	}

	data->irq_status	= irq_memory + size * 0;
	data->fn_irq_bits	= irq_memory + size * 1;
	data->current_irq_mask	= irq_memory + size * 2;
	data->new_irq_mask	= irq_memory + size * 3;

	if (rmi_dev->xport->input) {
		/*
		 * The transport driver already has an input device.
		 * In some cases it is preferable to reuse the transport
		 * devices input device instead of creating a new one here.
		 * One example is some HID touchpads report "pass-through"
		 * button events not handled by the rmi driver.
		 */
		data->input = rmi_dev->xport->input;
		pdata->unified_input = true;
	} else if (pdata->unified_input) {
		data->input = devm_input_allocate_device(dev);
		rmi_driver_set_input_params(rmi_dev, data->input);
		data->input->phys = devm_kasprintf(dev, GFP_KERNEL,
						"%s/input0", dev_name(dev));
	}

	irq_count = 0;
	dev_dbg(dev, "Creating functions.");
	retval = rmi_scan_pdt(rmi_dev, &irq_count, rmi_create_function);
	if (retval < 0) {
		dev_err(dev, "Function creation failed with code %d.\n",
			retval);
		goto err_destroy_functions;
	}

	if (!data->f01_container) {
		dev_err(dev, "Missing F01 container!\n");
		retval = -EINVAL;
		goto err_destroy_functions;
	}

	retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr + 1,
				data->current_irq_mask, data->num_of_irq_regs);
	if (retval < 0) {
		dev_err(dev, "%s: Failed to read current IRQ mask.\n",
			__func__);
		goto err_destroy_functions;
	}

#ifdef CONFIG_PM_SLEEP
	data->pm_data = pdata->pm_data;
	data->pre_suspend = pdata->pre_suspend;
	data->post_suspend = pdata->post_suspend;
	data->pre_resume = pdata->pre_resume;
	data->post_resume = pdata->post_resume;

	mutex_init(&data->suspend_mutex);
#endif

	if (data->input) {
		rmi_driver_set_input_name(rmi_dev, data->input);
		if (!rmi_dev->xport->input) {
			if (input_register_device(data->input)) {
				dev_err(dev, "%s: Failed to register input device.\n",
					__func__);
				goto err_destroy_functions;
			}
		}
	}

	if (rmi_dev->xport->irq > 0) {
		if (!rmi_dev->xport->hard_irq)
			rmi_dev->xport->irq_flags |= IRQF_ONESHOT;
	} else if (data->polling) {
		data->poll_interval = ktime_set(0,
			(pdata->poll_interval_ms ? pdata->poll_interval_ms :
			DEFAULT_POLL_INTERVAL_MS) * 1000 * 1000);
		data->polling = true;
	}

	if (data->f01_container->dev.driver) {
		/* Driver already bound, so enable ATTN now. */
		return enable_sensor(rmi_dev);
	}

	return 0;

err_destroy_functions:
	rmi_free_function_list(rmi_dev);
	kfree(irq_memory);
err_free_mem:
	kfree(data);
	return retval < 0 ? retval : 0;
}

static struct rmi_driver rmi_physical_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_physical",
		.bus	= &rmi_bus_type,
		.pm	= &rmi_driver_pm,
		.probe = rmi_driver_probe,
		.remove = rmi_driver_remove,
	},
	.irq_handler = rmi_driver_irq_handler,
	.reset_handler = rmi_driver_reset_handler,
	.clear_irq_bits = rmi_driver_clear_irq_bits,
	.set_irq_bits = rmi_driver_set_irq_bits,
	.set_input_params = rmi_driver_set_input_params,
};

bool rmi_is_physical_driver(struct device_driver *drv)
{
	return drv == &rmi_physical_driver.driver;
}

int __init rmi_register_physical_driver(void)
{
	int error;

	error = driver_register(&rmi_physical_driver.driver);
	if (error) {
		pr_err("%s: driver register failed, code=%d.\n", __func__,
		       error);
		return error;
	}

	return 0;
}

void __exit rmi_unregister_physical_driver(void)
{
	driver_unregister(&rmi_physical_driver.driver);
}
