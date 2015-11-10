/*
 *  Copyright (c) 2013 Andrew Duggan <aduggan@synaptics.com>
 *  Copyright (c) 2013 Synaptics Incorporated
 *  Copyright (c) 2014 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 *  Copyright (c) 2014 Red Hat, Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/rmi.h>
#include "hid-ids.h"

#define RMI_MOUSE_REPORT_ID		0x01 /* Mouse emulation Report */
#define RMI_WRITE_REPORT_ID		0x09 /* Output Report */
#define RMI_READ_ADDR_REPORT_ID		0x0a /* Output Report */
#define RMI_READ_DATA_REPORT_ID		0x0b /* Input Report */
#define RMI_ATTN_REPORT_ID		0x0c /* Input Report */
#define RMI_SET_RMI_MODE_REPORT_ID	0x0f /* Feature Report */

/* flags */
#define RMI_READ_REQUEST_PENDING	0
#define RMI_READ_DATA_PENDING		1
#define RMI_STARTED			2

#define RMI_SLEEP_NORMAL		0x0
#define RMI_SLEEP_DEEP_SLEEP		0x1

/* device flags */
#define RMI_DEVICE			BIT(0)
#define RMI_DEVICE_HAS_PHYS_BUTTONS	BIT(1)

/*
 * retrieve the ctrl registers
 * the ctrl register has a size of 20 but a fw bug split it into 16 + 4,
 * and there is no way to know if the first 20 bytes are here or not.
 * We use only the first 12 bytes, so get only them.
 */
#define RMI_F11_CTRL_REG_COUNT		12

enum rmi_mode_type {
	RMI_MODE_OFF			= 0,
	RMI_MODE_ATTN_REPORTS		= 1,
	RMI_MODE_NO_PACKED_ATTN_REPORTS	= 2,
};

/**
 * struct rmi_data - stores information for hid communication
 *
 * @page_mutex: Locks current page to avoid changing pages in unexpected ways.
 * @page: Keeps track of the current virtual page
 *
 * @wait: Used for waiting for read data
 *
 * @writeReport: output buffer when writing RMI registers
 * @readReport: input buffer when reading RMI registers
 *
 * @input_report_size: size of an input report (advertised by HID)
 * @output_report_size: size of an output report (advertised by HID)
 *
 * @flags: flags for the current device (started, reading, etc...)
 *
 * @f11: placeholder of internal RMI function F11 description
 * @f30: placeholder of internal RMI function F30 description
 *
 * @max_fingers: maximum finger count reported by the device
 * @max_x: maximum x value reported by the device
 * @max_y: maximum y value reported by the device
 *
 * @gpio_led_count: count of GPIOs + LEDs reported by F30
 * @button_count: actual physical buttons count
 * @button_mask: button mask used to decode GPIO ATTN reports
 * @button_state_mask: pull state of the buttons
 *
 * @input: pointer to the kernel input device
 *
 * @reset_work: worker which will be called in case of a mouse report
 * @hdev: pointer to the struct hid_device
 */
struct rmi_data {
	struct mutex page_mutex;
	int page;
	struct rmi_transport_dev xport;

	wait_queue_head_t wait;

	u8 *writeReport;
	u8 *readReport;

	int input_report_size;
	int output_report_size;

	unsigned long flags;

	struct work_struct reset_work;
	struct hid_device *hdev;

	unsigned long device_flags;
};

#define RMI_PAGE(addr) (((addr) >> 8) & 0xff)

static int rmi_write_report(struct hid_device *hdev, u8 *report, int len);

/**
 * rmi_set_page - Set RMI page
 * @hdev: The pointer to the hid_device struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct hid_device *hdev, u8 page)
{
	struct rmi_data *data = hid_get_drvdata(hdev);
	int retval;

	data->writeReport[0] = RMI_WRITE_REPORT_ID;
	data->writeReport[1] = 1;
	data->writeReport[2] = 0xFF;
	data->writeReport[4] = page;

	retval = rmi_write_report(hdev, data->writeReport,
			data->output_report_size);
	if (retval != data->output_report_size) {
		dev_err(&hdev->dev,
			"%s: set page failed: %d.", __func__, retval);
		return retval;
	}

	data->page = page;
	return 0;
}

static int rmi_set_mode(struct hid_device *hdev, u8 mode)
{
	int ret;
	u8 txbuf[2] = {RMI_SET_RMI_MODE_REPORT_ID, mode};

	ret = hid_hw_raw_request(hdev, RMI_SET_RMI_MODE_REPORT_ID, txbuf,
			sizeof(txbuf), HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0) {
		dev_err(&hdev->dev, "unable to set rmi mode to %d (%d)\n", mode,
			ret);
		return ret;
	}

	return 0;
}

static int rmi_write_report(struct hid_device *hdev, u8 *report, int len)
{
	int ret;

	ret = hid_hw_output_report(hdev, (void *)report, len);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to write hid report (%d)\n", ret);
		return ret;
	}

	return ret;
}

static int rmi_hid_read_block(struct rmi_transport_dev *xport, u16 addr,
		void *buf, size_t len)
{
	struct rmi_data *data = container_of(xport, struct rmi_data, xport);
	struct hid_device *hdev = data->hdev;
	int ret;
	int bytes_read;
	int bytes_needed;
	int retries;
	int read_input_count;

	mutex_lock(&data->page_mutex);

	if (RMI_PAGE(addr) != data->page) {
		ret = rmi_set_page(hdev, RMI_PAGE(addr));
		if (ret < 0)
			goto exit;
	}

	for (retries = 5; retries > 0; retries--) {
		data->writeReport[0] = RMI_READ_ADDR_REPORT_ID;
		data->writeReport[1] = 0; /* old 1 byte read count */
		data->writeReport[2] = addr & 0xFF;
		data->writeReport[3] = (addr >> 8) & 0xFF;
		data->writeReport[4] = len  & 0xFF;
		data->writeReport[5] = (len >> 8) & 0xFF;

		set_bit(RMI_READ_REQUEST_PENDING, &data->flags);

		ret = rmi_write_report(hdev, data->writeReport,
						data->output_report_size);
		if (ret != data->output_report_size) {
			clear_bit(RMI_READ_REQUEST_PENDING, &data->flags);
			dev_err(&hdev->dev,
				"failed to write request output report (%d)\n",
				ret);
			goto exit;
		}

		bytes_read = 0;
		bytes_needed = len;
		while (bytes_read < len) {
			if (!wait_event_timeout(data->wait,
				test_bit(RMI_READ_DATA_PENDING, &data->flags),
					msecs_to_jiffies(1000))) {
				hid_warn(hdev, "%s: timeout elapsed\n",
					 __func__);
				ret = -EAGAIN;
				break;
			}

			read_input_count = data->readReport[1];
			memcpy(buf + bytes_read, &data->readReport[2],
				read_input_count < bytes_needed ?
					read_input_count : bytes_needed);

			bytes_read += read_input_count;
			bytes_needed -= read_input_count;
			clear_bit(RMI_READ_DATA_PENDING, &data->flags);
		}

		if (ret >= 0) {
			ret = 0;
			break;
		}
	}

exit:
	clear_bit(RMI_READ_REQUEST_PENDING, &data->flags);
	mutex_unlock(&data->page_mutex);
	return ret;
}

static int rmi_hid_write_block(struct rmi_transport_dev *xport, u16 addr,
		const void *buf, size_t len)
{
	struct rmi_data *data = container_of(xport, struct rmi_data, xport);
	struct hid_device *hdev = data->hdev;
	int ret;

	mutex_lock(&data->page_mutex);

	if (RMI_PAGE(addr) != data->page) {
		ret = rmi_set_page(hdev, RMI_PAGE(addr));
		if (ret < 0)
			goto exit;
	}

	data->writeReport[0] = RMI_WRITE_REPORT_ID;
	data->writeReport[1] = len;
	data->writeReport[2] = addr & 0xFF;
	data->writeReport[3] = (addr >> 8) & 0xFF;
	memcpy(&data->writeReport[4], buf, len);

	ret = rmi_write_report(hdev, data->writeReport,
					data->output_report_size);
	if (ret < 0) {
		dev_err(&hdev->dev,
			"failed to write request output report (%d)\n",
			ret);
		goto exit;
	}
	ret = 0;

exit:
	mutex_unlock(&data->page_mutex);
	return ret;
}

static void rmi_reset_work(struct work_struct *work)
{
	struct rmi_data *hdata = container_of(work, struct rmi_data,
						reset_work);

	/* switch the device to RMI if we receive a generic mouse report */
	rmi_set_mode(hdata->hdev, RMI_MODE_ATTN_REPORTS);
}

static inline int rmi_schedule_reset(struct hid_device *hdev)
{
	struct rmi_data *hdata = hid_get_drvdata(hdev);
	return schedule_work(&hdata->reset_work);
}

static int rmi_input_event(struct hid_device *hdev, u8 *data, int size)
{
	struct rmi_data *hdata = hid_get_drvdata(hdev);
	struct rmi_device *rmi_dev = hdata->xport.rmi_dev;
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);

	if (!(test_bit(RMI_STARTED, &hdata->flags)))
		return 0;

	*(drvdata->irq_status) = data[1];
	hdata->xport.attn_data = &data[2];
	hdata->xport.attn_size = hdata->input_report_size - 2;

	rmi_process_interrupt_requests(rmi_dev);

	return 1;
}

static int rmi_read_data_event(struct hid_device *hdev, u8 *data, int size)
{
	struct rmi_data *hdata = hid_get_drvdata(hdev);

	if (!test_bit(RMI_READ_REQUEST_PENDING, &hdata->flags)) {
		hid_dbg(hdev, "no read request pending\n");
		return 0;
	}

	memcpy(hdata->readReport, data, size < hdata->input_report_size ?
			size : hdata->input_report_size);
	set_bit(RMI_READ_DATA_PENDING, &hdata->flags);
	wake_up(&hdata->wait);

	return 1;
}

static int rmi_check_sanity(struct hid_device *hdev, u8 *data, int size)
{
	int valid_size = size;
	/*
	 * On the Dell XPS 13 9333, the bus sometimes get confused and fills
	 * the report with a sentinel value "ff". Synaptics told us that such
	 * behavior does not comes from the touchpad itself, so we filter out
	 * such reports here.
	 */

	while ((data[valid_size - 1] == 0xff) && valid_size > 0)
		valid_size--;

	return valid_size;
}

static int rmi_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	size = rmi_check_sanity(hdev, data, size);
	if (size < 2)
		return 0;

	switch (data[0]) {
	case RMI_READ_DATA_REPORT_ID:
		return rmi_read_data_event(hdev, data, size);
	case RMI_ATTN_REPORT_ID:
		return rmi_input_event(hdev, data, size);
	default:
		return 1;
	}

	return 0;
}

static int rmi_event(struct hid_device *hdev, struct hid_field *field,
			struct hid_usage *usage, __s32 value)
{
	struct rmi_data *data = hid_get_drvdata(hdev);

	if ((data->device_flags & RMI_DEVICE) &&
	    (field->application == HID_GD_POINTER ||
	    field->application == HID_GD_MOUSE)) {
		if (data->device_flags & RMI_DEVICE_HAS_PHYS_BUTTONS) {
			if ((usage->hid & HID_USAGE_PAGE) == HID_UP_BUTTON)
				return 0;

			if ((usage->hid == HID_GD_X || usage->hid == HID_GD_Y)
			    && !value)
				return 1;
		}

		rmi_schedule_reset(hdev);
		return 1;
	}

	return 0;
}

#ifdef CONFIG_PM
static int rmi_post_reset(struct hid_device *hdev)
{
	return rmi_set_mode(hdev, RMI_MODE_ATTN_REPORTS);
}

static int rmi_post_resume(struct hid_device *hdev)
{
	return rmi_set_mode(hdev, RMI_MODE_ATTN_REPORTS);
}
#endif /* CONFIG_PM */

static void rmi_input_configured(struct hid_device *hdev, struct hid_input *hi)
{
	struct rmi_data *data = hid_get_drvdata(hdev);
	struct input_dev *input = hi->input;
	int ret;

	if (data->device_flags & RMI_DEVICE) {
		data->xport.input = input;

		hid_dbg(hdev, "Opening low level driver\n");
		ret = hid_hw_open(hdev);
		if (ret)
			return;

		/* Allow incoming hid reports */
		hid_device_io_start(hdev);

		ret = rmi_set_mode(hdev, RMI_MODE_ATTN_REPORTS);
		if (ret < 0) {
			dev_err(&hdev->dev, "failed to set rmi mode\n");
			goto exit;
		}

		ret = rmi_set_page(hdev, 0);
		if (ret < 0) {
			dev_err(&hdev->dev, "failed to set page select to 0.\n");
			goto exit;
		}

		ret = rmi_register_transport_device(&data->xport);
		if (ret < 0) {
			dev_err(&hdev->dev, "failed to register transport driver\n");
			goto exit;
		}

		set_bit(RMI_STARTED, &data->flags);
	}

exit:
	hid_device_io_stop(hdev);
	hid_hw_close(hdev);
	return ret;
}

static int rmi_input_mapping(struct hid_device *hdev,
		struct hid_input *hi, struct hid_field *field,
		struct hid_usage *usage, unsigned long **bit, int *max)
{
	struct rmi_data *data = hid_get_drvdata(hdev);

	/*
	 * we want to make HID ignore the advertised HID collection
	 * for RMI deivces
	 */
	if (data->device_flags & RMI_DEVICE) {
		if ((data->device_flags & RMI_DEVICE_HAS_PHYS_BUTTONS) &&
		    ((usage->hid & HID_USAGE_PAGE) == HID_UP_BUTTON))
			return 0;

		return -1;
	}

	return 0;
}

static int rmi_check_valid_report_id(struct hid_device *hdev, unsigned type,
		unsigned id, struct hid_report **report)
{
	int i;

	*report = hdev->report_enum[type].report_id_hash[id];
	if (*report) {
		for (i = 0; i < (*report)->maxfield; i++) {
			unsigned app = (*report)->field[i]->application;
			if ((app & HID_USAGE_PAGE) >= HID_UP_MSVENDOR)
				return 1;
		}
	}

	return 0;
}

struct rmi_2d_sensor_platform_data rmi_hid_2d_sensor_data = {
	.sensor_type = rmi_sensor_touchpad,
	.axis_align.flip_y = true,
	.kernel_tracking = true,
};

static struct rmi_f30_data rmi_hid_f30_data = {
};

static struct rmi_device_platform_data rmi_hid_pdata = {
	.sensor_pdata = &rmi_hid_2d_sensor_data,
	.f30_data = &rmi_hid_f30_data,
	.unified_input = true,
};

static const struct rmi_transport_ops hid_rmi_ops = {
	.write_block	= rmi_hid_write_block,
	.read_block	= rmi_hid_read_block,
};

static int rmi_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct rmi_data *data = NULL;
	int ret;
	size_t alloc_size;
	struct hid_report *input_report;
	struct hid_report *output_report;
	struct hid_report *feature_report;

	data = devm_kzalloc(&hdev->dev, sizeof(struct rmi_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	INIT_WORK(&data->reset_work, rmi_reset_work);
	data->hdev = hdev;

	hid_set_drvdata(hdev, data);

	hdev->quirks |= HID_QUIRK_NO_INIT_REPORTS;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}

	if (id->driver_data)
		data->device_flags = id->driver_data;

	/*
	 * Check for the RMI specific report ids. If they are misisng
	 * simply return and let the events be processed by hid-input
	 */
	if (!rmi_check_valid_report_id(hdev, HID_FEATURE_REPORT,
	    RMI_SET_RMI_MODE_REPORT_ID, &feature_report)) {
		hid_dbg(hdev, "device does not have set mode feature report\n");
		goto start;
	}

	if (!rmi_check_valid_report_id(hdev, HID_INPUT_REPORT,
	    RMI_ATTN_REPORT_ID, &input_report)) {
		hid_dbg(hdev, "device does not have attention input report\n");
		goto start;
	}

	data->input_report_size = hid_report_len(input_report);

	if (!rmi_check_valid_report_id(hdev, HID_OUTPUT_REPORT,
	    RMI_WRITE_REPORT_ID, &output_report)) {
		hid_dbg(hdev,
			"device does not have rmi write output report\n");
		goto start;
	}

	data->output_report_size = hid_report_len(output_report);

	data->device_flags |= RMI_DEVICE;
	alloc_size = data->output_report_size + data->input_report_size;

	data->writeReport = devm_kzalloc(&hdev->dev, alloc_size, GFP_KERNEL);
	if (!data->writeReport) {
		ret = -ENOMEM;
		return ret;
	}

	data->readReport = data->writeReport + data->output_report_size;

	init_waitqueue_head(&data->wait);

	mutex_init(&data->page_mutex);

	data->xport.dev = &hdev->dev;
	data->xport.pdata = rmi_hid_pdata;
	data->xport.proto_name = "hid";
	data->xport.ops = &hid_rmi_ops;

start:
	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	return 0;
}

static void rmi_remove(struct hid_device *hdev)
{
	struct rmi_data *hdata = hid_get_drvdata(hdev);

	clear_bit(RMI_STARTED, &hdata->flags);
	rmi_unregister_transport_device(&hdata->xport);

	hid_hw_stop(hdev);
}

static const struct hid_device_id rmi_id[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_RAZER, USB_DEVICE_ID_RAZER_BLADE_14),
		.driver_data = RMI_DEVICE_HAS_PHYS_BUTTONS },
	{ HID_DEVICE(HID_BUS_ANY, HID_GROUP_RMI, HID_ANY_ID, HID_ANY_ID) },
	{ }
};
MODULE_DEVICE_TABLE(hid, rmi_id);

static struct hid_driver rmi_driver = {
	.name = "hid-rmi",
	.id_table		= rmi_id,
	.probe			= rmi_probe,
	.remove			= rmi_remove,
	.event			= rmi_event,
	.raw_event		= rmi_raw_event,
	.input_mapping		= rmi_input_mapping,
	.input_configured	= rmi_input_configured,
#ifdef CONFIG_PM
	.resume			= rmi_post_resume,
	.reset_resume		= rmi_post_reset,
#endif
};

module_hid_driver(rmi_driver);

MODULE_AUTHOR("Andrew Duggan <aduggan@synaptics.com>");
MODULE_DESCRIPTION("RMI HID driver");
MODULE_LICENSE("GPL");
