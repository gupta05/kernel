/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _LINUX_SPMI_H
#define _LINUX_SPMI_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>

/* Maximum slave identifier */
#define SPMI_MAX_SLAVE_ID		16

/* SPMI Commands */
#define SPMI_CMD_EXT_WRITE		0x00
#define SPMI_CMD_RESET			0x10
#define SPMI_CMD_SLEEP			0x11
#define SPMI_CMD_SHUTDOWN		0x12
#define SPMI_CMD_WAKEUP			0x13
#define SPMI_CMD_AUTHENTICATE		0x14
#define SPMI_CMD_MSTR_READ		0x15
#define SPMI_CMD_MSTR_WRITE		0x16
#define SPMI_CMD_TRANSFER_BUS_OWNERSHIP	0x1A
#define SPMI_CMD_DDB_MASTER_READ	0x1B
#define SPMI_CMD_DDB_SLAVE_READ		0x1C
#define SPMI_CMD_EXT_READ		0x20
#define SPMI_CMD_EXT_WRITEL		0x30
#define SPMI_CMD_EXT_READL		0x38
#define SPMI_CMD_WRITE			0x40
#define SPMI_CMD_READ			0x60
#define SPMI_CMD_ZERO_WRITE		0x80

/**
 * Client/device handle (struct spmi_device):
 *  This is the client/device handle returned when a SPMI device
 *  is registered with a controller.
 *  Pointer to this structure is used by client-driver as a handle.
 *  @dev: Driver model representation of the device.
 *  @ctrl: SPMI controller managing the bus hosting this device.
 *  @usid: Unique Slave IDentifier.
 */
struct spmi_device {
	struct device		dev;
	struct spmi_controller	*ctrl;
	u8			usid;
};
#define to_spmi_device(d) container_of(d, struct spmi_device, dev)

static inline void *spmi_device_get_drvdata(const struct spmi_device *sdev)
{
	return dev_get_drvdata(&sdev->dev);
}

static inline void spmi_device_set_drvdata(struct spmi_device *sdev, void *data)
{
	dev_set_drvdata(&sdev->dev, data);
}

/**
 * spmi_controller_alloc: Allocate a new SPMI controller
 * @ctrl: associated controller
 *
 * Caller is responsible for either calling spmi_device_add() to add the
 * newly allocated controller, or calling spmi_device_put() to discard it.
 */
struct spmi_device *spmi_device_alloc(struct spmi_controller *ctrl);

static inline void spmi_device_put(struct spmi_device *sdev)
{
	if (sdev)
		put_device(&sdev->dev);
}

/**
 * spmi_device_add: add a device previously constructed via spmi_device_alloc()
 * @sdev: spmi_device to be added
 */
int spmi_device_add(struct spmi_device *sdev);

/**
 * spmi_device_remove: remove a device
 * @sdev: spmi_device to be removed
 */
void spmi_device_remove(struct spmi_device *sdev);

/**
 * struct spmi_controller: interface to the SPMI master controller
 * @dev: Driver model representation of the device.
 * @nr: board-specific number identifier for this controller/bus
 * @cmd: sends a non-data command sequence on the SPMI bus.
 * @read_cmd: sends a register read command sequence on the SPMI bus.
 * @write_cmd: sends a register write command sequence on the SPMI bus.
 */
struct spmi_controller {
	struct device		dev;
	unsigned int		nr;
	int	(*cmd)(struct spmi_controller *ctrl, u8 opcode, u8 sid);
	int	(*read_cmd)(struct spmi_controller *ctrl, u8 opcode,
			    u8 sid, u16 addr, u8 bc, u8 *buf);
	int	(*write_cmd)(struct spmi_controller *ctrl, u8 opcode,
			     u8 sid, u16 addr, u8 bc, const u8 *buf);
};
#define to_spmi_controller(d) container_of(d, struct spmi_controller, dev)

static inline
void *spmi_controller_get_drvdata(const struct spmi_controller *ctrl)
{
	return dev_get_drvdata(&ctrl->dev);
}

static inline void spmi_controller_set_drvdata(struct spmi_controller *ctrl,
					       void *data)
{
	dev_set_drvdata(&ctrl->dev, data);
}

/**
 * spmi_controller_alloc: Allocate a new SPMI controller
 * @parent: parent device
 * @size: size of private data
 *
 * Caller is responsible for either calling spmi_controller_add() to add the
 * newly allocated controller, or calling spmi_controller_put() to discard it.
 */
struct spmi_controller *spmi_controller_alloc(struct device *parent,
					      size_t size);

static inline void spmi_controller_put(struct spmi_controller *ctrl)
{
	if (ctrl)
		put_device(&ctrl->dev);
}

/**
 * spmi_controller_add: Controller bring-up.
 * @ctrl: controller to be registered.
 *
 * Register a controller previously allocated via spmi_controller_alloc() with
 * the SPMI core
 */
int spmi_controller_add(struct spmi_controller *ctrl);

/**
 * spmi_controller_remove: Controller tear-down.
 *
 * Remove a SPMI controller.
 */
int spmi_controller_remove(struct spmi_controller *ctrl);

/**
 * struct spmi_driver: Manage SPMI generic/slave device driver
 * @driver: SPMI device drivers should initialize name and owner field of
 *	    this structure
 * @probe: binds this driver to a SPMI device.
 * @remove: unbinds this driver from the SPMI device.
 * @shutdown: standard shutdown callback used during powerdown/halt.
 * @suspend: standard suspend callback used during system suspend
 * @resume: standard resume callback used during system resume
 */
struct spmi_driver {
	struct device_driver driver;
	int	(*probe)(struct spmi_device *sdev);
	int	(*remove)(struct spmi_device *sdev);
	void	(*shutdown)(struct spmi_device *sdev);
	int	(*suspend)(struct spmi_device *sdev, pm_message_t pmesg);
	int	(*resume)(struct spmi_device *sdev);
};
#define to_spmi_driver(d) container_of(d, struct spmi_driver, driver)

/**
 * spmi_driver_register: Client driver registration with SPMI framework.
 * @sdrv: client driver to be associated with client-device.
 *
 * This API will register the client driver with the SPMI framework.
 * It is called from the driver's module-init function.
 */
int spmi_driver_register(struct spmi_driver *sdrv);

/**
 * spmi_driver_unregister - reverse effect of spmi_driver_register
 * @sdrv: the driver to unregister
 * Context: can sleep
 */
static inline void spmi_driver_unregister(struct spmi_driver *sdrv)
{
	if (sdrv)
		driver_unregister(&sdrv->driver);
}

#define module_spmi_driver(__spmi_driver) \
	module_driver(__spmi_driver, spmi_driver_register, \
			spmi_driver_unregister)

/**
 * spmi_register_read() - register read
 * @sdev: SPMI device
 * @addr: slave register address (5-bit address).
 * @buf: buffer to be populated with data from the Slave.
 *
 * Reads 1 byte of data from a Slave device register.
 */
int spmi_register_read(struct spmi_device *sdev, u8 addr, u8 *buf);

/**
 * spmi_ext_register_read() - extended register read
 * @sdev: SPMI device
 * @addr: slave register address (8-bit address).
 * @buf: buffer to be populated with data from the Slave.
 * @len: the request number of bytes to read (up to 16 bytes).
 *
 * Reads up to 16 bytes of data from the extended register space on a
 * Slave device.
 */
int spmi_ext_register_read(struct spmi_device *sdev, u8 addr, u8 *buf,
			   size_t len);

/**
 * spmi_ext_register_readl() - extended register read long
 * @sdev: SPMI device
 * @addr: slave register address (16-bit address).
 * @buf: buffer to be populated with data from the Slave.
 * @len: the request number of bytes to read (up to 8 bytes).
 *
 * Reads up to 8 bytes of data from the extended register space on a
 * Slave device using 16-bit address.
 */
int spmi_ext_register_readl(struct spmi_device *sdev, u16 addr, u8 *buf,
			    size_t len);

/**
 * spmi_register_write() - register write
 * @sdev: SPMI device
 * @addr: slave register address (5-bit address).
 * @buf: buffer containing the data to be transferred to the Slave.
 *
 * Writes 1 byte of data to a Slave device register.
 */
int spmi_register_write(struct spmi_device *sdev, u8 addr, const u8 *buf);

/**
 * spmi_register_zero_write() - register zero write
 * @sdev: SPMI device
 * @data: the data to be written to register 0 (7-bits).
 *
 * Writes data to register 0 of the Slave device.
 */
int spmi_register_zero_write(struct spmi_device *sdev, u8 data);

/**
 * spmi_ext_register_write() - extended register write
 * @sdev: SPMI device
 * @addr: slave register address (8-bit address).
 * @buf: buffer containing the data to be transferred to the Slave.
 * @len: the request number of bytes to read (up to 16 bytes).
 *
 * Writes up to 16 bytes of data to the extended register space of a
 * Slave device.
 */
int spmi_ext_register_write(struct spmi_device *sdev, u8 addr,
			    const u8 *buf, size_t len);

/**
 * spmi_ext_register_writel() - extended register write long
 * @sdev: SPMI device
 * @addr: slave register address (16-bit address).
 * @buf: buffer containing the data to be transferred to the Slave.
 * @len: the request number of bytes to read (up to 8 bytes).
 *
 * Writes up to 8 bytes of data to the extended register space of a
 * Slave device using 16-bit address.
 */
int spmi_ext_register_writel(struct spmi_device *sdev, u16 addr,
			     const u8 *buf, size_t len);

/**
 * spmi_command_reset() - sends RESET command to the specified slave
 * @sdev: SPMI device
 *
 * The Reset command initializes the Slave and forces all registers to
 * their reset values. The Slave shall enter the STARTUP state after
 * receiving a Reset command.
 *
 * Returns
 * -EINVAL for invalid slave identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_reset(struct spmi_device *sdev);

/**
 * spmi_command_sleep() - sends SLEEP command to the specified slave
 * @sdev: SPMI device
 *
 * The Sleep command causes the Slave to enter the user defined SLEEP state.
 *
 * Returns
 * -EINVAL for invalid slave identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_sleep(struct spmi_device *sdev);

/**
 * spmi_command_wakeup() - sends WAKEUP command to the specified slave
 * @sdev: SPMI device
 *
 * The Wakeup command causes the Slave to move from the SLEEP state to
 * the ACTIVE state.
 *
 * Returns
 * -EINVAL for invalid slave identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_wakeup(struct spmi_device *sdev);

/**
 * spmi_command_shutdown() - sends SHUTDOWN command to the specified slave
 * @sdev: SPMI device
 *
 * The Shutdown command causes the Slave to enter the SHUTDOWN state.
 *
 * Returns
 * -EINVAL for invalid slave identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_shutdown(struct spmi_device *sdev);

#endif
