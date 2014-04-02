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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <dt-bindings/spmi/spmi.h>

static DEFINE_MUTEX(ctrl_idr_lock);
static DEFINE_IDR(ctrl_idr);

static void spmi_dev_release(struct device *dev)
{
	struct spmi_device *sdev = to_spmi_device(dev);
	kfree(sdev);
}

static struct device_type spmi_dev_type = {
	.release	= spmi_dev_release,
};

static void spmi_ctrl_release(struct device *dev)
{
	struct spmi_controller *ctrl = to_spmi_controller(dev);
	mutex_lock(&ctrl_idr_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&ctrl_idr_lock);
	kfree(ctrl);
}

static struct device_type spmi_ctrl_type = {
	.release	= spmi_ctrl_release,
};

#ifdef CONFIG_PM_SLEEP
static int spmi_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_suspend(dev);
	else
		return 0;
}

static int spmi_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_resume(dev);
	else
		return 0;
}
#else
#define spmi_pm_suspend		NULL
#define spmi_pm_resume		NULL
#endif

static const struct dev_pm_ops spmi_pm_ops = {
	.suspend	= spmi_pm_suspend,
	.resume		= spmi_pm_resume,
	SET_RUNTIME_PM_OPS(
		pm_generic_suspend,
		pm_generic_resume,
		pm_generic_runtime_idle
	)
};

static int spmi_device_match(struct device *dev, struct device_driver *drv)
{
	if (of_driver_match_device(dev, drv))
		return 1;

	if (drv->name)
		return strncmp(dev_name(dev), drv->name,
			       SPMI_NAME_SIZE) == 0;

	return 0;
}

int spmi_device_add(struct spmi_device *sdev)
{
	struct spmi_controller *ctrl = sdev->ctrl;
	int err;

	dev_set_name(&sdev->dev, "%d-%02x", ctrl->nr, sdev->usid);

	err = device_add(&sdev->dev);
	if (err < 0) {
		dev_err(&sdev->dev, "Can't add %s, status %d\n",
			dev_name(&sdev->dev), err);
		goto err_device_add;
	}

	dev_dbg(&sdev->dev, "device %s registered\n", dev_name(&sdev->dev));

err_device_add:
	return err;
}
EXPORT_SYMBOL_GPL(spmi_device_add);

void spmi_device_remove(struct spmi_device *sdev)
{
	device_unregister(&sdev->dev);
}
EXPORT_SYMBOL_GPL(spmi_device_remove);

static inline int
spmi_cmd(struct spmi_controller *ctrl, u8 opcode, u8 sid)
{
	if (!ctrl || !ctrl->cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

	return ctrl->cmd(ctrl, opcode, sid);
}

static inline int spmi_read_cmd(struct spmi_controller *ctrl,
				u8 opcode, u8 sid, u16 addr, u8 bc, u8 *buf)
{
	if (!ctrl || !ctrl->read_cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

	return ctrl->read_cmd(ctrl, opcode, sid, addr, bc, buf);
}

static inline int spmi_write_cmd(struct spmi_controller *ctrl,
				 u8 opcode, u8 sid, u16 addr, u8 bc,
				 const u8 *buf)
{
	if (!ctrl || !ctrl->write_cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

	return ctrl->write_cmd(ctrl, opcode, sid, addr, bc, buf);
}

/*
 * register read/write: 5-bit address, 1 byte of data
 * extended register read/write: 8-bit address, up to 16 bytes of data
 * extended register read/write long: 16-bit address, up to 8 bytes of data
 */

int spmi_register_read(struct spmi_device *sdev, u8 addr, u8 *buf)
{
	/* 5-bit register address */
	if (addr > 0x1F)
		return -EINVAL;

	return spmi_read_cmd(sdev->ctrl, SPMI_CMD_READ, sdev->usid, addr, 0,
			     buf);
}
EXPORT_SYMBOL_GPL(spmi_register_read);

int spmi_ext_register_read(struct spmi_device *sdev, u8 addr, u8 *buf,
			   size_t len)
{
	/* 8-bit register address, up to 16 bytes */
	if (len == 0 || len > 16)
		return -EINVAL;

	return spmi_read_cmd(sdev->ctrl, SPMI_CMD_EXT_READ, sdev->usid, addr,
			     len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_read);

int spmi_ext_register_readl(struct spmi_device *sdev, u16 addr, u8 *buf,
			    size_t len)
{
	/* 16-bit register address, up to 8 bytes */
	if (len == 0 || len > 8)
		return -EINVAL;

	return spmi_read_cmd(sdev->ctrl, SPMI_CMD_EXT_READL, sdev->usid, addr,
			     len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_readl);

int spmi_register_write(struct spmi_device *sdev, u8 addr, const u8 *buf)
{
	/* 5-bit register address */
	if (addr > 0x1F)
		return -EINVAL;

	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_WRITE, sdev->usid, addr, 0,
			      buf);
}
EXPORT_SYMBOL_GPL(spmi_register_write);

int spmi_register_zero_write(struct spmi_device *sdev, u8 data)
{
	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_ZERO_WRITE, sdev->usid, 0,
			      0, &data);
}
EXPORT_SYMBOL_GPL(spmi_register_zero_write);

int spmi_ext_register_write(struct spmi_device *sdev, u8 addr, const u8 *buf,
			    size_t len)
{
	/* 8-bit register address, up to 16 bytes */
	if (len == 0 || len > 16)
		return -EINVAL;

	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_EXT_WRITE, sdev->usid, addr,
			      len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_write);

int spmi_ext_register_writel(struct spmi_device *sdev, u16 addr, const u8 *buf,
			     size_t len)
{
	/* 4-bit Slave Identifier, 16-bit register address, up to 8 bytes */
	if (len == 0 || len > 8)
		return -EINVAL;

	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_EXT_WRITEL, sdev->usid,
			      addr, len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_writel);

int spmi_command_reset(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_RESET, sdev->usid);
}
EXPORT_SYMBOL_GPL(spmi_command_reset);

int spmi_command_sleep(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_SLEEP, sdev->usid);
}
EXPORT_SYMBOL_GPL(spmi_command_sleep);

int spmi_command_wakeup(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_WAKEUP, sdev->usid);
}
EXPORT_SYMBOL_GPL(spmi_command_wakeup);

int spmi_command_shutdown(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_SHUTDOWN, sdev->usid);
}
EXPORT_SYMBOL_GPL(spmi_command_shutdown);

static int spmi_drv_probe(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);
	int err = 0;

	if (sdrv->probe)
		err = sdrv->probe(to_spmi_device(dev));

	return err;
}

static int spmi_drv_remove(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);
	int err = 0;

	if (sdrv->remove)
		err = sdrv->remove(to_spmi_device(dev));

	return err;
}

static void spmi_drv_shutdown(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);

	if (sdrv->shutdown)
		sdrv->shutdown(to_spmi_device(dev));
}

static struct bus_type spmi_bus_type = {
	.name		= "spmi",
	.match		= spmi_device_match,
	.pm		= &spmi_pm_ops,
	.probe		= spmi_drv_probe,
	.remove		= spmi_drv_remove,
	.shutdown	= spmi_drv_shutdown,
};

struct spmi_device *spmi_device_alloc(struct spmi_controller *ctrl)
{
	struct spmi_device *sdev;

	sdev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return NULL;

	sdev->ctrl = ctrl;
	device_initialize(&sdev->dev);
	sdev->dev.parent = &ctrl->dev;
	sdev->dev.bus = &spmi_bus_type;
	sdev->dev.type = &spmi_dev_type;
	return sdev;
}
EXPORT_SYMBOL_GPL(spmi_device_alloc);

struct spmi_controller *spmi_controller_alloc(struct device *parent,
					      size_t size)
{
	struct spmi_controller *ctrl;
	int id;

	if (WARN_ON(!parent))
		return NULL;

	ctrl = kzalloc(sizeof(*ctrl) + size, GFP_KERNEL);
	if (!ctrl)
		return NULL;

	device_initialize(&ctrl->dev);
	ctrl->dev.type = &spmi_ctrl_type;
	ctrl->dev.bus = &spmi_bus_type;
	ctrl->dev.parent = parent;
	ctrl->dev.of_node = parent->of_node;
	spmi_controller_set_drvdata(ctrl, &ctrl[1]);

	idr_preload(GFP_KERNEL);
	mutex_lock(&ctrl_idr_lock);
	id = idr_alloc(&ctrl_idr, ctrl, 0, 0, GFP_NOWAIT);
	mutex_unlock(&ctrl_idr_lock);
	idr_preload_end();

	if (id < 0) {
		dev_err(parent,
			"unable to allocate SPMI controller identifier.\n");
		spmi_controller_put(ctrl);
		return NULL;
	}

	ctrl->nr = id;
	dev_set_name(&ctrl->dev, "spmi-%d", id);

	dev_dbg(&ctrl->dev, "allocated controller 0x%p id %d\n", ctrl, id);
	return ctrl;
}
EXPORT_SYMBOL_GPL(spmi_controller_alloc);

static int of_spmi_register_devices(struct spmi_controller *ctrl)
{
	struct device_node *node;
	int err;

	dev_dbg(&ctrl->dev, "of_spmi_register_devices\n");

	if (!ctrl->dev.of_node)
		return -ENODEV;

	dev_dbg(&ctrl->dev, "looping through children\n");

	for_each_available_child_of_node(ctrl->dev.of_node, node) {
		struct spmi_device *sdev;
		u32 reg[2];

		dev_dbg(&ctrl->dev, "adding child %s\n", node->full_name);

		err = of_property_read_u32_array(node, "reg", reg, 2);
		if (err) {
			dev_err(&ctrl->dev,
				"node %s does not have 'reg' property\n",
				node->full_name);
			continue;
		}

		if (reg[1] != SPMI_USID) {
			dev_err(&ctrl->dev,
				"node %s contains unsupported 'reg' entry\n",
				node->full_name);
			continue;
		}

		if (reg[0] > 0xF) {
			dev_err(&ctrl->dev,
				"invalid usid on node %s\n",
				node->full_name);
			continue;
		}

		dev_dbg(&ctrl->dev, "read usid %02x\n", reg[0]);

		sdev = spmi_device_alloc(ctrl);
		if (!sdev)
			continue;

		sdev->dev.of_node = node;
		sdev->usid = (u8) reg[0];

		err = spmi_device_add(sdev);
		if (err) {
			dev_err(&sdev->dev,
				"failure adding device. status %d\n", err);
			spmi_device_put(sdev);
			continue;
		}
	}

	return 0;
}

int spmi_controller_add(struct spmi_controller *ctrl)
{
	int ret;

	/* Can't register until after driver model init */
	if (WARN_ON(!spmi_bus_type.p))
		return -EAGAIN;

	ret = device_add(&ctrl->dev);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_OF))
		of_spmi_register_devices(ctrl);

	dev_dbg(&ctrl->dev, "spmi-%d registered: dev:%p\n",
		ctrl->nr, &ctrl->dev);

	return 0;
};
EXPORT_SYMBOL_GPL(spmi_controller_add);

/* Remove a device associated with a controller */
static int spmi_ctrl_remove_device(struct device *dev, void *data)
{
	struct spmi_device *spmidev = to_spmi_device(dev);
	if (dev->type == &spmi_dev_type)
		spmi_device_remove(spmidev);
	return 0;
}

/**
 * spmi_controller_remove: Controller tear-down.
 * @ctrl: controller to be removed.
 *
 * Controller added with the above API is torn down using this API.
 */
int spmi_controller_remove(struct spmi_controller *ctrl)
{
	int dummy;

	if (!ctrl)
		return -EINVAL;

	dummy = device_for_each_child(&ctrl->dev, NULL,
				      spmi_ctrl_remove_device);
	device_unregister(&ctrl->dev);
	return 0;
}
EXPORT_SYMBOL_GPL(spmi_controller_remove);

int spmi_driver_register(struct spmi_driver *drv)
{
	drv->driver.bus = &spmi_bus_type;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(spmi_driver_register);

static void __exit spmi_exit(void)
{
	bus_unregister(&spmi_bus_type);
}
module_exit(spmi_exit);

static int __init spmi_init(void)
{
	return bus_register(&spmi_bus_type);
}
postcore_initcall(spmi_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SPMI module");
MODULE_ALIAS("platform:spmi");
