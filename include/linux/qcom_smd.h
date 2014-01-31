#ifndef __LINUX_MSM_SMEM_H__
#define __LINUX_MSM_SMEM_H__

#include <linux/device.h>
#include <linux/mod_devicetable.h>

struct qcom_smd_device {
	struct device dev;
};

/**
 * struct qcom_smd_driver - qcom smd driver struct
 * @drv: underlying device driver
 * @id_table: rpmsg ids serviced by this driver
 * @probe: invoked when a matching rpmsg channel (i.e. device) is found
 * @remove: invoked when the rpmsg channel is removed
 * @callback: invoked when an inbound message is received on the channel
 */
struct qcom_smd_driver {
	struct device_driver driver;
	int (*probe)(struct qcom_smd_device *dev);
	void (*remove)(struct qcom_smd_device *dev);
	int (*callback)(struct qcom_smd_device *, void *, size_t);
};

int register_qcom_smd_driver(struct qcom_smd_driver *drv);

int qcom_smd_send(struct qcom_smd_device *qsdev, void *data, int len);

#endif

