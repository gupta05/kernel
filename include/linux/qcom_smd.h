#ifndef __LINUX_MSM_SMEM_H__
#define __LINUX_MSM_SMEM_H__

#include <linux/device.h>
#include <linux/mod_devicetable.h>

struct qcom_smd;
struct qcom_smd_channel;
struct qcom_smd_lookup;

struct qcom_smd_device {
	struct device dev;

	struct qcom_smd *smd;
	struct qcom_smd_lookup *lookup;
};

/**
 * struct qcom_smd_driver - qcom smd driver struct
 * @driver: underlying device driver
 * @probe: invoked when a matching smd channel (i.e. device) is found
 * @remove: invoked when the smd channel is removed
 * @callback: invoked when an inbound message is received on the channel
 */
struct qcom_smd_driver {
	struct device_driver driver;
	int (*probe)(struct qcom_smd_device *dev);
	int (*remove)(struct qcom_smd_device *dev);
};

int register_qcom_smd_driver(struct qcom_smd_driver *drv);

struct qcom_smd_channel *qcom_smd_request_channel(struct qcom_smd_device *qsdev, const char *name, int (*callback)(struct qcom_smd_channel *, void *, size_t, void *), void *dev);

int qcom_smd_send(struct qcom_smd_channel *channel, void *data, int len);

int qcom_smsm_change_state(struct qcom_smd_device *qsdev, u32 clear_mask, u32 set_mask);

#endif

