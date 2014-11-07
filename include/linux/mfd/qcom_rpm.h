#ifndef __QCOM_RPM_H__
#define __QCOM_RPM_H__

#include <linux/types.h>

struct device;
struct qcom_rpm;

#define RPM_ACTIVE_STATE	0
#define RPM_SLEEP_STATE		1

struct qcom_rpm *dev_get_qcom_rpm(struct device *dev);
int qcom_rpm_write(struct qcom_rpm *rpm, int state, int resource, u32 *buf, size_t count);

#endif
