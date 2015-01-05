#ifndef __QCOM_SMD_RPM_H__
#define __QCOM_SMD_RPM_H__

struct qcom_smd_rpm;

#define QCOM_SMD_RPM_ACTIVE_STATE        0
#define QCOM_SMD_RPM_SLEEP_STATE         1

int qcom_rpm_smd_write(struct qcom_smd_rpm *rpm,
		       int state,
		       u32 resource_type, u32 resource_id,
		       void *buf, size_t count);

#endif
