#ifndef __QCOM_SMD_RPM_H__
#define __QCOM_SMD_RPM_H__

struct qcom_smd_rpm;

int qcom_rpm_smd_write(struct qcom_smd_rpm *rpm, int resource,
		       void *buf, size_t count);

#endif
