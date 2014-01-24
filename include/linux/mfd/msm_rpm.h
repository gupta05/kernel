#ifndef __MSM_RPM_H__
#define __MSM_RPM_H__

enum {
	MSM_RPM_INVALID,
	MSM_RPM_VERSION,
	MSM_RPM_REQ_CTX,
	MSM_RPM_REQ_SEL,
	MSM_RPM_ACK_CTX,
	MSM_RPM_ACK_SEL,
	MSM_RPM_PM8921_L16,
};

int msm_rpm_read_status(const struct device *dev, int resource, u32 *buf, size_t count);
int msm_rpm_write(const struct device *dev, int resource, u32 *buf, size_t count);

#endif
