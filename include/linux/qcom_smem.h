#ifndef __QCOM_SMEM_H__
#define __QCOM_SMEM_H__

struct qcom_smem;

struct qcom_smem *dev_get_qcom_smem(struct device *dev);

int qcom_smem_get(struct qcom_smem *smem, int smem_id, void **ptr, size_t *size);

#endif
