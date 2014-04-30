#ifndef __QCOM_SMEM_H__
#define __QCOM_SMEM_H__

struct qcom_smem;
struct qcom_smsm;

struct qcom_smem *dev_get_qcom_smem(struct device *dev);
struct qcom_smsm *dev_get_qcom_smsm(struct device *dev);

int qcom_smem_get(struct qcom_smem *smem, int smem_id, void **ptr, size_t *size);
int qcom_smem_signal(struct qcom_smem *smem, int offset, int bit);

int qcom_smem_alloc(struct qcom_smem *smem, int smem_id, size_t size);

/* XXX: */
int qcom_smsm_change_state(struct qcom_smsm *smsm, u32 clear_mask, u32 set_mask);

#endif
