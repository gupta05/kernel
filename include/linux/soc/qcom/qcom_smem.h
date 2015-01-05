#ifndef __QCOM_SMEM_H__
#define __QCOM_SMEM_H__

struct device_node;
struct qcom_smem;

struct qcom_smem *of_get_qcom_smem(struct device_node *node);

int qcom_smem_alloc(struct qcom_smem *smem, unsigned smem_id, size_t size);
int qcom_smem_get(struct qcom_smem *smem, unsigned item, void **ptr, size_t *size);

unsigned qcom_smem_get_free_space(struct qcom_smem *smem);

#endif
