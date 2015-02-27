#ifndef __QCOM_SMEM_H__
#define __QCOM_SMEM_H__

struct device_node;
struct qcom_smem;

int qcom_smem_alloc(unsigned item, size_t size);
int qcom_smem_get(unsigned item, void **ptr, size_t *size);
int qcom_smem_put(unsigned item);

unsigned qcom_smem_get_free_space(void);

#endif
