#include <linux/module.h>
#include <linux/wcnss_wlan.h>
#include <linux/soc/qcom/smd.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/wait.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <net/mac80211.h>

#define WCNSS_MAX_CH_NUM	45

extern int prima_driver_init(void);

struct wcnss {
	struct device *dev;
	struct qcom_smd_channel *channel;

	void (*smd_cb)(void*, unsigned);
	void *smd_cb_data;

	int tx_irq;
	int rx_irq;

	struct qcom_smem_state *tx_enable_state;
	struct qcom_smem_state *tx_rings_empty_state;
	unsigned tx_enable_state_bit;
	unsigned tx_rings_empty_state_bit;

	struct resource mmio;

	u8 mac[ETH_ALEN];
	unsigned int serial;

	u16 unsafe_ch_count;
	u16 unsafe_ch_list[WCNSS_MAX_CH_NUM];

	spinlock_t rsp_pkt_lock;
	struct list_head rsp_pkt_list;
	struct work_struct init_work;
};

static struct wcnss *_wcnss;

struct device *wcnss_wlan_get_device(void)
{
	return _wcnss->dev;
}

void wcnss_get_monotonic_boottime(struct timespec *ts)
{
        get_monotonic_boottime(ts);
}

struct resource *wcnss_wlan_get_memory_map(struct device *dev)
{
	return &_wcnss->mmio;
}

int wcnss_wlan_get_dxe_tx_irq(struct device *dev)
{
	return _wcnss->tx_irq;
}

int wcnss_wlan_get_dxe_rx_irq(struct device *dev)
{
	return _wcnss->rx_irq;
}

void wcnss_wlan_register_pm_ops(struct device *dev, const struct dev_pm_ops *pm_ops)
{

}

void wcnss_wlan_unregister_pm_ops(struct device *dev, const struct dev_pm_ops *pm_ops)
{

}

void wcnss_register_thermal_mitigation(struct device *dev, void (*tm_notify)(struct device *dev, int))
{

}

void wcnss_unregister_thermal_mitigation(void (*tm_notify)(struct device *dev, int))
{

}

int wcnss_req_power_on_lock(char *driver_name)
{
	return 0;
}

int wcnss_free_power_on_lock(char *driver_name)
{
	return 0;
}

unsigned int wcnss_get_serial_number(void)
{
	return _wcnss->serial;
}

int wcnss_get_wlan_mac_address(char mac_addr[WLAN_MAC_ADDR_SIZE])
{
	memcpy(mac_addr, _wcnss->mac, WLAN_MAC_ADDR_SIZE);

	return 0;
}

void wcnss_allow_suspend(void)
{

}

void wcnss_prevent_suspend(void)
{

}

int wcnss_hardware_type(void)
{
	return WCNSS_PRONTO_HW;
}

void wcnss_reset_intr(void)
{

}

void wcnss_suspend_notify(void)
{

}

void wcnss_resume_notify(void)
{

}

void wcnss_riva_log_debug_regs(void)
{

}

void wcnss_pronto_log_debug_regs(void)
{

}

int wcnss_is_hw_pronto_ver3(void)
{
	return false;
}

extern int wcnss_ctrl_done_loading_nv;

int wcnss_device_ready(void)
{
	return !!wcnss_ctrl_done_loading_nv;
}

int wcnss_cbc_complete(void)
{
	return true;
}

int wcnss_device_is_shutdown(void)
{
	return false;
}

void wcnss_riva_dump_pmic_regs(void)
{

}

u32 wcnss_get_wlan_rx_buff_count(void)
{
	return 1024;
}

int wcnss_wlan_iris_xo_mode(void)
{
	return WCNSS_XO_48MHZ;
}

#ifdef CONFIG_WCNSS_REGISTER_DUMP_ON_BITE
void wcnss_log_debug_regs_on_bite(void)
{

}
#endif

int wcnss_set_wlan_unsafe_channel(u16 *unsafe_ch_list, u16 ch_count)
{
	BUG_ON(ch_count >= ARRAY_SIZE(_wcnss->unsafe_ch_list));

	memcpy(_wcnss->unsafe_ch_list, unsafe_ch_list, ch_count * sizeof(unsafe_ch_list[0]));
	_wcnss->unsafe_ch_count = ch_count;

	return 0;
}

int wcnss_get_wlan_unsafe_channel(u16 *unsafe_ch_list, u16 buffer_size, u16 *ch_count)
{
	BUG_ON(buffer_size < _wcnss->unsafe_ch_count * sizeof(unsafe_ch_list[0]));
	memcpy(unsafe_ch_list, _wcnss->unsafe_ch_list, _wcnss->unsafe_ch_count * sizeof(unsafe_ch_list[0]));
	*ch_count = _wcnss->unsafe_ch_count;

	return 0;
}

int wcnss_change_smsm_state(unsigned state, u32 set, u32 clr)
{
#if 0
	return qcom_smem_state_update_bits(_wcnss->tx_enable_state, set | clr, set);
#endif
	return 0;
}

struct wcnss_rsp_pkt {
	struct list_head list;
	size_t msg_len;
	u8 data[];
};

static int wcnss_smd_callback(struct qcom_smd_device *sdev, const void *buf, size_t len)
{
	struct wcnss_rsp_pkt *pkt;

	pkt = kmalloc(sizeof(struct wcnss_rsp_pkt) + len, GFP_ATOMIC);
	if (!pkt)
		return -ENOMEM;

	spin_lock(&_wcnss->rsp_pkt_lock);
	list_add_tail(&pkt->list, &_wcnss->rsp_pkt_list);
	pkt->msg_len = len;
	memcpy(pkt->data, buf, len);
	spin_unlock(&_wcnss->rsp_pkt_lock);

	_wcnss->smd_cb(_wcnss->smd_cb_data, WCNSS_SMD_EVENT_DATA);

	return 0;
}

int wcnss_smd_cur_packet_size(void)
{
	struct wcnss_rsp_pkt *pkt;
	struct wcnss *wcn = _wcnss;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&wcn->rsp_pkt_lock, flags);
	if (!list_empty(&wcn->rsp_pkt_list)) {
		pkt = list_first_entry(&wcn->rsp_pkt_list, struct wcnss_rsp_pkt, list);
		ret = pkt->msg_len;
	}
	spin_unlock_irqrestore(&wcn->rsp_pkt_lock, flags);

	return ret;

}

int wcnss_smd_read_avail(void)
{
	return wcnss_smd_cur_packet_size();
}

int wcnss_smd_read(void *buf, size_t len)
{
	struct wcnss_rsp_pkt *pkt;
	struct wcnss *wcn = _wcnss;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&wcn->rsp_pkt_lock, flags);

	pkt = list_first_entry(&wcn->rsp_pkt_list, struct wcnss_rsp_pkt, list);
	if (len != pkt->msg_len) {
		ret = -EINVAL;
		goto unlock;
	}

	memcpy(buf, pkt->data, pkt->msg_len);
	ret = pkt->msg_len;

	list_del(wcn->rsp_pkt_list.next);
	kfree(pkt);

unlock:
	spin_unlock_irqrestore(&wcn->rsp_pkt_lock, flags);
	return ret;
}

int wcnss_smd_write(const void *buf, size_t len)
{
	int ret;

	ret = qcom_smd_send(_wcnss->channel, buf, len);
	return ret < 0 ? ret : len;
}

int wcnss_smd_close(void)
{
	return 0;
}

int wcnss_smd_open_channel(void *data, void (*cb)(void*, unsigned))
{
	_wcnss->smd_cb = cb;
	_wcnss->smd_cb_data = data;

	_wcnss->smd_cb(_wcnss->smd_cb_data, WCNSS_SMD_EVENT_OPEN);

	return 0;
}

static int wcnss_platform_get_resources(struct wcnss *wcn, struct device *dev)
{
	u32 mmio[2];
	int ret;

	/* Set TX IRQ */
	wcn->tx_irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!wcn->tx_irq) {
		dev_err(dev, "failed to get tx_irq\n");
		return -ENOENT;
	}

	/* Set RX IRQ */
	wcn->rx_irq = irq_of_parse_and_map(dev->of_node, 1);
	if (!wcn->rx_irq) {
		dev_err(dev, "failed to get rx_irq\n");
		return -ENOENT;
	}

#if 0
	/* Acquire SMSM tx enable handle */
	wcn->tx_enable_state = qcom_smem_state_get(dev, "tx-enable",
			&wcn->tx_enable_state_bit);
	if (IS_ERR(wcn->tx_enable_state)) {
		dev_err(dev, "failed to get tx-enable state\n");
		return -ENOENT;
	}

	/* Acquire SMSM tx rings empty handle */
	wcn->tx_rings_empty_state = qcom_smem_state_get(dev, "tx-rings-empty",
			&wcn->tx_rings_empty_state_bit);
	if (IS_ERR(wcn->tx_rings_empty_state)) {
		dev_err(dev, "failed to get tx-rings-empty state\n");
		return -ENOENT;
	}
#endif

	/* Map the memory */
	ret = of_property_read_u32_array(dev->of_node, "qcom,wcnss-mmio", mmio, 2);
	if (ret) {
		dev_err(dev, "failed to get qcom,wcnss-mmio\n");
		return -ENOENT;
	}

	wcn->mmio.flags = IORESOURCE_MEM;
	wcn->mmio.start = mmio[0];
	wcn->mmio.end = mmio[0] + mmio[1];

	return 0;
}

static void wcnss_init_prima(struct work_struct *work)
{
	int ret;

	ret = prima_driver_init();
	if (ret < 0)
		dev_err(_wcnss->dev, "failed to initialize prima\n");
}

extern int wcnss_prealloc_init(void);
extern void wcnss_prealloc_deinit(void);

static int wcnss_probe(struct qcom_smd_device *sdev)
{
	struct wcnss *wcn;
	const u8 *addr;
	int ret;

	dev_dbg(&sdev->dev, "platform probe\n");

	sdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	of_dma_configure_ops(&sdev->dev, sdev->dev.of_node);

	_wcnss = wcn = devm_kzalloc(&sdev->dev, sizeof(struct wcnss), GFP_KERNEL);
	if (!wcn)
		return -ENOMEM;

	wcn->dev = &sdev->dev;
	wcn->channel = sdev->channel;

	wcn->serial = 123;

	spin_lock_init(&wcn->rsp_pkt_lock);
	INIT_LIST_HEAD(&wcn->rsp_pkt_list);
	INIT_WORK(&wcn->init_work, wcnss_init_prima);

	ret = wcnss_platform_get_resources(wcn, &sdev->dev);
	if (ret)
		return ret;

	addr = of_get_property(sdev->dev.of_node, "local-mac-address", &ret);
	if (addr && ret != ETH_ALEN) {
		dev_err(&sdev->dev, "invalid local-mac-address\n");
		return -EINVAL;
	} else if (addr) {
		dev_info(&sdev->dev, "mac address: %pM\n", addr);
		memcpy(wcn->mac, addr, ETH_ALEN);
	}

	wcnss_prealloc_init();

	schedule_work(&wcn->init_work);

	return 0;
}

static void wcnss_remove(struct qcom_smd_device *sdev)
{
	struct ieee80211_hw *hw = dev_get_drvdata(&sdev->dev);
	struct wcnss *wcn = hw->priv;

	dev_dbg(&sdev->dev, "platform remove\n");

#if 0
	qcom_smem_state_put(wcn->tx_enable_state);
	qcom_smem_state_put(wcn->tx_rings_empty_state);
#endif

	wcnss_prealloc_deinit();
}

static const struct of_device_id wcnss_of_match[] = {
	{ .compatible = "qcom,wcn3620-wlan" },
	{ .compatible = "qcom,wcn3660-wlan" },
	{ .compatible = "qcom,wcn3680-wlan" },
	{}
};
MODULE_DEVICE_TABLE(of, wcnss_of_match);

static struct qcom_smd_driver wcnss_driver = {
	.probe      = wcnss_probe,
	.remove     = wcnss_remove,
	.callback   = wcnss_smd_callback,
	.driver         = {
		.name   = "wcnss",
		.of_match_table = wcnss_of_match,
		.owner  = THIS_MODULE,
	},
};

module_qcom_smd_driver(wcnss_driver);
