
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <linux/qcom_smd-rpm.h>

struct msm_rpm_reg {
	struct mutex lock;
	struct device *dev;
	struct regulator_desc desc;

	const int resource;

	int is_enabled;

	int uV;
};

static const struct msm_rpm_reg pm8841_s1 = { .desc.name = "pm8841_s1", .resource = MSM_RPM_PM8841_S1 };
static const struct msm_rpm_reg pm8841_s2 = { .desc.name = "pm8841_s2", .resource = MSM_RPM_PM8841_S2 };

static const struct msm_rpm_reg pm8941_s1 = { .desc.name = "pm8941_s1", .resource = MSM_RPM_PM8941_S1 };
static const struct msm_rpm_reg pm8941_s2 = { .desc.name = "pm8941_s2", .resource = MSM_RPM_PM8941_S2 };
static const struct msm_rpm_reg pm8941_s3 = { .desc.name = "pm8941_s3", .resource = MSM_RPM_PM8941_S3 };

static const struct msm_rpm_reg pm8941_ldo3 = { .desc.name = "pm8941_ldo3", .resource = MSM_RPM_PM8941_L3 };
static const struct msm_rpm_reg pm8941_ldo6 = { .desc.name = "pm8941_ldo6", .resource = MSM_RPM_PM8941_L6 };
static const struct msm_rpm_reg pm8941_ldo9 = { .desc.name = "pm8941_ldo9", .resource = MSM_RPM_PM8941_L9 };
static const struct msm_rpm_reg pm8941_ldo11 = { .desc.name = "pm8941_ldo11", .resource = MSM_RPM_PM8941_L11 };
static const struct msm_rpm_reg pm8941_ldo19 = { .desc.name = "pm8941_ldo19", .resource = MSM_RPM_PM8941_L19 };
static const struct msm_rpm_reg pm8941_ldo20 = { .desc.name = "pm8941_ldo20", .resource = MSM_RPM_PM8941_L20 };
static const struct msm_rpm_reg pm8941_ldo22 = { .desc.name = "pm8941_ldo22", .resource = MSM_RPM_PM8941_L22 };

static const struct of_device_id rpm_of_match[] = {
	{ .compatible = "qcom,pm8841-s1", .data = &pm8841_s1 },
	{ .compatible = "qcom,pm8841-s2", .data = &pm8841_s2 },
	{ .compatible = "qcom,pm8941-s1", .data = &pm8941_s1 },
	{ .compatible = "qcom,pm8941-s2", .data = &pm8941_s2 },
	{ .compatible = "qcom,pm8941-s3", .data = &pm8941_s3 },
	{ .compatible = "qcom,pm8941-ldo3", .data = &pm8941_ldo3 },
	{ .compatible = "qcom,pm8941-ldo6", .data = &pm8941_ldo6 },
	{ .compatible = "qcom,pm8941-ldo9", .data = &pm8941_ldo9 },
	{ .compatible = "qcom,pm8941-ldo11", .data = &pm8941_ldo11 },
	{ .compatible = "qcom,pm8941-ldo19", .data = &pm8941_ldo19 },
	{ .compatible = "qcom,pm8941-ldo20", .data = &pm8941_ldo20 },
	{ .compatible = "qcom,pm8941-ldo22", .data = &pm8941_ldo22 },
};
MODULE_DEVICE_TABLE(of, rpm_of_match);

struct rpm_regulator_data {
	u32 key;
	u32 nbytes;
	u32 value;
};

#define SWEN 0x6e657773 /* 'swen' */
#define UV 0x7675 /* 'uv' */

int qcom_rpm_smd_write(const struct device *dev, int resource, void *buf, size_t count);

static int rpm_reg_enable(struct regulator_dev *rdev)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);
	struct rpm_regulator_data data;
	int ret;

	data.key = SWEN;
	data.nbytes = sizeof(u32);
	data.value = 1;

	mutex_lock(&vreg->lock);
	ret = qcom_rpm_smd_write(vreg->dev->parent, vreg->resource, &data, sizeof(data));
	if (!ret)
		vreg->is_enabled = 1;
	mutex_unlock(&vreg->lock);

	return ret;
}

static int rpm_reg_is_enabled(struct regulator_dev *rdev)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->is_enabled;
}

static int rpm_reg_disable(struct regulator_dev *rdev)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);
	struct rpm_regulator_data data;
	int ret;

	data.key = SWEN;
	data.nbytes = sizeof(u32);
	data.value = 0;

	mutex_lock(&vreg->lock);
	ret = qcom_rpm_smd_write(vreg->dev->parent, vreg->resource, &data, sizeof(data));
	if (!ret)
		vreg->is_enabled = 0;
	mutex_unlock(&vreg->lock);

	return ret;
}

static int rpm_reg_get_voltage(struct regulator_dev *rdev)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);
	return vreg->uV;
}

static int rpm_reg_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV,
			       unsigned *selector)
{
	struct msm_rpm_reg *vreg = rdev_get_drvdata(rdev);
	struct rpm_regulator_data data;
	int ret = 0;

	data.key = UV;
	data.nbytes = sizeof(u32);
	data.value = min_uV;

	ret = qcom_rpm_smd_write(vreg->dev->parent, vreg->resource, &data, sizeof(data));

	return ret;
}

static unsigned int rpm_reg_get_optimum_mode(struct regulator_dev *rdev, int input_uV, int output_uV, int load_uA)
{
	return REGULATOR_MODE_NORMAL;
}

static int rpm_reg_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	return 0;
}

static struct regulator_ops ldo_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,

	.get_voltage = rpm_reg_get_voltage,
	.set_voltage = rpm_reg_set_voltage,

	.get_optimum_mode = rpm_reg_get_optimum_mode,
	.set_mode = rpm_reg_set_mode,
};

static int rpm_reg_probe(struct platform_device *pdev)
{
	struct regulator_init_data *initdata;
	const struct msm_rpm_reg *template;
	const struct of_device_id *match;
	struct regulator_config config = { };
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	struct msm_rpm_reg *vreg;

	match = of_match_device(rpm_of_match, &pdev->dev);
	template = match->data;

	initdata = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node);
	if (!initdata)
		return -EINVAL;

	vreg = devm_kmalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		dev_err(&pdev->dev, "failed to allocate vreg\n");
		return -ENOMEM;
	}
	memcpy(vreg, template, sizeof(*vreg));

	mutex_init(&vreg->lock);
	vreg->dev = &pdev->dev;

	/* XXX:
	 * tighten constraints of initdata
	 * round min_uV and max_uV to step values
	 */

	/*
	initdata->constraints.valid_ops_mask |= REGULATOR_CHANGE_VOLTAGE;
	*/
	initdata->constraints.valid_ops_mask |= REGULATOR_CHANGE_DRMS;
	initdata->constraints.valid_ops_mask |= REGULATOR_CHANGE_MODE;

	initdata->constraints.valid_modes_mask |= REGULATOR_MODE_NORMAL;
	initdata->constraints.valid_modes_mask |= REGULATOR_MODE_IDLE;

	initdata->constraints.input_uV = initdata->constraints.max_uV;

	if (initdata->constraints.apply_uV &&
	    initdata->constraints.min_uV == initdata->constraints.max_uV) {
		vreg->uV = initdata->constraints.min_uV;
	}

	desc = &vreg->desc;
	desc->id = -1;
	desc->ops = &ldo_ops;
	desc->owner = THIS_MODULE;
	desc->type = REGULATOR_VOLTAGE;

	config.dev = &pdev->dev;
	config.init_data = initdata;
	config.driver_data = vreg;
	config.of_node = pdev->dev.of_node;
        rdev = devm_regulator_register(&pdev->dev, desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "can't register regulator\n");
		return PTR_ERR(rdev);
	}

	platform_set_drvdata(pdev, rdev);

	return 0;
}

static int rpm_reg_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rpm_reg_driver = {
	.probe          = rpm_reg_probe,
	.remove         = rpm_reg_remove,
	.driver  = {
		.name  = "msm_rpm_smd_reg",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rpm_of_match),
	},
};

static int __init rpm_reg_init(void)
{
	return platform_driver_register(&rpm_reg_driver);
}
arch_initcall(rpm_reg_init);

static void __exit rpm_reg_exit(void)
{
	platform_driver_unregister(&rpm_reg_driver);
}
module_exit(rpm_reg_exit)

MODULE_DESCRIPTION("Qualcomm RPM regulator driver");
MODULE_LICENSE("GPLv2");

