/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8921-core.h>

#include <dt-bindings/pinctrl/qcom,pm8xxx-gpio.h>

#include "core.h"
#include "pinconf.h"
#include "pinctrl-utils.h"

/* direction */
#define PM8XXX_GPIO_DIR_OUT		BIT(0)
#define PM8XXX_GPIO_DIR_IN		BIT(1)

/* output buffer */
#define PM8XXX_GPIO_PUSH_PULL		0
#define PM8XXX_GPIO_OPEN_DRAIN		1

/* bias */
#define PM8XXX_GPIO_BIAS_PU_30		0
#define PM8XXX_GPIO_BIAS_PU_1P5		1
#define PM8XXX_GPIO_BIAS_PU_31P5	2
#define PM8XXX_GPIO_BIAS_PU_1P5_30	3
#define PM8XXX_GPIO_BIAS_PD		4
#define PM8XXX_GPIO_BIAS_NP		5

/* GPIO registers */
#define SSBI_REG_ADDR_MPP_BASE		XXX
#define SSBI_REG_ADDR_MPP(n)		(SSBI_REG_ADDR_MPP_BASE + n)

#define PM8XXX_GPIO_WRITE		BIT(7)

/* custom pinconf parameters */
#define PM8XXX_QCOM_DRIVE_STRENGH	(PIN_CONFIG_END + 1)
#define PM8XXX_QCOM_PULL_UP_STRENGTH	(PIN_CONFIG_END + 2)

/* number of pinmux functions, includes gpio & paired */
#define PM8XXX_NUM_FUNCTIONS		4
struct pm8xxx_pingroup {
	const char *name;
	unsigned functions[PM8XXX_NUM_FUNCTIONS];
};

struct pm8xxx_function {
	const char *name;
	const char * const *groups;
	unsigned ngroups;
};

/**
 * struct pm8xxx_gpio_data - static data for the gpio block
 * @npins:		number of available pins
 * @power_sources:	mapping table from dt power source values to register
 *			selector values
 * @npower_sources:	number of possible power sources
 * @pin_groups:		list of pin_groups and their possible functions
 * @functions:		list of mappings from function to pin groups
 */
struct pm8xxx_gpio_data {
	unsigned npins;
	const unsigned *power_sources;
	unsigned npower_sources;

	struct pm8xxx_pingroup *pin_groups;
	struct pm8xxx_function *functions;
};

/**
 * struct pm8xxx_pin_data - dynamic configuration for a pin
 * @reg:		address of the control register
 * @irq:		IRQ from the PMIC interrupt controller
 * @power_source:	logical selected voltage source, mapping in static data
 *			is used translate to register values
 * @direction:		mask of DIR_OUT and DIR_IN
 * @output_buffer:	output buffer configuration
 * @output_value:	configured output value
 * @bias:		register view of configured bias
 * @pull_up_strength:	placeholder for selected pull up strength
 *			only used to configure bias when pull up is selected
 * @output_strength:	selector of output-strength
 * @disable:		pin disabled / configured as tristate
 * @function:		pinmux selector
 * @inverted:		pin logic is inverted
 */
struct pm8xxx_pin_data {
	unsigned reg;
	int irq;
	u8 power_source;
	u8 direction;
	u8 output_buffer;
	bool output_value;
	u8 bias;
	u8 pull_up_strength;
	u8 output_strength;
	bool disable;
	u8 function;
	bool inverted;
};

struct pm8xxx_mpp {
	struct device *dev;
	struct regmap *regmap;
	struct pinctrl_dev *pctrl;
	struct gpio_chip chip;

	struct pinctrl_desc desc;

	const struct pm8xxx_gpio_data *data;
};

enum pm8xxx_functions {
	PM8XXX_MUX_ext_reg_en,
	PM8XXX_MUX_ext_smps_en,
	PM8XXX_MUX_fclk,
	PM8XXX_MUX_gpio,
	PM8XXX_MUX_kypd_drv,
	PM8XXX_MUX_kypd_sns,
	PM8XXX_MUX_lpa,
	PM8XXX_MUX_lpg,
	PM8XXX_MUX_mp3_clk,
	PM8XXX_MUX_paired,
	PM8XXX_MUX_sleep_clk,
	PM8XXX_MUX_uart,
	PM8XXX_MUX_uim,
	PM8XXX_MUX_upl,
	PM8XXX_MUX__, /* N/A */
	PM8XXX_MUX_COUNT = PM8XXX_MUX__
};

#define FUNCTION(fname, ...) \
	[PM8XXX_MUX_ ## fname] { \
		.name = #fname, \
		.groups = (const char* const[]){ __VA_ARGS__ }, \
		.ngroups = ARRAY_SIZE(((char*[]){ __VA_ARGS__ })), \
	}

#define PINGROUP(id, f1, f2) \
	{ \
		.name = "gpio" #id, \
		.functions = { \
			PM8XXX_MUX_gpio, \
			PM8XXX_MUX_paired, \
			PM8XXX_MUX_ ## f1, \
			PM8XXX_MUX_ ## f2, \
		}, \
	}

static int pm8xxx_read_bank(struct pm8xxx_mpp *pctrl,
			    struct pm8xxx_pin_data *pin, int bank)
{
	unsigned int val = bank << 4;
	int ret;

	ret = regmap_write(pctrl->regmap, pin->reg, val);
	if (ret) {
		dev_err(pctrl->dev,
			"failed to select bank %d\n", bank);
		return ret;
	}

	ret = regmap_read(pctrl->regmap, pin->reg, &val);
	if (ret) {
		dev_err(pctrl->dev,
			"failed to read register %d\n", bank);
		return ret;
	}

	return val;
}

static int pm8xxx_write_bank(struct pm8xxx_mpp *pctrl,
			     struct pm8xxx_pin_data *pin,
			     int bank,
			     u8 val)
{
	int ret;

	val |= PM8XXX_GPIO_WRITE;
	val |= bank << 4;

	ret = regmap_write(pctrl->regmap, pin->reg, val);
	if (ret)
		dev_err(pctrl->dev, "failed to write register\n");

	return ret;
}

static int pm8xxx_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->npins;
}

static const char *pm8xxx_get_group_name(struct pinctrl_dev *pctldev,
					 unsigned group)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->pin_groups[group].name;
}

static int pm8xxx_get_group_pins(struct pinctrl_dev *pctldev,
				 unsigned group,
				 const unsigned **pins,
				 unsigned *num_pins)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*pins = &pctrl->desc.pins[group].number;
	*num_pins = 1;

	return 0;
}

static int pm8xxx_dt_append_subnode_to_map(struct pinctrl_dev *pctldev,
					   struct device_node *np,
					   struct pinctrl_map **map,
					   unsigned *reserved_maps,
					   unsigned *num_maps)
{
	unsigned long cfgs[2];
	unsigned int ncfgs = 0;
	struct property *prop;
	const char *group;
	int reserve;
	int ret;
	u32 val;

	ret = of_property_read_u32(np, "qcom,drive-strength", &val);
	if (!ret) {
		cfgs[ncfgs++] =
			pinconf_to_config_packed(PM8XXX_QCOM_DRIVE_STRENGH,
						 val);
	}

	ret = of_property_read_u32(np, "qcom,pull-up-strength", &val);
	if (!ret) {
		cfgs[ncfgs++] =
			pinconf_to_config_packed(PM8XXX_QCOM_PULL_UP_STRENGTH,
						 val);
	}

	if (ncfgs == 0)
		return 0;

	reserve = of_property_count_strings(np, "pins");
	if (reserve < 0) {
		dev_err(pctldev->dev, "could not parse property pins\n");
		return reserve;
	}

	ret = pinctrl_utils_reserve_map(pctldev, map, reserved_maps,
					num_maps, reserve);
	if (ret < 0)
		return ret;

        of_property_for_each_string(np, "pins", prop, group) {
		ret = pinctrl_utils_add_map_configs(pctldev, map,
						    reserved_maps, num_maps,
						    group,
						    cfgs, ncfgs,
						    PIN_MAP_TYPE_CONFIGS_GROUP);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int pm8xxx_dt_node_to_map(struct pinctrl_dev *pctldev,
				 struct device_node *np_config,
				 struct pinctrl_map **map,
				 unsigned *num_maps)
{
	unsigned reserved_maps;
	struct device_node *np;
	int ret;

	reserved_maps = 0;
	*map = NULL;
	*num_maps = 0;

	for_each_child_of_node(np_config, np) {
		ret = pinconf_generic_dt_subnode_to_map(pctldev, np, map,
				&reserved_maps, num_maps,
				PIN_MAP_TYPE_CONFIGS_GROUP);
		if (ret < 0)
			goto err;

		ret = pm8xxx_dt_append_subnode_to_map(pctldev, np, map,
						      &reserved_maps, num_maps);
		if (ret < 0)
			goto err;
	}
	return 0;

err:
	pinctrl_utils_dt_free_map(pctldev, *map, *num_maps);
	return ret;
}

static const struct pinctrl_ops pm8xxx_pin_datactrl_ops = {
	.get_groups_count	= pm8xxx_get_groups_count,
	.get_group_name		= pm8xxx_get_group_name,
	.get_group_pins         = pm8xxx_get_group_pins,
	.dt_node_to_map		= pm8xxx_dt_node_to_map,
	.dt_free_map		= pinctrl_utils_dt_free_map,
};

static int pm8xxx_get_functions_count(struct pinctrl_dev *pctldev)
{
	return PM8XXX_MUX_COUNT;
}

static const char *pm8xxx_get_function_name(struct pinctrl_dev *pctldev,
					    unsigned function)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_function *fn = &pctrl->data->functions[function];

	if (!pctrl->data->functions)
		return NULL;

	return fn->name;
}

static int pm8xxx_get_function_groups(struct pinctrl_dev *pctldev,
				      unsigned function,
				      const char * const **groups,
				      unsigned * const num_groups)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_function *fn = &pctrl->data->functions[function];

	if (!pctrl->data->functions)
		return -EINVAL;

	*groups = fn->groups;
	*num_groups = fn->ngroups;
	return 0;
}

static int pm8xxx_pinmux_enable(struct pinctrl_dev *pctldev,
				unsigned function,
				unsigned group)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[group].drv_data;
	struct pm8xxx_pingroup *g = &pctrl->data->pin_groups[group];
	u8 val;
	int i;
	
	for (i = 0; i < PM8XXX_NUM_FUNCTIONS; i++) {
		if (g->functions[i] == function)
			break;
	}

	if (WARN_ON(i == PM8XXX_NUM_FUNCTIONS))
		return -EINVAL;

	pin->function = i;
	val = pin->function << 1;

	pm8xxx_write_bank(pctrl, pin, 4, val);

	return 0;
}

static const struct pinmux_ops pm8xxx_pinmux_ops = {
	.get_functions_count	= pm8xxx_get_functions_count,
	.get_function_name	= pm8xxx_get_function_name,
	.get_function_groups	= pm8xxx_get_function_groups,
	.enable			= pm8xxx_pinmux_enable,
};

static int pm8xxx_pin_config_get(struct pinctrl_dev *pctldev,
				 unsigned int offset,
				 unsigned long *config)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset].drv_data;
	unsigned param = pinconf_to_config_param(*config);
	unsigned arg;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		arg = pin->bias == PM8XXX_GPIO_BIAS_NP;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = pin->bias == PM8XXX_GPIO_BIAS_PD;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		arg = pin->bias <= PM8XXX_GPIO_BIAS_PU_1P5_30;
		break;
	case PM8XXX_QCOM_PULL_UP_STRENGTH:
		arg = pin->pull_up_strength;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		arg = pin->disable;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		arg = pin->direction == PM8XXX_GPIO_DIR_IN;
		break;
	case PIN_CONFIG_OUTPUT:
		arg = pin->direction & PM8XXX_GPIO_DIR_OUT && pin->output_value;
		break;
	case PIN_CONFIG_POWER_SOURCE:
		arg = pin->power_source;
		break;
	case PM8XXX_QCOM_DRIVE_STRENGH:
		arg = pin->output_strength;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		arg = pin->output_buffer == PM8XXX_GPIO_PUSH_PULL;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		arg = pin->output_buffer == PM8XXX_GPIO_OPEN_DRAIN;
		break;
	default:
		dev_err(pctrl->dev,
			"unsupported config parameter: %x\n",
			param);
		return -EINVAL;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int resolve_power_source(struct pm8xxx_mpp *pctrl, unsigned arg)
{
	const struct pm8xxx_gpio_data *data = pctrl->data;
	int i;

	for (i = 0; i < data->npower_sources; i++) {
		if (data->power_sources[i] == arg)
			return i;
	}

	dev_err(pctrl->dev, "invalid power source\n");
	return -EINVAL;
}

static int pm8xxx_pin_config_set(struct pinctrl_dev *pctldev,
				 unsigned int offset,
				 unsigned long *configs,
				 unsigned num_configs)
{
	struct pm8xxx_mpp *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset].drv_data;
	unsigned param;
	unsigned arg;
	unsigned i;
	int ret;
	u8 banks = 0;
	u8 val;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			pin->bias = PM8XXX_GPIO_BIAS_NP;
			banks |= BIT(2);
			pin->disable = 0;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			pin->bias = PM8XXX_GPIO_BIAS_PD;
			banks |= BIT(2);
			pin->disable = 0;
			banks |= BIT(3);
			break;
		case PM8XXX_QCOM_PULL_UP_STRENGTH:
			if (arg > PM8XXX_GPIO_BIAS_PU_1P5_30) {
				dev_err(pctrl->dev,
					"invalid pull-up strength\n");
				return -EINVAL;
			}
			pin->pull_up_strength = arg;
			/* FALLTHROUGH */
		case PIN_CONFIG_BIAS_PULL_UP:
			pin->bias = pin->pull_up_strength;
			banks |= BIT(2);
			pin->disable = 0;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
			pin->disable = 1;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			pin->direction = PM8XXX_GPIO_DIR_IN;
			banks |= BIT(1);
			break;
		case PIN_CONFIG_OUTPUT:
			pin->direction = PM8XXX_GPIO_DIR_OUT;
			pin->output_value = !!arg;
			banks |= BIT(1);
			break;
		case PIN_CONFIG_POWER_SOURCE:
			/* Sanity check the power source */
			ret = resolve_power_source(pctrl, arg);
			if (ret < 0)
				return ret;
			pin->power_source = arg;
			banks |= BIT(0);
			break;
		case PM8XXX_QCOM_DRIVE_STRENGH:
			if (arg > PM8XXX_GPIO_STRENGTH_LOW) {
				dev_err(pctrl->dev, "invalid drive strength\n");
				return -EINVAL;
			}
			pin->output_strength = arg;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			pin->output_buffer = PM8XXX_GPIO_PUSH_PULL;
			banks |= BIT(1);
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			pin->output_buffer = PM8XXX_GPIO_OPEN_DRAIN;
			banks |= BIT(1);
			break;
		default:
			dev_err(pctrl->dev,
					"unsupported config parameter: %x\n",
					param);
			return -EINVAL;
		}
	}

	if (banks & BIT(0)) {
		ret = resolve_power_source(pctrl, pin->power_source);
		pm8xxx_write_bank(pctrl, pin, 0, ret << 1);
	}

	if (banks & BIT(1)) {
		val = pin->direction << 2;
		val |= pin->output_buffer << 1;
		val |= pin->output_value;
		pm8xxx_write_bank(pctrl, pin, 1, val);
	}

	if (banks & BIT(2)) {
		val = pin->bias << 1;
		pm8xxx_write_bank(pctrl, pin, 2, val);
	}

	if (banks & BIT(3)) {
		val = pin->output_strength << 2;
		val |= pin->disable;
		pm8xxx_write_bank(pctrl, pin, 3, val);
	}

	if (banks & BIT(4)) {
		val = pin->function << 1;
		pm8xxx_write_bank(pctrl, pin, 4, val);
	}

	if (banks & BIT(5)) {
		val = 0;
		if (!pin->inverted)
			val |= BIT(3);
		pm8xxx_write_bank(pctrl, pin, 5, val);
	}

	return 0;
}

static const struct pinconf_ops pm8xxx_pin_dataconf_ops = {
	.pin_config_group_get = pm8xxx_pin_config_get,
	.pin_config_group_set = pm8xxx_pin_config_set,
};

static struct pinctrl_desc pm8xxx_gpio_desc = {
	.name = "pm8xxx-gpio",
	.pctlops = &pm8xxx_pin_datactrl_ops,
	.pmxops = &pm8xxx_pinmux_ops,
	.confops = &pm8xxx_pin_dataconf_ops,
	.owner = THIS_MODULE,
};

static int pm8xxx_gpio_direction_input(struct gpio_chip *chip,
				       unsigned offset)
{
	struct pm8xxx_mpp *pctrl = container_of(chip, struct pm8xxx_mpp, chip);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset - 1].drv_data;
	u8 val;

	pin->direction = PM8XXX_GPIO_DIR_IN;
	val = pin->direction << 2;

	pm8xxx_write_bank(pctrl, pin, 1, val);

	return 0;
}

static int pm8xxx_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset,
					int value)
{
	struct pm8xxx_mpp *pctrl = container_of(chip, struct pm8xxx_mpp, chip);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset - 1].drv_data;
	u8 val;

	pin->direction = PM8XXX_GPIO_DIR_OUT;
	pin->output_value = !!value;

	val = pin->direction << 2;
	val |= pin->output_buffer << 1;
	val |= pin->output_value;

	pm8xxx_write_bank(pctrl, pin, 1, val);

	return 0;
}

static int pm8xxx_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct pm8xxx_mpp *pctrl = container_of(chip, struct pm8xxx_mpp, chip);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset - 1].drv_data;

	if (pin->direction == PM8XXX_GPIO_DIR_OUT)
		return pin->output_value;
	else
		return pm8xxx_read_irq_status(pin->irq);
}

static void pm8xxx_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct pm8xxx_mpp *pctrl = container_of(gc, struct pm8xxx_mpp, chip);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset - 1].drv_data;
	u8 val;

	pin->output_value = !!value;

	val = pin->direction << 2;
	val |= pin->output_buffer << 1;
	val |= pin->output_value;

	pm8xxx_write_bank(pctrl, pin, 1, val);
}

static int pm8xxx_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct pm8xxx_mpp *pctrl = container_of(chip, struct pm8xxx_mpp, chip);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset - 1].drv_data;

	return pin->irq;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static void pm8xxx_gpio_dbg_show_one(struct seq_file *s,
				  struct pinctrl_dev *pctldev,
				  struct gpio_chip *chip,
				  unsigned offset,
				  unsigned gpio)
{
	struct pm8xxx_mpp *pctrl = container_of(chip, struct pm8xxx_mpp, chip);
	struct pm8xxx_pin_data *pin = pctrl->desc.pins[offset].drv_data;
	struct pm8xxx_pingroup *g = &pctrl->data->pin_groups[offset];
	int function;

	static const char * const directions[] = {
		"off", "out", "in", "both"
	};
	static const char * const sources[] = {
		"bb", "l2", "l3", "l4", "l5", "l6", "l7", "l8", "l11",
		"l14", "l15", "l17", "s3", "s4", "vph"
	};
	static const char * const biases[] = {
		"pull-up 30uA", "pull-up 1.5uA", "pull-up 31.5uA",
		"pull-up 1.5uA + 30uA boost", "pull-down 10uA", "no pull"
	};
	static const char * const buffer_types[] = {
		"push-pull", "open-drain"
	};
	static const char * const strengths[] = {
		"no", "high", "medium", "low"
	};

	seq_printf(s, " gpio%-2d:", offset + 1);
	if (pin->disable) {
		seq_puts(s, " ---");
	} else {
		function = g->functions[pin->function];

		seq_printf(s, " %-4s", directions[pin->direction]);
		seq_printf(s, " %-7s", pctrl->data->functions[function].name);
		seq_printf(s, " %-3s", sources[pin->power_source]);
		seq_printf(s, " %-27s", biases[pin->bias]);
		seq_printf(s, " %-10s", buffer_types[pin->output_buffer]);
		seq_printf(s, " %-4s", pin->output_value ? "high" : "low");
		seq_printf(s, " %-7s", strengths[pin->output_strength]);
		if (pin->inverted)
			seq_puts(s, " inverted");
	}
}

static void pm8xxx_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	unsigned gpio = chip->base;
	unsigned i;

	for (i = 0; i < chip->ngpio; i++, gpio++) {
		pm8xxx_gpio_dbg_show_one(s, NULL, chip, i, gpio);
		seq_puts(s, "\n");
	}
}

#else
#define msm_gpio_dbg_show NULL
#endif

static struct gpio_chip pm8xxx_gpio_template = {
	.direction_input = pm8xxx_gpio_direction_input,
	.direction_output = pm8xxx_gpio_direction_output,
	.get = pm8xxx_gpio_get,
	.set = pm8xxx_gpio_set,
	.to_irq = pm8xxx_gpio_to_irq,
	.dbg_show = pm8xxx_gpio_dbg_show,
	.owner = THIS_MODULE,
};

static int pm8xxx_pin_populate(struct pm8xxx_mpp *pctrl,
			       struct pm8xxx_pin_data *pin)
{
	const struct pm8xxx_gpio_data *data = pctrl->data;
	int val;

	val = pm8xxx_read_bank(pctrl, pin, 0);
	if (val < 0)
		return val;

	pin->power_source = data->power_sources[(val >> 1) & 0x7];

	val = pm8xxx_read_bank(pctrl, pin, 1);
	if (val < 0)
		return val;

	pin->direction = (val >> 2) & 0x3;
	pin->output_buffer = !!(val & BIT(1));
	pin->output_value = val & BIT(0);

	val = pm8xxx_read_bank(pctrl, pin, 2);
	if (val < 0)
		return val;

	pin->bias = (val >> 1) & 0x7;
	if (pin->bias <= PM8XXX_GPIO_BIAS_PU_1P5_30)
		pin->pull_up_strength = pin->bias;
	else
		pin->pull_up_strength = PM8XXX_GPIO_BIAS_PU_30;

	val = pm8xxx_read_bank(pctrl, pin, 3);
	if (val < 0)
		return val;

	pin->output_strength = (val >> 2) & 0x3;
	pin->disable = val & BIT(0);

	val = pm8xxx_read_bank(pctrl, pin, 4);
	if (val < 0)
		return val;

	pin->function = (val >> 1) & 0x7;

	val = pm8xxx_read_bank(pctrl, pin, 5);
	if (val < 0)
		return val;

	pin->inverted = !(val & BIT(3));

	return 0;
}

static const struct pm8xxx_gpio_data pm8018_gpio_data = {
	.npins = 12,
	.power_sources = (int[]) {
		PM8XXX_GPIO_VIN_L4, PM8XXX_GPIO_VIN_L14, PM8XXX_GPIO_VIN_S3,
		PM8XXX_GPIO_VIN_L6, PM8XXX_GPIO_VIN_L2, PM8XXX_GPIO_VIN_L5,
		-1, PM8XXX_GPIO_VIN_VPH,
	},
	.npower_sources = 8,
};

static const struct pm8xxx_gpio_data pm8038_gpio_data = {
	.npins = 12,
	.power_sources = (int[]) {
		PM8XXX_GPIO_VIN_L20, PM8XXX_GPIO_VIN_L11, PM8XXX_GPIO_VIN_L5,
		PM8XXX_GPIO_VIN_L15, PM8XXX_GPIO_VIN_L17, -1, -1,
		PM8XXX_GPIO_VIN_VPH
	},
	.npower_sources = 8,
};

static const struct pm8xxx_gpio_data pm8058_gpio_data = {
	.npins = 40,
	.power_sources = (int[]) {
		PM8XXX_GPIO_VIN_VPH, PM8XXX_GPIO_VIN_S3, PM8XXX_GPIO_VIN_L2,
		PM8XXX_GPIO_VIN_L3,
	},
	.npower_sources = 4,
};

static const struct pm8xxx_gpio_data pm8921_gpio_data = {
	.npins = 44,
	.power_sources = (int[]) {
		PM8XXX_GPIO_VIN_S4, -1, PM8XXX_GPIO_VIN_L15,
		PM8XXX_GPIO_VIN_L17, -1, -1, PM8XXX_GPIO_VIN_VPH,
	},
	.npower_sources = 7,
};

static const struct of_device_id pm8xxx_mpp_of_match[] = {
	{ .compatible = "qcom,pm8018-gpio", .data = &pm8018_gpio_data },
	{ .compatible = "qcom,pm8038-gpio", .data = &pm8038_gpio_data },
	{ .compatible = "qcom,pm8058-gpio", .data = &pm8058_gpio_data },
	{ .compatible = "qcom,pm8917-gpio", .data = &pm8917_gpio_data },
	{ .compatible = "qcom,pm8921-gpio", .data = &pm8921_gpio_data },
	{ },
};
MODULE_DEVICE_TABLE(of, pm8xxx_mpp_of_match);

static int pm8xxx_mpp_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct pm8xxx_pin_data *pin_data;
	struct pinctrl_pin_desc *pins;
	struct pm8xxx_mpp *pctrl;
	int ret;
	int i;

	match = of_match_node(pm8xxx_mpp_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->dev = &pdev->dev;
	pctrl->data = match->data;
	
	pctrl->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pctrl->regmap) {
		dev_err(&pdev->dev, "parent regmap unavailable\n");
		return -ENXIO;
	}

	pctrl->desc = pm8xxx_gpio_desc;
	pctrl->desc.npins = pctrl->data->npins;

	pins = devm_kcalloc(&pdev->dev,
			    pctrl->desc.npins,
			    sizeof(struct pinctrl_pin_desc),
			    GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	pin_data = devm_kcalloc(&pdev->dev,
				pctrl->desc.npins,
				sizeof(struct pm8xxx_pin_data),
				GFP_KERNEL);
	if (!pin_data)
		return -ENOMEM;

	for (i = 0; i < pctrl->desc.npins; i++) {
		pin_data[i].reg = SSBI_REG_ADDR_MPP(i);
		pin_data[i].irq = platform_get_irq(pdev, i);
		if (pin_data[i].irq < 0) {
			dev_err(&pdev->dev,
				"missing interrupts for pin %d\n", i);
			return pin_data[i].irq;
		}

		ret = pm8xxx_pin_populate(pctrl, &pin_data[i]);
		if (ret)
			return ret;

		pins[i].number = i;
		pins[i].drv_data = &pin_data[i];
	}
	pctrl->desc.pins = pins;

	pctrl->pctrl = pinctrl_register(&pctrl->desc, &pdev->dev, pctrl);
	if (!pctrl->pctrl) {
		dev_err(&pdev->dev, "couldn't register pm8xxx gpio driver\n");
		return -ENODEV;
	}

	pctrl->chip = pm8xxx_gpio_template;
	pctrl->chip.base = -1;
	pctrl->chip.dev = &pdev->dev;
	pctrl->chip.of_node = pdev->dev.of_node;
	pctrl->chip.label = dev_name(pctrl->dev);
	pctrl->chip.ngpio = pctrl->data->npins;
	ret = gpiochip_add(&pctrl->chip);
	if (ret) {
		dev_err(&pdev->dev, "failed register gpiochip\n");
		goto unregister_pinctrl;
	}

	ret = gpiochip_add_pin_range(&pctrl->chip,
				     dev_name(pctrl->dev),
				     1, 0, pctrl->chip.ngpio);
	if (ret) {
		dev_err(pctrl->dev, "failed to add pin range\n");
		goto unregister_gpiochip;
	}

	platform_set_drvdata(pdev, pctrl);

	dev_dbg(&pdev->dev, "Qualcomm pm8xxx gpio driver probed\n");

	return 0;

unregister_pinctrl:
	pinctrl_unregister(pctrl->pctrl);

unregister_gpiochip:
	if (gpiochip_remove(&pctrl->chip))
		dev_err(&pdev->dev, "unable to unregister gpiochip\n");

	return ret;
}

static int pm8xxx_mpp_remove(struct platform_device *pdev)
{
	struct pm8xxx_mpp *pctrl = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&pctrl->chip);
	if (ret) {
		dev_err(&pdev->dev, "failed to remove gpiochip\n");
		return ret;
	}

	pinctrl_unregister(pctrl->pctrl);

	return 0;
}

static struct platform_driver pm8xxx_mpp_driver = {
	.driver = {
		.name = "pm8xxx_mpp",
		.owner = THIS_MODULE,
		.of_match_table = pm8xxx_mpp_of_match,
	},
	.probe = pm8xxx_mpp_probe,
	.remove = pm8xxx_mpp_remove,
};

module_platform_driver(pm8xxx_mpp_driver);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm PM8xxx MPP driver");
MODULE_LICENSE("GPL v2");
