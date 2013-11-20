/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Copyright (c) 2013, Sony Mobile Communications, AB.
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
/*
 * Qualcomm PM8xxx Pulse Width Modulation (PWM) driver
 * The HW module is also called LPG (Light Pattern Generator).
 *
 * The module operates as a PWM which can be controlled via a ramp.  Each
 * LPG PWM has:
 *  - a range within the ramp which to operate
 *  - duration of each step within the ramp
 *  - a frequency for the PWM
 *  - pause periods at both ends of the ramp
 *  - options to control ramp operation:
 *    - repeat/oneshot
 *    - reverse
 *    - swap direction for each iteration
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/pwm.h>

#define PM8X_LPG_DRIVER_NAME	"qcom,pm8xxx-pwm"

#define PM8X_PWM_1KHZ			1024
#define PM8X_PWM_M_MAX			7

#define PM8X_REV1_RAMP_CTRL		0x88

#define PM8X_PWM_LUT_STEP_MS_MAX	499
#define PM8X_PWM_MAX_PAUSE_CNT		8191
#define PM8X_PWM_LUT_PAUSE_MS_MAX \
	((PM8X_PWM_MAX_PAUSE_CNT - 1) * PM8X_PWM_LUT_STEP_MS_MAX)

/* LPG registers */
enum pm8xxx_lpg_registers_list {
	PM8X_LPG_REVISION2		= 0x01,

	PM8X_RAMP_CFG			= 0x40,
#define PM8X_RAMP_DIR			BIT(4)
#define PM8X_RAMP_REPEAT		BIT(3)
#define PM8X_RAMP_TOGGLE		BIT(2)
#define PM8X_RAMP_EN_PAUSE_HI		BIT(1)
#define PM8X_RAMP_EN_PAUSE_LO		BIT(0)

	PM8X_PWM_SIZE_CLK		= 0x41,
#define PM8X_PWM_SIZE_SHIFT		4
#define PM8X_PWM_SIZE_MASK		0x30
#define PM8X_PWM_SIZE_9			0x30
#define PM8X_PWM_FREQ_CLK_SEL_MASK	0x03

	PM8X_PWM_FREQ_PDIV_CLK		= 0x42,
#define PM8X_PWM_FREQ_PDIV_SHIFT	5
#define PM8X_PWM_FREQ_PDIV_MASK		0x60
#define PM8X_PWM_FREQ_EXP_MASK		0x07

	PM8X_PWM_TYPE			= 0x43,
#define PM8X_PWM_TYPE_GLTCH_RM		BIT(5)

	PM8X_PWM_VALUE			= 0x44,
#define PM8X_PWM_VALUE_MASK		0x01FF
	PM8X_PWM_VALUE_LSB		= PM8X_PWM_VALUE,
	PM8X_PWM_VALUE_MSB		= 0x45,

	PM8X_PWM_CTRL			= 0x46,
#define PM8X_PWM_CTRL_EN_OUT		0xe0
#define PM8X_PWM_CTRL_SRC_SEL		BIT(2)
#define PM8X_PWM_CTRL_EN_RAMP		BIT(1)

	PM8X_RAMP_CTRL			= 0x47,
#define PM8X_RAMP_CTRL_START		BIT(0)

	PM8X_RAMP_STEP_DUR		= 0x50,
	PM8X_RAMP_STEP_DUR_LSB		= PM8X_RAMP_STEP_DUR,
	PM8X_RAMP_STEP_DUR_MSB		= 0x51,

	PM8X_RAMP_PAUSE_HI_CNT		= 0x52,
	PM8X_RAMP_PAUSE_HI_CNT_LSB	= PM8X_RAMP_PAUSE_HI_CNT,
	PM8X_RAMP_PAUSE_HI_CNT_MSB	= 0x53,

	PM8X_RAMP_PAUSE_LO_CNT		= 0x54,
	PM8X_RAMP_PAUSE_LO_CNT_LSB	= PM8X_RAMP_PAUSE_LO_CNT,
	PM8X_RAMP_PAUSE_LO_CNT_MSB	= 0x55,

	PM8X_RAMP_HI_INDEX		= 0x56,
	PM8X_RAMP_LO_INDEX		= 0x57,
};

#define PM8X_MODE_LUT_CTRL	PM8X_PWM_CTRL_EN_RAMP
#define PM8X_MODE_PWM_CTRL	PM8X_PWM_CTRL_SRC_SEL
#define PM8X_MODE_PWM_OUTPUT	(PM8X_PWM_CTRL_EN_OUT | PM8X_MODE_PWM_CTRL)
#define PM8X_MODE_LUT_OUTPUT	(PM8X_PWM_CTRL_EN_OUT | PM8X_MODE_LUT_CTRL)

enum pm8xxx_lpg_mode {
	PM8X_LPG_MODE_INVALID,
	PM8X_LPG_MODE_PWM,
	PM8X_LPG_MODE_LUT,
};

enum pm8xxx_lpg_state {
	PM8X_LPG_DISABLE,
	PM8X_LPG_ENABLE,
};

struct pm8xxx_lut_params {
	u32 idx_start;
	u32 idx_len;
	u32 idx_cnt;
	u32 pause_hi;
	u32 pause_lo;
	u32 ramp_duration;

#define PM8X_LUT_FLAGS_LOOP		BIT(0)
#define PM8X_LUT_FLAGS_REV		BIT(1)
#define PM8X_LUT_FLAGS_TOGGLE		BIT(2)
	u32 flags;
};

struct pm8xxx_lut_cfg {
	u8 lo_index;
	u8 hi_index;
	u16 step_ms;
	u16 pause_hi_cnt;
	u16 pause_lo_cnt;
	bool ramp_direction;
	bool pattern_repeat;
	bool ramp_toggle;
};

struct pm8xxx_pwm_freq {
	u8 pwm_size;
	int clk;
	int pre_div;
	int pre_div_exp;
};

struct pm8xxx_pwm_cfg {
	bool write_req;
	int pwm_value;
	int pwm_max_value;
	int period;
	struct pm8xxx_pwm_freq freq;
};

struct pm8xxx_lpg_channel {
	u16 addr;
	u8 revision;
	u8 channel_id;
	struct pm8xxx_lpg_chip *lpg;
	enum pm8xxx_lpg_state state;
	enum pm8xxx_lpg_mode mode;
	struct pm8xxx_pwm_cfg pwm_config;
	struct pm8xxx_lut_cfg lut_config;
	struct pm8xxx_lut_params lut_params;
};

#define PM8X_LUT_SIZE 64
struct pm8xxx_lut {
	u16 addr;
	u8 size;
	u16 values[PM8X_LUT_SIZE];
};

#define PM8X_NPWM 8
struct pm8xxx_lpg_chip {
	struct pwm_chip chip;
	struct regmap *regmap;
	struct pm8xxx_lpg_channel channels[PM8X_NPWM];
	struct pm8xxx_lut lut;
	struct mutex mutex;
};

#define to_lpg_chip(_chip) container_of(_chip, struct pm8xxx_lpg_chip, chip)
#define to_lpg_channel(_lpg, _pwm) (&(_lpg)->channels[(_pwm)->hwpwm])

static const unsigned int pm8xxx_clk_t[][3] = {
	{
		1 * (NSEC_PER_SEC / 1024),
		1 * (NSEC_PER_SEC / 32768),
		1 * (NSEC_PER_SEC / 19200000),
	},
	{
		3 * (NSEC_PER_SEC / 1024),
		3 * (NSEC_PER_SEC / 32768),
		3 * (NSEC_PER_SEC / 19200000),
	},
	{
		5 * (NSEC_PER_SEC / 1024),
		5 * (NSEC_PER_SEC / 32768),
		5 * (NSEC_PER_SEC / 19200000),
	},
	{
		6 * (NSEC_PER_SEC / 1024),
		6 * (NSEC_PER_SEC / 32768),
		6 * (NSEC_PER_SEC / 19200000),
	},
};

static int pm8xxx_lpg_write(struct pm8xxx_lpg_channel *chan, u16 addr, u8 val)
{
	return regmap_bulk_write(chan->lpg->regmap, chan->addr + addr, &val, 1);
}

static int pm8xxx_lpg_write_u16(struct pm8xxx_lpg_channel *chan,
		u16 addr, u16 val)
{
	u8 a[] = { val & 0xff, val >> 8 };
	return regmap_bulk_write(chan->lpg->regmap, chan->addr + addr, a, 2);
}

/*
 * PWM Frequency = Clock Frequency / (N * T)
 *	or
 * PWM Period = Clock Period * (N * T)
 *	where
 * N = 2^9 or 2^6 for 9-bit or 6-bit PWM size
 * T = Pre-divide * 2^m, where m = 0..7 (exponent)
 *
 * This is the formula to figure out m for the best pre-divide and clock:
 * (PWM Period / N) = (Pre-divide * Clock Period) * 2^m
 */
static void pm8xxx_lpg_calc_freq(unsigned int period_us,
		struct pm8xxx_pwm_freq *freq)
{
	int		n, m, clk, div;
	int		best_m, best_div, best_clk;
	unsigned int	last_err, cur_err, min_err;
	unsigned int	tmp_p, period_n;

	/* PWM Period / N */
	if (period_us < ((unsigned)(-1) / NSEC_PER_USEC)) {
		period_n = (period_us * NSEC_PER_USEC) >> 6;
		n = 6;
	} else {
		period_n = (period_us >> 9) * NSEC_PER_USEC;
		n = 9;
	}

	min_err = last_err = (unsigned)(-1);
	best_m = 0;
	best_clk = 0;
	best_div = 0;
	for (clk = 0; clk < ARRAY_SIZE(pm8xxx_clk_t[0]); clk++) {
		for (div = 0; div < ARRAY_SIZE(pm8xxx_clk_t); div++) {
			/* period_n = (PWM Period / N) */
			/* tmp_p = (Pre-divide * Clock Period) * 2^m */
			tmp_p = pm8xxx_clk_t[div][clk];
			for (m = 0; m <= PM8X_PWM_M_MAX; m++) {
				if (period_n > tmp_p)
					cur_err = period_n - tmp_p;
				else
					cur_err = tmp_p - period_n;

				if (cur_err < min_err) {
					min_err = cur_err;
					best_m = m;
					best_clk = clk;
					best_div = div;
				}

				if (m && cur_err > last_err)
					/* Break for bigger cur_err */
					break;

				last_err = cur_err;
				tmp_p <<= 1;
			}
		}
	}

	/* Use higher resolution */
	if (best_m >= 3 && n == 6) {
		n += 3;
		best_m -= 3;
	}

	freq->pwm_size = n;
	freq->clk = best_clk;
	freq->pre_div = best_div;
	freq->pre_div_exp = best_m;
}

static u32 pm8xxx_lpg_calc_pwm_value(u8 size, u32 period_us, u32 duty_us)
{
	u32 max_value;
	u32 value;

	/* Figure out pwm_value with overflow handling */
	if (duty_us < 1 << (sizeof(u32) * 8 - size))
		value = (duty_us << size) / period_us;
	else
		value = duty_us / (period_us >> size);
	max_value = (1 << size) - 1;
	if (value > max_value)
		value = max_value;
	return value;
}

static int pm8xxx_lpg_write_table(struct pm8xxx_lpg_chip *lpg,
		struct pm8xxx_lut *lut)
{
	u8 i, vals[128];

	if (lut->size > PM8X_LUT_SIZE)
		return -EOVERFLOW;

	for (i = 0; i < lut->size; ++i) {
		u16 v = lut->values[i];
		vals[i * 2 + 0] = v & 0xff;
		vals[i * 2 + 1] = (v & PM8X_PWM_VALUE_MASK) >> 8;
	}

	return regmap_bulk_write(lpg->regmap, lut->addr, vals, lut->size * 2);
}

static void pm8xxx_lpg_set_period(struct pm8xxx_lpg_channel *chan,
		int period_us)
{
	if (chan->pwm_config.period != period_us) {
		pm8xxx_lpg_calc_freq(period_us, &chan->pwm_config.freq);
		chan->pwm_config.period = period_us;
		chan->pwm_config.write_req = true;
	}
}

static int pm8xxx_lpg_write_pwm_value(struct pm8xxx_lpg_channel *chan)
{
	struct pm8xxx_pwm_cfg	*p = &chan->pwm_config;
	unsigned int		max_pwm_value;

	max_pwm_value = (1 << p->freq.pwm_size) - 1;

	if (p->pwm_value > max_pwm_value)
		p->pwm_value = max_pwm_value;
	if (p->pwm_max_value && (p->pwm_value > p->pwm_max_value))
		p->pwm_value = p->pwm_max_value;

	return pm8xxx_lpg_write_u16(chan, PM8X_PWM_VALUE, p->pwm_value);
}

static int pm8xxx_lpg_configure_freq(struct pm8xxx_lpg_channel *chan)
{
	struct pm8xxx_pwm_freq *p = &chan->pwm_config.freq;
	int rc;
	u8 val;

	if (!chan->pwm_config.write_req)
		return 0;

	val = (p->clk + 1) & PM8X_PWM_FREQ_CLK_SEL_MASK;
	val |= p->pwm_size > 6 ? PM8X_PWM_SIZE_9 : 0;
	rc = pm8xxx_lpg_write(chan, PM8X_PWM_SIZE_CLK, val);
	if (rc)
		return rc;

	val = (p->pre_div << PM8X_PWM_FREQ_PDIV_SHIFT) &
			PM8X_PWM_FREQ_PDIV_MASK;
	val |= p->pre_div_exp & PM8X_PWM_FREQ_EXP_MASK;
	rc = pm8xxx_lpg_write(chan, PM8X_PWM_FREQ_PDIV_CLK, val);
	if (rc)
		return rc;

	chan->pwm_config.write_req = false;
	val = PM8X_PWM_TYPE_GLTCH_RM;
	return pm8xxx_lpg_write(chan, PM8X_PWM_TYPE, val);
}

static int pm8xxx_lpg_set_lut_state(struct pm8xxx_lpg_channel *chan,
				enum pm8xxx_lpg_state state)
{
	struct {
		u16 addr;
		u8 value;
	} control[] = {
		{
			chan->addr + PM8X_RAMP_CTRL,
			PM8X_RAMP_CTRL_START
		},
		{
			chan->lpg->lut.addr + PM8X_REV1_RAMP_CTRL,
			BIT(chan->channel_id)
		}

	};
	u8 value;
	int rc;

	rc = pm8xxx_lpg_write(chan, PM8X_PWM_CTRL, state == PM8X_LPG_ENABLE ?
			PM8X_MODE_LUT_OUTPUT : PM8X_MODE_LUT_CTRL);
	if (rc)
		return rc;
	chan->state = state;

	if (chan->revision >= ARRAY_SIZE(control)) {
		dev_err(chan->lpg->chip.dev, "invalid LPG revision %d\n",
				chan->revision);
		return -EINVAL;
	}

	value = control[chan->revision].value;

	return regmap_bulk_write(chan->lpg->regmap,
			control[chan->revision].addr, &value, 1);
}

static int pm8xxx_lpg_set_pwm_state(struct pm8xxx_lpg_channel *chan,
					enum pm8xxx_lpg_state state)
{
	int rc;

	rc = pm8xxx_lpg_write(chan, PM8X_PWM_CTRL, state == PM8X_LPG_ENABLE ?
			PM8X_MODE_PWM_OUTPUT : PM8X_MODE_PWM_CTRL);
	if (rc)
		return rc;
	chan->state = state;

	/*
	 * Due to LPG hardware bug in PWM mode, we have to write
	 * PWM values one more time.
	 */
	if (state == PM8X_LPG_ENABLE)
		return pm8xxx_lpg_write_pwm_value(chan);

	return rc;
}

static int pm8xxx_lpg_write_pwm_config(struct pm8xxx_lpg_channel *chan)
{
	int rc;

	rc = pm8xxx_lpg_write_pwm_value(chan);
	if (rc)
		return rc;

	rc = pm8xxx_lpg_configure_freq(chan);
	if (rc)
		return rc;

	return 0;
}

static int pm8xxx_lpg_update_pwm_config(struct pm8xxx_lpg_channel *chan,
		int duty_us, int period_us)
{
	struct pm8xxx_pwm_cfg *pwm_config;
	struct pm8xxx_pwm_freq *freq;

	pwm_config = &chan->pwm_config;
	freq = &pwm_config->freq;

	pm8xxx_lpg_set_period(chan, period_us);
	pwm_config->pwm_value = pm8xxx_lpg_calc_pwm_value(freq->pwm_size,
			period_us, duty_us);
	dev_dbg(chan->lpg->chip.dev, "pwm channel %u: (%d/%d) = %u\n",
			chan->channel_id, duty_us, period_us,
			pwm_config->pwm_value);

	return 0;
}

static int pm8xxx_lpg_write_lut_config(struct pm8xxx_lpg_channel *chan)
{
	struct pm8xxx_lut_cfg *cfg;
	u8 value;
	int rc;

	cfg = &chan->lut_config;

	value = 0;
	value |= (cfg->pause_hi_cnt) ? PM8X_RAMP_EN_PAUSE_HI : 0;
	value |= (cfg->pause_lo_cnt) ? PM8X_RAMP_EN_PAUSE_LO : 0;
	value |= (cfg->ramp_toggle) ? PM8X_RAMP_TOGGLE : 0;
	value |= (cfg->pattern_repeat) ? PM8X_RAMP_REPEAT : 0;
	value |= (cfg->ramp_direction) ? PM8X_RAMP_DIR : 0;
	rc = pm8xxx_lpg_write(chan, PM8X_RAMP_CFG, value);
	if (rc)
		return rc;
	rc = pm8xxx_lpg_configure_freq(chan);
	if (rc)
		return rc;
	rc = pm8xxx_lpg_write(chan, PM8X_PWM_CTRL, PM8X_MODE_LUT_CTRL);
	if (rc)
		return rc;
	chan->state = PM8X_LPG_DISABLE;

	rc = pm8xxx_lpg_write_u16(chan, PM8X_RAMP_STEP_DUR, cfg->step_ms);
	if (rc)
		return rc;

	rc = pm8xxx_lpg_write_u16(chan, PM8X_RAMP_PAUSE_HI_CNT,
			cfg->pause_hi_cnt);
	if (rc)
		return rc;

	rc = pm8xxx_lpg_write_u16(chan, PM8X_RAMP_PAUSE_LO_CNT,
			cfg->pause_lo_cnt);
	if (rc)
		return rc;

	rc = pm8xxx_lpg_write(chan, PM8X_RAMP_HI_INDEX, cfg->hi_index);
	if (rc)
		return rc;

	rc = pm8xxx_lpg_write(chan, PM8X_RAMP_LO_INDEX, cfg->lo_index);
	if (rc)
		return rc;
	return rc;
}

static int pm8xxx_lpg_update_lut_config(struct pm8xxx_lpg_channel *chan,
		struct pm8xxx_lut_params *p)
{
	struct pm8xxx_lut_cfg *cfg;
	u32 step_ms;
	u32 nsteps;

	cfg = &chan->lut_config;

	nsteps = p->idx_cnt;
	cfg->lo_index = p->idx_start;
	cfg->hi_index = p->idx_start + (nsteps - 1);

	step_ms = p->ramp_duration / nsteps;
	if (step_ms < 1)
		step_ms = 1;
	if (step_ms > PM8X_PWM_LUT_STEP_MS_MAX)
		step_ms = PM8X_PWM_LUT_STEP_MS_MAX;

	if (p->pause_lo) {
		cfg->pause_lo_cnt = p->pause_lo / step_ms + 1;
		if (cfg->pause_lo_cnt > PM8X_PWM_MAX_PAUSE_CNT)
			cfg->pause_lo_cnt = PM8X_PWM_MAX_PAUSE_CNT;
	}

	if (p->pause_hi) {
		cfg->pause_hi_cnt = p->pause_hi / step_ms + 1;
		if (cfg->pause_hi_cnt > PM8X_PWM_MAX_PAUSE_CNT)
			cfg->pause_hi_cnt = PM8X_PWM_MAX_PAUSE_CNT;
	}

	cfg->step_ms = (step_ms * PM8X_PWM_1KHZ) / 1000;
	cfg->ramp_direction	=  !(p->flags & PM8X_LUT_FLAGS_REV);
	cfg->pattern_repeat	= !!(p->flags & PM8X_LUT_FLAGS_LOOP);
	cfg->ramp_toggle	= !!(p->flags & PM8X_LUT_FLAGS_TOGGLE);

	return 0;
}

static int pm8xxx_lpg_set_state(struct pm8xxx_lpg_channel *chan,
		enum pm8xxx_lpg_state state)
{
	switch (chan->mode) {
	case PM8X_LPG_MODE_LUT:
		return pm8xxx_lpg_set_lut_state(chan, state);
	case PM8X_LPG_MODE_PWM:
		return pm8xxx_lpg_set_pwm_state(chan, state);
	default:
		break;
	}
	return -EINVAL;
}

static int pm8xxx_lpg_update_config(struct pm8xxx_lpg_channel *chan,
		int duty_us, int period_us)
{
	enum pm8xxx_lpg_state state;
	int rc = 0;

	state = chan->state;
	dev_info(chan->lpg->chip.dev, "update config; state = %d\n", state);
	switch (chan->mode) {
	case PM8X_LPG_MODE_LUT:
		pm8xxx_lpg_set_period(chan, period_us);
		chan->lut_params.idx_cnt =
				chan->lut_params.idx_len * duty_us / period_us;
		rc = pm8xxx_lpg_update_lut_config(chan, &chan->lut_params);
		break;
	case PM8X_LPG_MODE_PWM:
		rc = pm8xxx_lpg_update_pwm_config(chan, duty_us, period_us);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	if (rc)
		return rc;
	switch (chan->mode) {
	case PM8X_LPG_MODE_LUT:
		rc = pm8xxx_lpg_write_lut_config(chan);
		break;
	case PM8X_LPG_MODE_PWM:
		rc = pm8xxx_lpg_write_pwm_config(chan);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	if (rc)
		return rc;

	return pm8xxx_lpg_set_state(chan, state);
}

static int pm8xxx_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pm8xxx_lpg_channel *chan;
	struct pm8xxx_lpg_chip *lpg;

	lpg = to_lpg_chip(chip);
	chan = to_lpg_channel(lpg, pwm);

	mutex_lock(&lpg->mutex);
	if (chan->mode == PM8X_LPG_MODE_INVALID) {
		mutex_unlock(&lpg->mutex);
		return -EINVAL;
	}
	mutex_unlock(&lpg->mutex);

	return 0;
}

static void pm8xxx_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pm8xxx_lpg_channel *chan;
	struct pm8xxx_lpg_chip *lpg;

	lpg = to_lpg_chip(chip);
	chan = to_lpg_channel(lpg, pwm);

	mutex_lock(&lpg->mutex);
	pm8xxx_lpg_set_state(chan, PM8X_LPG_DISABLE);
	mutex_unlock(&lpg->mutex);
}

static int pm8xxx_pwm_config(struct pwm_chip *chip,
		struct pwm_device *pwm, int duty_ns, int period_ns)
{
	struct pm8xxx_lpg_channel *chan;
	struct pm8xxx_lpg_chip *lpg;
	int rc;

	lpg = to_lpg_chip(chip);
	chan = to_lpg_channel(lpg, pwm);

	mutex_lock(&lpg->mutex);
	rc = pm8xxx_lpg_update_config(chan, duty_ns, period_ns);
	mutex_unlock(&lpg->mutex);

	return rc;
}

static int pm8xxx_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pm8xxx_lpg_channel *chan;
	struct pm8xxx_lpg_chip *lpg;
	int rc;

	lpg = to_lpg_chip(chip);
	chan = to_lpg_channel(lpg, pwm);

	mutex_lock(&lpg->mutex);

	if (chan->pwm_config.period == 0) {
		rc = pm8xxx_lpg_update_config(chan, 0, pwm->period);
		if (rc)
			goto out;
	}

	rc = pm8xxx_lpg_set_state(chan, PM8X_LPG_ENABLE);

out:
	mutex_unlock(&lpg->mutex);
	return rc;
}

static void pm8xxx_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pm8xxx_lpg_channel *chan;
	struct pm8xxx_lpg_chip *lpg;
	int rc = 0;

	lpg = to_lpg_chip(chip);
	chan = to_lpg_channel(lpg, pwm);

	mutex_lock(&lpg->mutex);
	rc = pm8xxx_lpg_set_state(chan, PM8X_LPG_DISABLE);
	if (rc)
		dev_err(chip->dev, "failed to disable PWM channel: %d\n",
				chan->channel_id);
	mutex_unlock(&lpg->mutex);
}

/**
  * pm8xxx_pwm_configure_pause() - configure high & low pauses for LPG mode
  * @chip: PWM chip
  * @pwm: PWM device
  * @hi: duration to pause at ramp end in ms
  * @lo: duration to pause at ramp start in ms
  *
  * Passing 0 for both hi and lo disables ramp functionality.
  *
  */
int pm8xxx_pwm_configure_pause(struct pwm_chip *chip, struct pwm_device *pwm,
		unsigned int hi, unsigned int lo)
{
	struct pm8xxx_lpg_channel *chan;
	struct pm8xxx_lpg_chip *lpg;
	int rc = 0;

	lpg = to_lpg_chip(chip);
	chan = to_lpg_channel(lpg, pwm);
	dev_info(lpg->chip.dev, "pause = (%u, %u)\n", lo, hi);
	mutex_lock(&lpg->mutex);

	chan->lut_params.pause_hi = hi;
	chan->lut_params.pause_lo = lo;

	switch (chan->mode) {
	case PM8X_LPG_MODE_LUT:
		if (!hi && !lo)
			chan->mode = PM8X_LPG_MODE_PWM;
		break;
	case PM8X_LPG_MODE_PWM:
		if (hi || lo) {
			chan->mode = PM8X_LPG_MODE_LUT;
			rc = pm8xxx_lpg_update_lut_config(chan,
					&chan->lut_params);
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&lpg->mutex);

	return rc;
}
EXPORT_SYMBOL_GPL(pm8xxx_pwm_configure_pause);

static const struct pwm_ops pm8xxx_pwm_ops = {
	.request = pm8xxx_pwm_request,
	.free = pm8xxx_pwm_free,
	.config = pm8xxx_pwm_config,
	.enable = pm8xxx_pwm_enable,
	.disable = pm8xxx_pwm_disable,
	.owner = THIS_MODULE,
};

static int pm8xxx_lpg_parse_lut_dt(struct platform_device *pdev,
		struct device_node *node, struct pm8xxx_lpg_channel *chan)
{
	struct pm8xxx_lut_params p;
	int i, rc;
	const struct {
		const char *name;
		u32 *ptr;
		u32 def, min, max;
	} opt_u32[] = {
		{
			"qcom,ramp-duration", &p.ramp_duration,
			0, 1, PM8X_PWM_LUT_STEP_MS_MAX * PM8X_LUT_SIZE
		},
		{
			"qcom,ramp-pause-top", &p.pause_hi,
			500, 0, PM8X_PWM_LUT_PAUSE_MS_MAX
		},
		{
			"qcom,ramp-pause-bottom", &p.pause_lo,
			500, 0, PM8X_PWM_LUT_PAUSE_MS_MAX
		},
	};
	const struct {
		const char *name;
		u32 bit;
	} opt_bool[] = {
		{ "qcom,ramp-reverse", PM8X_LUT_FLAGS_REV },
		{ "qcom,ramp-pingpong", PM8X_LUT_FLAGS_TOGGLE },
		{ "qcom,ramp-repeat", PM8X_LUT_FLAGS_LOOP },
	};

	rc = of_property_read_u32_index(node, "qcom,lut-range",
			0, &p.idx_start);
	if (rc) {
		dev_err(&pdev->dev, "offset missing for 'qcom,lut-range'\n");
		return rc;
	}
	rc = of_property_read_u32_index(node, "qcom,lut-range", 1, &p.idx_len);
	if (rc) {
		dev_err(&pdev->dev, "length missing for 'qcom,lut-range'\n");
		return rc;
	}
	if (p.idx_start + p.idx_len > PM8X_LUT_SIZE) {
		dev_err(&pdev->dev, "invalid range for 'qcom,lut-range'\n");
		return -EINVAL;
	}
	p.idx_cnt = p.idx_len;

	for (i = 0; i < ARRAY_SIZE(opt_u32); ++i) {
		u32 val;

		val = opt_u32[i].def;
		rc = of_property_read_u32(node, opt_u32[i].name, &val);
		if (rc && rc != -EINVAL) {
			dev_err(&pdev->dev, "invalid parameter for '%s'\n",
					opt_u32[i].name);
			return rc;
		} else if (val < opt_u32[i].min || val > opt_u32[i].max) {
			dev_err(&pdev->dev,
					"value out of range [%u..%u] in '%s'\n",
					opt_u32[i].min, opt_u32[i].max,
					opt_u32[i].name);
			val = opt_u32[i].def;
		}
		*opt_u32[i].ptr = val;
		rc = 0;
	}

	p.flags = 0;
	for (i = 0; i < ARRAY_SIZE(opt_bool); ++i) {
		bool val;

		val = of_property_read_bool(node, opt_bool[i].name);
		if (val)
			p.flags |= opt_bool[i].bit;
		rc = 0;
	}

	rc = pm8xxx_lpg_update_lut_config(chan, &p);
	chan->lut_params = p;

	return rc;
}

/* Fill in lpg device elements based on values found in device tree. */
static int pm8xxx_lpg_parse_dt(struct platform_device *pdev,
		struct pm8xxx_lpg_chip *lpg)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child;
	u32 res;
	int len;
	int rc = 0;

	rc = of_property_read_u32_array(node, "reg", &res, 1);
	if (rc) {
		dev_err(&pdev->dev, "LUT address missing\n");
		return rc;
	}
	lpg->lut.addr = res;

	if (of_find_property(node, "qcom,lut", &len)) {
		if (len & 0x1 || len > 128) {
			dev_err(&pdev->dev, "invalid size for 'qcom,lut'\n");
			return -EINVAL;
		}
		len /= sizeof(u16);
		rc = of_property_read_u16_array(node,
				"qcom,lut", lpg->lut.values, len);
		if (rc) {
			dev_err(&pdev->dev, "unable to read 'qcom,lut'\n");
			return rc;
		}
		lpg->lut.size = len;

		rc = pm8xxx_lpg_write_table(lpg, &lpg->lut);
		if (rc) {
			dev_err(&pdev->dev, "unable to write LUT @ %x\n", res);
			return rc;
		}
	}

	for_each_child_of_node(node, child) {
		struct pm8xxx_lpg_channel *chan;
		u32 channel;
		u8 revision;
		u32 addr;

		rc = of_property_read_u32(child, "reg", &addr);
		if (rc) {
			dev_err(&pdev->dev, "missing base address\n");
			return rc;
		}

		rc = regmap_bulk_read(lpg->regmap,
				addr + PM8X_LPG_REVISION2, &revision, 1);
		if (rc || revision > 1) {
			if (rc)
				dev_err(&pdev->dev,
						"unable to read LPG revision");
			else
				dev_err(&pdev->dev,
						"unsupported LPG revision %d\n",
						revision);
			return rc;
		}

		rc = of_property_read_u32(child, "qcom,channel-id", &channel);
		if (rc) {
			dev_err(&pdev->dev, "missing `qcom,channel-id`\n");
			return rc;
		}
		if (channel >= PM8X_NPWM) {
			dev_err(&pdev->dev, "invalid `qcom,channel-id`\n");
			return rc;
		}

		chan = &lpg->channels[channel];
		chan->lpg = lpg;
		chan->addr = addr;
		chan->channel_id = channel;
		chan->revision = revision;

		rc = of_property_read_u32(child, "qcom,pwm-max-value",
					&chan->pwm_config.pwm_max_value);
		if (rc)
			chan->pwm_config.pwm_max_value = 0;

		rc = pm8xxx_lpg_parse_lut_dt(pdev, child, chan);
		if (rc)
			return rc;
		chan->mode = PM8X_LPG_MODE_PWM;
		rc = pm8xxx_lpg_set_state(chan, PM8X_LPG_DISABLE);
		if (rc) {
			dev_err(&pdev->dev, "unable to set mode\n");
			chan->mode = PM8X_LPG_MODE_INVALID;
			return rc;
		}
		dev_info(&pdev->dev, "added channel %d\n", chan->channel_id);
	}

	return 0;
}

static ssize_t sync_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct pm8xxx_lpg_chip *lpg;
	u16 rev0_addrs[PM8X_NPWM];
	u8 rev1_bits;
	int nrev0;
	int i, rc;

	lpg = dev_get_drvdata(dev);

	mutex_lock(&lpg->mutex);

	nrev0 = 0;
	rev1_bits = 0;
	for (i = 0; i < PM8X_NPWM; ++i) {
		struct pm8xxx_lpg_channel *chan;
		u8 value;

		chan = &lpg->channels[i];
		if (chan->mode != PM8X_LPG_MODE_LUT)
			continue;
		if (chan->state != PM8X_LPG_ENABLE)
			continue;
		if (chan->revision == 0)
			rev0_addrs[nrev0++] = chan->addr + PM8X_RAMP_CTRL;
		else if (chan->revision == 1)
			rev1_bits |= BIT(chan->channel_id);
		else
			continue;

		value = PM8X_MODE_LUT_CTRL;
		rc = regmap_bulk_write(lpg->regmap,
				chan->addr + PM8X_PWM_CTRL, &value, 1);
		if (rc)
			goto err;
		value = PM8X_MODE_LUT_OUTPUT;
		rc = regmap_bulk_write(lpg->regmap,
				chan->addr + PM8X_PWM_CTRL, &value, 1);
		if (rc)
			goto err;
	}

	for (i = 0; i < nrev0; ++i) {
		u8 value = PM8X_RAMP_CTRL_START;
		rc = regmap_bulk_write(lpg->regmap, rev0_addrs[i], &value, 1);
		if (rc)
			goto err;
	}

	if (rev1_bits) {
		rc = regmap_bulk_write(lpg->regmap,
				lpg->lut.addr + PM8X_REV1_RAMP_CTRL,
				&rev1_bits, 1);
		if (rc)
			goto err;
	}

	mutex_unlock(&lpg->mutex);

	return len;
err:
	mutex_unlock(&lpg->mutex);
	return rc;
}

static DEVICE_ATTR_WO(sync);
static struct attribute *pm8xxx_attrs[] = {
	&dev_attr_sync.attr,
	NULL,
};
static const struct attribute_group pm8xxx_attr_group = {
	.attrs = pm8xxx_attrs,
};

static int pm8xxx_lpg_probe(struct platform_device *pdev)
{
	struct pm8xxx_lpg_chip *lpg;
	int rc;

	lpg = devm_kzalloc(&pdev->dev, sizeof(*lpg), GFP_KERNEL);
	if (lpg == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, lpg);

	lpg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!lpg->regmap) {
		dev_err(&pdev->dev, "unable to get regmap\n");
		return -EINVAL;
	}

	rc = pm8xxx_lpg_parse_dt(pdev, lpg);
	if (rc)
		return rc;

	mutex_init(&lpg->mutex);

	lpg->chip.dev = &pdev->dev;
	lpg->chip.ops = &pm8xxx_pwm_ops;
	lpg->chip.base = -1;
	lpg->chip.npwm = PM8X_NPWM;
	rc = pwmchip_add(&lpg->chip);
	if (rc) {
		dev_err(&pdev->dev, "failed to add PWM chip\n");
		return rc;
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &pm8xxx_attr_group);
	if (rc) {
		pwmchip_remove(&lpg->chip);
		dev_err(&pdev->dev, "failed to sysfs group\n");
		return rc;
	}

	return 0;
}

static int pm8xxx_lpg_remove(struct platform_device *pdev)
{
	struct pm8xxx_lpg_chip *lpg;

	lpg = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &pm8xxx_attr_group);

	return pwmchip_remove(&lpg->chip);
}

static const struct of_device_id pm8xxx_lpg_match_table[] = {
	{ .compatible = PM8X_LPG_DRIVER_NAME, },
	{}
};

static struct platform_driver pm8xxx_lpg_driver = {
	.driver		= {
		.owner = THIS_MODULE,
		.name = PM8X_LPG_DRIVER_NAME,
		.of_match_table = pm8xxx_lpg_match_table,
	},
	.probe		= pm8xxx_lpg_probe,
	.remove		= pm8xxx_lpg_remove,
};

module_platform_driver(pm8xxx_lpg_driver);
MODULE_DESCRIPTION("PM8xxx PMIC PWM driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" PM8X_LPG_DRIVER_NAME);
