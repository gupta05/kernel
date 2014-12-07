/* Copyright (c) 2014, Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver is for the multi-block Switch-Mode Battery Charger and Boost
 * (SMBB) hardware, found in Qualcomm PM8941 PMICs.  The charger is an
 * integrated, single-cell lithium-ion battery charger.
 *
 * Sub-components:
 *  - Charger core
 *  - Buck
 *  - DC charge-path
 *  - USB charge-path
 *  - Battery interface
 *  - Boost (not implemented)
 *  - Misc
 *  - HF-Buck
 */

#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define SMBB_CHG_VMAX          0x040
#define SMBB_CHG_VSAFE         0x041
#define SMBB_CHG_CFG           0x043
#define SMBB_CHG_IMAX          0x044
#define SMBB_CHG_ISAFE         0x045
#define SMBB_CHG_VIN_MIN       0x047
#define SMBB_CHG_CTRL          0x049
#define CTRL_EN BIT(7)
#define SMBB_CHG_VBAT_WEAK     0x052
#define SMBB_CHG_IBAT_TERM_CHG 0x05b
#define IBAT_TERM_CHG_IEOC     BIT(7)
#define IBAT_TERM_CHG_IEOC_BMS BIT(7)
#define IBAT_TERM_CHG_IEOC_CHG 0
#define SMBB_CHG_VBAT_DET      0x05d
#define SMBB_CHG_TCHG_MAX_EN   0x060
#define TCHG_MAX_EN    BIT(7)
#define SMBB_CHG_WDOG_TIME     0x062
#define SMBB_CHG_WDOG_EN       0x065
#define WDOG_EN        BIT(7)

#define SMBB_BUCK_REG_MODE     0x174
#define BUCK_REG_MODE          BIT(0)
#define BUCK_REG_MODE_VBAT     BIT(0)
#define BUCK_REG_MODE_VSYS     0

#define SMBB_BAT_PRES_STATUS   0x208
#define PRES_STATUS_BAT_PRES   BIT(7)
#define SMBB_BAT_TEMP_STATUS   0x209
#define TEMP_STATUS_OK BIT(7)
#define TEMP_STATUS_HOT        BIT(6)
#define SMBB_BAT_BTC_CTRL      0x249
#define BTC_CTRL_COMP_EN       BIT(7)
#define BTC_CTRL_COLD_EXT      BIT(1)
#define BTC_CTRL_HOT_EXT_N     BIT(0)

#define SMBB_USB_IMAX          0x344
#define SMBB_USB_ENUM_TIMER_STOP 0x34e
#define ENUM_TIMER_STOP BIT(0)
#define SMBB_USB_SEC_ACCESS    0x3d0
#define SEC_ACCESS_MAGIC       0xa5
#define SMBB_USB_REV_BST       0x3ed
#define REV_BST_CHG_GONE       BIT(7)

#define SMBB_DC_IMAX           0x444

#define SMBB_MISC_REV2         0x601
#define SMBB_MISC_BOOT_DONE    0x642
#define BOOT_DONE BIT(7)

#define STATUS_USBIN_VALID     BIT(0) /* USB connection is valid */
#define STATUS_DCIN_VALID      BIT(1) /* DC connection is valid */
#define STATUS_BAT_HOT         BIT(2) /* Battery temp 1=Hot, 0=Cold */
#define STATUS_BAT_OK          BIT(3) /* Battery temp OK */
#define STATUS_BAT_PRESENT     BIT(4) /* Battery is present */
#define STATUS_CHG_DONE                BIT(5) /* Charge cycle is complete */
#define STATUS_CHG_TRKL                BIT(6) /* Trickle charging */
#define STATUS_CHG_FAST                BIT(7) /* Fast charging */
#define STATUS_CHG_GONE                BIT(8) /* No charger is connected */

enum pm8941_attr {
       ATTR_BAT_ISAFE,
       ATTR_BAT_IMAX,
       ATTR_USBIN_IMAX,
       ATTR_DCIN_IMAX,
       ATTR_BAT_VSAFE,
       ATTR_BAT_VMAX,
       ATTR_BAT_VMIN,
       ATTR_CHG_VDET,
       ATTR_VIN_MIN,
       _ATTR_CNT,
};

struct pm8941_charger {
       unsigned int revision;
       unsigned int addr;
       struct device *dev;

       bool dc_disabled;
       bool jeita_ext_temp;
       unsigned long status;
       struct mutex statlock;

       unsigned int attr[_ATTR_CNT];

       struct power_supply usb_psy;
       struct power_supply dc_psy;
       struct power_supply bat_psy;
       struct regmap *regmap;
};

static int pm8941_vbat_weak_fn(unsigned int index)
{
       return 2100000 + index * 100000;
}

static int pm8941_vin_fn(unsigned int index)
{
       if (index > 42)
               return 5600000 + (index - 43) * 200000;
       return 3400000 + index * 50000;
}

static int pm8941_vmax_fn(unsigned int index)
{
       return 3240000 + index * 10000;
}

static int pm8941_vbat_det_fn(unsigned int index)
{
       return 3240000 + index * 20000;
}

static int pm8941_imax_fn(unsigned int index)
{
       if (index < 2)
               return 100000 + index * 50000;
       return index * 100000;
}

static int pm8941_bat_imax_fn(unsigned int index)
{
       return index * 50000;
}

static unsigned int pm8941_hw_lookup(unsigned int val, int (*fn)(unsigned int))
{
       unsigned int widx;
       unsigned int sel;

       for (widx = sel = 0; (*fn)(widx) <= val; ++widx)
               sel = widx;

       return sel;
}

static const struct pm8941_charger_attr {
       const char *name;
       unsigned int reg;
       unsigned int safe_reg;
       unsigned int max;
       unsigned int min;
       unsigned int fail_ok;
       int (*hw_fn)(unsigned int);
} pm8941_charger_attrs[] = {
       [ATTR_BAT_ISAFE] = {
               .name = "fast-charge-safe-current",
               .reg = SMBB_CHG_ISAFE,
               .max = 3000000,
               .min = 200000,
               .hw_fn = pm8941_bat_imax_fn,
               .fail_ok = 1,
       },
       [ATTR_BAT_IMAX] = {
               .name = "battery-charge-control-limit",
               .reg = SMBB_CHG_IMAX,
               .safe_reg = SMBB_CHG_ISAFE,
               .max = 3000000,
               .min = 200000,
               .hw_fn = pm8941_bat_imax_fn,
       },
       [ATTR_USBIN_IMAX] = {
               .name = "usb-charge-control-limit",
               .reg = SMBB_USB_IMAX,
               .max = 2500000,
               .min = 100000,
               .hw_fn = pm8941_imax_fn,
       },
       [ATTR_DCIN_IMAX] = {
               .name = "dc-charge-control-limit",
               .reg = SMBB_DC_IMAX,
               .max = 2500000,
               .min = 100000,
               .hw_fn = pm8941_imax_fn,
       },
       [ATTR_BAT_VSAFE] = {
               .name = "fast-charge-safe-voltage",
               .reg = SMBB_CHG_VSAFE,
               .max = 5000000,
               .min = 3240000,
               .hw_fn = pm8941_vmax_fn,
               .fail_ok = 1,
       },
       [ATTR_BAT_VMAX] = {
               .name = "fast-charge-high-watermark",
               .reg = SMBB_CHG_VMAX,
               .safe_reg = SMBB_CHG_VSAFE,
               .max = 5000000,
               .min = 3240000,
               .hw_fn = pm8941_vmax_fn,
       },
       [ATTR_BAT_VMIN] = {
               .name = "fast-charge-low-watermark",
               .reg = SMBB_CHG_VBAT_WEAK,
               .max = 3600000,
               .min = 2100000,
               .hw_fn = pm8941_vbat_weak_fn,
       },
       [ATTR_CHG_VDET] = {
               .name = "auto-recharge-low-watermark",
               .reg = SMBB_CHG_VBAT_DET,
               .max = 5000000,
               .min = 3240000,
               .hw_fn = pm8941_vbat_det_fn,
       },
       [ATTR_VIN_MIN] = {
               .name = "minimum-input-voltage",
               .reg = SMBB_CHG_VIN_MIN,
               .max = 9600000,
               .min = 4200000,
               .hw_fn = pm8941_vin_fn,
       },
};

static int pm8941_charger_attr_write(struct pm8941_charger *chg,
               enum pm8941_attr which, unsigned int val)
{
       const struct pm8941_charger_attr *prop;
       unsigned int wval;
       unsigned int out;
       int rc;

       prop = &pm8941_charger_attrs[which];

       if (val > prop->max || val < prop->min) {
               dev_err(chg->dev, "value out of range for %s [%u:%u]\n",
                       prop->name, prop->min, prop->max);
               return -EINVAL;
       }

       if (prop->safe_reg) {
               rc = regmap_read(chg->regmap,
                               chg->addr + prop->safe_reg, &wval);
               if (rc) {
                       dev_err(chg->dev,
                               "unable to read safe value for '%s'\n",
                               prop->name);
                       return rc;
               }

               wval = prop->hw_fn(wval);

               if (val > wval) {
                       dev_warn(chg->dev,
                               "%s above safe value, clamping at %u\n",
                               prop->name, wval);
                       val = wval;
               }
       }

       wval = pm8941_hw_lookup(val, prop->hw_fn);

       rc = regmap_write(chg->regmap, chg->addr + prop->reg, wval);
       if (rc) {
               dev_err(chg->dev, "unable to update %s", prop->name);
               return rc;
       }
       out = prop->hw_fn(wval);
       if (out != val) {
               dev_warn(chg->dev,
                       "%s inaccurate, rounded to %u\n",
                       prop->name, out);
       }

       dev_dbg(chg->dev, "%s <= %d\n", prop->name, out);

       chg->attr[which] = out;

       return 0;
}

static int pm8941_charger_attr_read(struct pm8941_charger *chg,
               enum pm8941_attr which)
{
       const struct pm8941_charger_attr *prop;
       unsigned int val;
       int rc;

       prop = &pm8941_charger_attrs[which];

       rc = regmap_read(chg->regmap, chg->addr + prop->reg, &val);
       if (rc) {
               dev_err(chg->dev, "failed to read %s\n", prop->name);
               return rc;
       }
       val = prop->hw_fn(val);
       dev_dbg(chg->dev, "%s => %d\n", prop->name, val);

       chg->attr[which] = val;

       return 0;
}

static int pm8941_charger_attr_parse(struct pm8941_charger *chg,
               enum pm8941_attr which)
{
       const struct pm8941_charger_attr *prop;
       unsigned int val;
       int rc;

       prop = &pm8941_charger_attrs[which];

       rc = of_property_read_u32(chg->dev->of_node, prop->name, &val);
       if (rc == 0) {
               rc = pm8941_charger_attr_write(chg, which, val);
               if (!rc || !prop->fail_ok)
                       return rc;
       }
       return pm8941_charger_attr_read(chg, which);
}

static void pm8941_set_line_flag(struct pm8941_charger *chg, int irq, int flag)
{
       bool state;
       int ret;

       ret = irq_get_irqchip_state(irq, IRQCHIP_STATE_LINE_LEVEL, &state);
       if (state < 0) {
               dev_err(chg->dev, "failed to read irq line\n");
               return;
       }

       mutex_lock(&chg->statlock);
       if (state)
               chg->status |= flag;
       else
               chg->status &= ~flag;
       mutex_unlock(&chg->statlock);

       dev_dbg(chg->dev, "status = %03lx\n", chg->status);
}

static irqreturn_t pm8941_usb_valid_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;

       pm8941_set_line_flag(chg, irq, STATUS_USBIN_VALID);
       power_supply_changed(&chg->usb_psy);

       return IRQ_HANDLED;
}

static irqreturn_t pm8941_dc_valid_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;

       pm8941_set_line_flag(chg, irq, STATUS_DCIN_VALID);
       if (!chg->dc_disabled)
               power_supply_changed(&chg->dc_psy);

       return IRQ_HANDLED;
}

static irqreturn_t pm8941_bat_temp_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;
       unsigned int val;
       int rc;

       rc = regmap_read(chg->regmap, chg->addr + SMBB_BAT_TEMP_STATUS, &val);
       if (rc)
               return IRQ_HANDLED;

       mutex_lock(&chg->statlock);
       if (val & TEMP_STATUS_OK) {
               chg->status |= STATUS_BAT_OK;
       } else {
               chg->status &= ~STATUS_BAT_OK;
               if (val & TEMP_STATUS_HOT)
                       chg->status |= STATUS_BAT_HOT;
       }
       mutex_unlock(&chg->statlock);

       power_supply_changed(&chg->bat_psy);
       return IRQ_HANDLED;
}

static irqreturn_t pm8941_bat_present_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;

       pm8941_set_line_flag(chg, irq, STATUS_BAT_PRESENT);
       power_supply_changed(&chg->bat_psy);

       return IRQ_HANDLED;
}

static irqreturn_t pm8941_chg_done_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;

       pm8941_set_line_flag(chg, irq, STATUS_CHG_DONE);
       power_supply_changed(&chg->bat_psy);

       return IRQ_HANDLED;
}

static irqreturn_t pm8941_chg_gone_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;

       pm8941_set_line_flag(chg, irq, STATUS_CHG_GONE);
       power_supply_changed(&chg->bat_psy);
       power_supply_changed(&chg->usb_psy);
       if (!chg->dc_disabled)
               power_supply_changed(&chg->dc_psy);

       return IRQ_HANDLED;
}

static irqreturn_t pm8941_chg_fast_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;

       pm8941_set_line_flag(chg, irq, STATUS_CHG_FAST);
       power_supply_changed(&chg->bat_psy);

       return IRQ_HANDLED;
}

static irqreturn_t pm8941_chg_trkl_handler(int irq, void *_data)
{
       struct pm8941_charger *chg = _data;

       pm8941_set_line_flag(chg, irq, STATUS_CHG_TRKL);
       power_supply_changed(&chg->bat_psy);

       return IRQ_HANDLED;
}

static const struct pm8941_irq {
       const char *name;
       irqreturn_t (*handler)(int, void *);
} pm8941_charger_irqs[] = {
       { "chg-done", pm8941_chg_done_handler },
       { "chg-fast", pm8941_chg_fast_handler },
       { "chg-trkl", pm8941_chg_trkl_handler },
       { "bat-temp-ok", pm8941_bat_temp_handler },
       { "bat-present", pm8941_bat_present_handler },
       { "chg-gone", pm8941_chg_gone_handler },
       { "usb-valid", pm8941_usb_valid_handler },
       { "dc-valid", pm8941_dc_valid_handler },
};

static int pm8941_usbin_get_property(struct power_supply *psy,
               enum power_supply_property psp,
               union power_supply_propval *val)
{
       struct pm8941_charger *chg =
                       container_of(psy, struct pm8941_charger, usb_psy);
       int rc = 0;

       switch (psp) {
       case POWER_SUPPLY_PROP_ONLINE:
               mutex_lock(&chg->statlock);
               val->intval = !(chg->status & STATUS_CHG_GONE) &&
                               (chg->status & STATUS_USBIN_VALID);
               mutex_unlock(&chg->statlock);
               break;
       case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
               val->intval = chg->attr[ATTR_USBIN_IMAX];
               break;
       case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
               val->intval = 2500000;
               break;
       default:
               rc = -EINVAL;
               break;
       }

       return rc;
}

static int pm8941_usbin_set_property(struct power_supply *psy,
               enum power_supply_property psp,
               const union power_supply_propval *val)
{
       struct pm8941_charger *chg =
                       container_of(psy, struct pm8941_charger, usb_psy);
       int rc = 0;

       switch (psp) {
       case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
               rc = pm8941_charger_attr_write(chg, ATTR_USBIN_IMAX,
                               val->intval);
               break;
       default:
               rc = -EINVAL;
               break;
       }

       return rc;
}

static int pm8941_dcin_get_property(struct power_supply *psy,
               enum power_supply_property psp,
               union power_supply_propval *val)
{
       struct pm8941_charger *chg =
                       container_of(psy, struct pm8941_charger, dc_psy);
       int rc = 0;

       switch (psp) {
       case POWER_SUPPLY_PROP_ONLINE:
               mutex_lock(&chg->statlock);
               val->intval = !(chg->status & STATUS_CHG_GONE) &&
                               (chg->status & STATUS_DCIN_VALID);
               mutex_unlock(&chg->statlock);
               break;
       case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
               val->intval = chg->attr[ATTR_DCIN_IMAX];
               break;
       case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
               val->intval = 2500000;
               break;
       default:
               rc = -EINVAL;
               break;
       }

       return rc;
}

static int pm8941_dcin_set_property(struct power_supply *psy,
               enum power_supply_property psp,
               const union power_supply_propval *val)
{
       struct pm8941_charger *chg =
                       container_of(psy, struct pm8941_charger, dc_psy);
       int rc = 0;

       switch (psp) {
       case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
               rc = pm8941_charger_attr_write(chg, ATTR_DCIN_IMAX,
                               val->intval);
               break;
       default:
               rc = -EINVAL;
               break;
       }

       return rc;
}

static int pm8941_charger_writable_property(struct power_supply *psy,
               enum power_supply_property psp)
{
       return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT;
}

static int pm8941_battery_get_property(struct power_supply *psy,
               enum power_supply_property psp,
               union power_supply_propval *val)
{
       struct pm8941_charger *chg =
                       container_of(psy, struct pm8941_charger, bat_psy);
       unsigned long status;
       int rc = 0;

       mutex_lock(&chg->statlock);
       status = chg->status;
       mutex_unlock(&chg->statlock);

       switch (psp) {
       case POWER_SUPPLY_PROP_STATUS:
               if (status & STATUS_CHG_GONE)
                       val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
               else if (!(status & (STATUS_DCIN_VALID | STATUS_USBIN_VALID)))
                       val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
               else if (status & STATUS_CHG_DONE)
                       val->intval = POWER_SUPPLY_STATUS_FULL;
               else if (!(status & STATUS_BAT_OK))
                       val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
               else if (status & (STATUS_CHG_FAST | STATUS_CHG_TRKL))
                       val->intval = POWER_SUPPLY_STATUS_CHARGING;
               else /* everything is ok for charging, but we are not... */
                       val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
               break;
       case POWER_SUPPLY_PROP_HEALTH:
               if (status & STATUS_BAT_OK)
                       val->intval = POWER_SUPPLY_HEALTH_GOOD;
               else if (status & STATUS_BAT_HOT)
                       val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
               else
                       val->intval = POWER_SUPPLY_HEALTH_COLD;
               break;
       case POWER_SUPPLY_PROP_CHARGE_TYPE:
               if (status & STATUS_CHG_FAST)
                       val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
               else if (status & STATUS_CHG_TRKL)
                       val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
               else
                       val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
               break;
       case POWER_SUPPLY_PROP_PRESENT:
               val->intval = !!(status & STATUS_BAT_PRESENT);
               break;
       case POWER_SUPPLY_PROP_CURRENT_MAX:
               val->intval = chg->attr[ATTR_BAT_IMAX];
               break;
       case POWER_SUPPLY_PROP_VOLTAGE_MAX:
               val->intval = chg->attr[ATTR_BAT_VMAX];
               break;
       case POWER_SUPPLY_PROP_TECHNOLOGY:
               /* this charger is a single-cell lithium-ion battery charger
                * only.  If you hook up some other technology, there will be
                * fireworks.
                */
               val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
               break;
       case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
               val->intval = 3000000; /* single-cell li-ion low end */
               break;
       default:
               rc = -EINVAL;
               break;
       }

       return rc;
}

static int pm8941_battery_set_property(struct power_supply *psy,
               enum power_supply_property psp,
               const union power_supply_propval *val)
{
       struct pm8941_charger *chg =
                       container_of(psy, struct pm8941_charger, bat_psy);
       int rc = 0;

       switch (psp) {
       case POWER_SUPPLY_PROP_CURRENT_MAX:
               rc = pm8941_charger_attr_write(chg, ATTR_BAT_IMAX, val->intval);
               break;
       case POWER_SUPPLY_PROP_VOLTAGE_MAX:
               rc = pm8941_charger_attr_write(chg, ATTR_BAT_VMAX, val->intval);
               break;
       default:
               rc = -EINVAL;
               break;
       }

       return rc;
}

static int pm8941_battery_writable_property(struct power_supply *psy,
               enum power_supply_property psp)
{
       switch (psp) {
       case POWER_SUPPLY_PROP_CURRENT_MAX:
       case POWER_SUPPLY_PROP_VOLTAGE_MAX:
               return 1;
       default:
               return 0;
       }
}

static enum power_supply_property pm8941_charger_properties[] = {
       POWER_SUPPLY_PROP_ONLINE,
       POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
       POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
};

static enum power_supply_property pm8941_battery_properties[] = {
       POWER_SUPPLY_PROP_STATUS,
       POWER_SUPPLY_PROP_HEALTH,
       POWER_SUPPLY_PROP_PRESENT,
       POWER_SUPPLY_PROP_CHARGE_TYPE,
       POWER_SUPPLY_PROP_CURRENT_MAX,
       POWER_SUPPLY_PROP_VOLTAGE_MAX,
       POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
       POWER_SUPPLY_PROP_TECHNOLOGY,
};

static const struct reg_off_mask_default {
       unsigned int offset;
       unsigned int mask;
       unsigned int value;
       unsigned int rev_mask;
} pm8941_charger_setup[] = {
       /* The bootloader is supposed to set this... make sure anyway. */
       { SMBB_MISC_BOOT_DONE, BOOT_DONE, BOOT_DONE },

       /* Disable software timer */
       { SMBB_CHG_TCHG_MAX_EN, TCHG_MAX_EN, 0 },

       /* Clear and disable watchdog */
       { SMBB_CHG_WDOG_TIME, 0xff, 160 },
       { SMBB_CHG_WDOG_EN, WDOG_EN, 0 },

       /* Use charger based EoC detection */
       { SMBB_CHG_IBAT_TERM_CHG, IBAT_TERM_CHG_IEOC, IBAT_TERM_CHG_IEOC_CHG },

       /* Disable GSM PA load adjustment.
        * The PA signal is incorrectly connected on v2.
        */
       { SMBB_CHG_CFG, 0xff, 0x00, BIT(3) },

       /* Use VBAT (not VSYS) to compensate for IR drop during fast charging */
       { SMBB_BUCK_REG_MODE, BUCK_REG_MODE, BUCK_REG_MODE_VBAT },

       /* Enable battery temperature comparators */
       { SMBB_BAT_BTC_CTRL, BTC_CTRL_COMP_EN, BTC_CTRL_COMP_EN },

       /* Stop USB enumeration timer */
       { SMBB_USB_ENUM_TIMER_STOP, ENUM_TIMER_STOP, ENUM_TIMER_STOP },

#if 0 /* FIXME supposedly only to disable hardware ARB termination */
       { SMBB_USB_SEC_ACCESS, SEC_ACCESS_MAGIC },
       { SMBB_USB_REV_BST, 0xff, REV_BST_CHG_GONE },
#endif

       /* Stop USB enumeration timer, again */
       { SMBB_USB_ENUM_TIMER_STOP, ENUM_TIMER_STOP, ENUM_TIMER_STOP },

       /* Enable charging */
       { SMBB_CHG_CTRL, CTRL_EN, CTRL_EN },
};

/* FIXME: pinctrl does something similar... is there a common function?
 * If not, should there be?
 *
 * Update: seems to be that pinctrl's variant is:
 *  rc = of_property_read_u32(..., &val)
 *  if (rc == -EINVAL)
 *    return false;
 *  else if (rc)
 *    return true;
 *  return !!val;
 *
 *  ... but it's based on a table with defaults, so the commonality ends there.
 *  There are still others who desire this type of behavior though.
 */
static bool of_property_read_override_bool(const struct device_node *np,
               const char *propname)
{
       struct property *prop = of_find_property(np, propname, NULL);

       if (!prop)
               return false;
       if (!prop->value)
               return true;
       if (prop->length == sizeof(u32))
               return !!*(const __be32 *)prop->value;
       return true;
}

static int pm8941_charger_probe(struct platform_device *pdev)
{
       static char *battery[] = { "pm8941-bif" };
       struct pm8941_charger *chg;
       int rc, i;

       chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
       if (!chg)
               return -ENOMEM;

       chg->dev = &pdev->dev;
       mutex_init(&chg->statlock);

       chg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
       if (!chg->regmap) {
               dev_err(&pdev->dev, "failed to locate regmap\n");
               return -ENODEV;
       }

       rc = of_property_read_u32(pdev->dev.of_node, "reg", &chg->addr);
       if (rc) {
               dev_err(&pdev->dev, "missing or invalid 'reg' property\n");
               return rc;
       }

       rc = regmap_read(chg->regmap, chg->addr + SMBB_MISC_REV2,
                       &chg->revision);
       if (rc) {
               dev_err(&pdev->dev, "unable to read revision\n");
               return rc;
       }

       chg->revision += 1;
       if (chg->revision != 2 && chg->revision != 3) {
               dev_err(&pdev->dev, "v1 hardware not supported\n");
               return -ENODEV;
       }
       dev_info(&pdev->dev, "Initializing SMBB rev %u", chg->revision);

       chg->bat_psy.name = battery[0];
       chg->bat_psy.type = POWER_SUPPLY_TYPE_BATTERY;
       chg->bat_psy.of_node = pdev->dev.of_node;
       chg->bat_psy.properties = pm8941_battery_properties;
       chg->bat_psy.num_properties = ARRAY_SIZE(pm8941_battery_properties);
       chg->bat_psy.get_property = pm8941_battery_get_property;
       chg->bat_psy.set_property = pm8941_battery_set_property;
       chg->bat_psy.property_is_writeable = pm8941_battery_writable_property;

       chg->usb_psy.name = "pm8941-usbin";
       chg->usb_psy.type = POWER_SUPPLY_TYPE_USB;
       chg->usb_psy.supplied_to = battery;
       chg->usb_psy.num_supplicants = ARRAY_SIZE(battery);
       chg->usb_psy.properties = pm8941_charger_properties;
       chg->usb_psy.num_properties = ARRAY_SIZE(pm8941_charger_properties);
       chg->usb_psy.get_property = pm8941_usbin_get_property;
       chg->usb_psy.set_property = pm8941_usbin_set_property;
       chg->usb_psy.property_is_writeable = pm8941_charger_writable_property;

       chg->dc_psy.name = "pm8941-dcin";
       chg->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
       chg->dc_psy.supplied_to = battery;
       chg->dc_psy.num_supplicants = ARRAY_SIZE(battery);
       chg->dc_psy.properties = pm8941_charger_properties;
       chg->dc_psy.num_properties = ARRAY_SIZE(pm8941_charger_properties);
       chg->dc_psy.get_property = pm8941_dcin_get_property;
       chg->dc_psy.set_property = pm8941_dcin_set_property;
       chg->dc_psy.property_is_writeable = pm8941_charger_writable_property;

       chg->dc_disabled =
               of_property_read_override_bool(pdev->dev.of_node, "disable-dc");

       for (i = 0; i < _ATTR_CNT; ++i) {
               rc = pm8941_charger_attr_parse(chg, i);
               if (rc) {
                       dev_err(&pdev->dev, "failed to parse/apply settings\n");
                       return rc;
               }
       }

       rc = power_supply_register(&pdev->dev, &chg->bat_psy);
       if (rc) {
               dev_err(&pdev->dev, "failed to register battery\n");
               return rc;
       }

       rc = power_supply_register(&pdev->dev, &chg->usb_psy);
       if (rc) {
               power_supply_unregister(&chg->bat_psy);
               dev_err(&pdev->dev, "failed to register USB power supply\n");
               return rc;
       }

       if (!chg->dc_disabled) {
               rc = power_supply_register(&pdev->dev, &chg->dc_psy);
               if (rc) {
                       power_supply_unregister(&chg->usb_psy);
                       power_supply_unregister(&chg->bat_psy);
                       dev_err(&pdev->dev,
                               "failed to register DC power supply\n");
                       return rc;
               }
       }

       for (i = 0; i < ARRAY_SIZE(pm8941_charger_irqs); ++i) {
               int irq;

               irq = platform_get_irq_byname(pdev,
                               pm8941_charger_irqs[i].name);
               if (irq < 0) {
                       dev_err(&pdev->dev, "failed to get irq '%s'\n",
                               pm8941_charger_irqs[i].name);
                       rc = irq;
                       goto err;
               }

               pm8941_charger_irqs[i].handler(irq, chg);

               rc = devm_request_threaded_irq(&pdev->dev, irq, NULL,
                               pm8941_charger_irqs[i].handler, IRQF_ONESHOT,
                               pm8941_charger_irqs[i].name, chg);
               if (rc) {
                       dev_err(&pdev->dev, "failed to request irq '%s'\n",
                               pm8941_charger_irqs[i].name);
                       goto err;
               }
       }

       chg->jeita_ext_temp = of_property_read_override_bool(pdev->dev.of_node,
                       "jeita-extended-temp-range");

       /* Set temperature range to [35%:70%] or [25%:80%] accordingly */
       rc = regmap_update_bits(chg->regmap, chg->addr + SMBB_BAT_BTC_CTRL,
                       BTC_CTRL_COLD_EXT | BTC_CTRL_HOT_EXT_N,
                       chg->jeita_ext_temp ?
                               BTC_CTRL_COLD_EXT :
                               BTC_CTRL_HOT_EXT_N);
       if (rc) {
               dev_err(&pdev->dev,
                       "unable to set %s temperature range\n",
                       chg->jeita_ext_temp ? "JEITA extended" : "normal");
               goto err;
       }

       for (i = 0; i < ARRAY_SIZE(pm8941_charger_setup); ++i) {
               const struct reg_off_mask_default *r = &pm8941_charger_setup[i];

               if (r->rev_mask & BIT(chg->revision))
                       continue;

               rc = regmap_update_bits(chg->regmap, chg->addr + r->offset,
                               r->mask, r->value);
               if (rc) {
                       dev_err(&pdev->dev,
                               "unable to initializing charging, bailing\n");
                       goto err;
               }
       }

       platform_set_drvdata(pdev, chg);

       return 0;
err:
       if (!chg->dc_disabled)
               power_supply_unregister(&chg->dc_psy);
       power_supply_unregister(&chg->usb_psy);
       power_supply_unregister(&chg->bat_psy);

       return rc;
}

static int pm8941_charger_remove(struct platform_device *pdev)
{
       struct pm8941_charger *chg;

       chg = platform_get_drvdata(pdev);

       regmap_update_bits(chg->regmap, chg->addr + SMBB_CHG_CTRL, CTRL_EN, 0);

       if (!chg->dc_disabled)
               power_supply_unregister(&chg->dc_psy);
       power_supply_unregister(&chg->usb_psy);
       power_supply_unregister(&chg->bat_psy);

       return 0;
}

static const struct of_device_id pm8941_charger_id_table[] = {
       { .compatible = "qcom,pm8941-charger" },
       { }
};
MODULE_DEVICE_TABLE(of, pm8941_charger_id_table);

static struct platform_driver pm8941_charger_driver = {
       .probe          = pm8941_charger_probe,
       .remove         = pm8941_charger_remove,
       .driver         = {
               .name   = "pm8941-charger",
               .of_match_table = pm8941_charger_id_table,
       },
};
module_platform_driver(pm8941_charger_driver);

MODULE_ALIAS("platform:pm8941_charger");
MODULE_DESCRIPTION("PM8941 charger driver");
MODULE_LICENSE("GPL v2");
