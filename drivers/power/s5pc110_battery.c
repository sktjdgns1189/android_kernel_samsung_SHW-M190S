/*
 * linux/drivers/power/s3c6410_battery.c
 *
 * Battery measurement code for S3C6410 platform.
 *
 * based on palmtx_battery.c
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <asm/mach-types.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mfd/max8998.h>
#include <linux/mfd/max8998-private.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <mach/battery.h>
#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/adc.h>
#include <plat/gpio-cfg.h>
#include <linux/android_alarm.h>
#include "s5pc110_battery.h"
#include <linux/proc_fs.h>
#include <linux/earlysuspend.h>

#define BAT_POLLING_INTERVAL	10000
#define ADC_TOTAL_COUNT		10
#define ADC_DATA_ARR_SIZE	6

#define OFFSET_VIBRATOR_ON		(0x1 << 0)
#define OFFSET_CAMERA_ON		(0x1 << 1)
#define OFFSET_MP3_PLAY			(0x1 << 2)
#define OFFSET_VIDEO_PLAY		(0x1 << 3)
#define OFFSET_VOICE_CALL_2G		(0x1 << 4)
#define OFFSET_VOICE_CALL_3G		(0x1 << 5)
#define OFFSET_DATA_CALL		(0x1 << 6)
#define OFFSET_LCD_ON			(0x1 << 7)
#define OFFSET_TA_ATTACHED		(0x1 << 8)
#define OFFSET_CAM_FLASH		(0x1 << 9)
#define OFFSET_BOOTING			(0x1 << 10)

#define DISCONNECT_BAT_FULL		0x1
#define DISCONNECT_TEMP_OVERHEAT	0x2
#define DISCONNECT_TEMP_FREEZE		0x4
#define DISCONNECT_OVER_TIME		0x8
#define DISCONNECT_CF_ERROR			0x10

#define ATTACH_USB	1
#define ATTACH_TA	2

#if 0
#define bat_dbg(fmt, ...) printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#define bat_info(fmt, ...) printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bat_dbg(fmt, ...)
#define bat_info(fmt, ...) printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

/* external references */
extern unsigned int HWREV;
extern unsigned int is_cal_ftm_sleep;
extern void max17040_reset_soc(void);

/* global variables */
int isVoiceCall=0;
EXPORT_SYMBOL(isVoiceCall);

struct battery_info {
	//u32 batt_current;		/* Battery Charging Current (mA) from ADC */
	u32 batt_current_adc;	/* Battery Charging Current ADC value */
	u32 batt_temp;		/* Battery Temperature (C) from ADC */
	u32 batt_temp_adc;	/* Battery Temperature ADC value */
	u32 batt_temp_radc; /* Battery Temperature Rescale ADC value */
	u32 batt_health;	/* Battery Health (Authority) */
	u32 dis_reason;
	u32 batt_vcell;
	u32 batt_soc;
	u32 batt_psoc;
	u32 batt_presoc;
	u32 charging_status;
	bool batt_is_full;      /* 0 : Not full 1: Full */
	bool is_dock_charge;
	bool is_dock_usb;
};

struct battest_info {
	int rechg_count;
	int full_count;
	int test_value;
	int test_esuspend;
	bool is_rechg_state;
};

struct adc_sample_info {
	unsigned int cnt;
	int total_adc;
	int average_adc;
	int adc_arr[ADC_TOTAL_COUNT];
	int index;
};

struct chg_data {
	struct device		*dev;
	struct max8998_dev	*iodev;
	struct work_struct	bat_work;
	struct max8998_charger_data *pdata;

	struct power_supply	psy_bat;
	struct power_supply	psy_usb;
	struct power_supply	psy_ac;
	struct workqueue_struct *monitor_wqueue;
	struct wake_lock	vbus_wake_lock;
	struct wake_lock	work_wake_lock;
	struct wake_lock	test_wake_lock;
	struct wake_lock	dock_wake_lock;
	//struct wake_lock	lowbat_wake_lock;
	struct adc_sample_info	adc_sample[ENDOFADC];
	struct battery_info	bat_info;
	struct mutex		mutex;
	struct timer_list	bat_work_timer;
	struct proc_dir_entry *entry;
	struct battest_info	test_info;
	struct early_suspend    bat_early_suspend;

	enum cable_type_t	cable_status;
	int			charging;
	bool			set_charge_timeout;
	bool			lowbat_warning;
	int			present;
	int			jig_status;
	int			expt_tblocking;
	int			boot_complete;
	int			is_dock_disabled;
	u8			esafe;
	bool			set_batt_full;
	unsigned long		discharging_time;
	struct max8998_charger_callbacks callbacks;
};

static bool lpm_charging_mode;
static bool lpm_checked;

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property max8998_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property s3c_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static ssize_t s3c_bat_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t s3c_bat_store_attrs(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);

#define SEC_BATTERY_ATTR(_name)								\
{											\
	.attr = {.name = #_name, .mode = 0664, .owner = THIS_MODULE },	\
	.show = s3c_bat_show_attrs,							\
	.store = s3c_bat_store_attrs,								\
}

static struct device_attribute s3c_battery_attrs[] = {
  	SEC_BATTERY_ATTR(batt_vol),
  	SEC_BATTERY_ATTR(batt_vol_adc),
	SEC_BATTERY_ATTR(batt_temp),
	SEC_BATTERY_ATTR(batt_temp_adc),
	SEC_BATTERY_ATTR(batt_temp_radc),
	SEC_BATTERY_ATTR(batt_current_adc),
	SEC_BATTERY_ATTR(charging_source),
	SEC_BATTERY_ATTR(fg_soc),
	SEC_BATTERY_ATTR(fg_psoc),
	SEC_BATTERY_ATTR(reset_soc),
	SEC_BATTERY_ATTR(charging_mode_booting),
	SEC_BATTERY_ATTR(batt_temp_check),
	SEC_BATTERY_ATTR(batt_full_check),
	SEC_BATTERY_ATTR(batt_temp_adc_spec),
	SEC_BATTERY_ATTR(batt_test_value),
	SEC_BATTERY_ATTR(talk_gsm),
	SEC_BATTERY_ATTR(talk_wcdma),
	SEC_BATTERY_ATTR(dmb_play),
	SEC_BATTERY_ATTR(music_play),
	SEC_BATTERY_ATTR(video_play),
	SEC_BATTERY_ATTR(camera_use),
	SEC_BATTERY_ATTR(internet_use),
	SEC_BATTERY_ATTR(is_booting),
	SEC_BATTERY_ATTR(hw_revision),
	SEC_BATTERY_ATTR(batt_esus_test),
	SEC_BATTERY_ATTR(batt_type),
};

static void batt_set_temper_exception(struct chg_data *chg, int bit)
{
    chg->expt_tblocking |= (0x1<<bit);
}   

static void batt_clear_temper_exception(struct chg_data *chg, int bit)
{
    chg->expt_tblocking &= ~(0x1<<bit);
}

/*
static void max8998_lowbat_config(struct chg_data *chg, int on)
{
	struct i2c_client *i2c = chg->iodev->i2c;

	if (on) {
		if (!chg->lowbat_warning)
			max8998_update_reg(i2c, MAX8998_REG_ONOFF3, 0x1, 0x1); //lowbat1
		max8998_update_reg(i2c, MAX8998_REG_ONOFF3, 0x2, 0x2); //lowbat2
	} else
		max8998_update_reg(i2c, MAX8998_REG_ONOFF3, 0x0, 0x3);
}
*/

/*
static void max8998_lowbat_warning(struct chg_data *chg)
{
	bat_info("%s\n", __func__);
	wake_lock_timeout(&chg->lowbat_wake_lock, 5 * HZ);
	chg->lowbat_warning = 1;
}

static void max8998_lowbat_critical(struct chg_data *chg)
{
	bat_info("%s\n", __func__);
	wake_lock_timeout(&chg->lowbat_wake_lock, 30 * HZ);
	chg->bat_info.batt_soc = 0;
}
*/

static int max8998_charging_control(struct chg_data *chg)
{
	struct i2c_client *i2c = chg->iodev->i2c;
	static int prev_charging = -1, prev_cable = -1;
	int ret;

	if ((prev_charging == chg->charging) && (prev_cable == chg->cable_status))
		return 0;

	bat_info("%s : chg(%d) cable(%d) dis(%X) bat(%d,%d,%d), esafe(%d)\n", __func__,
		chg->charging, chg->cable_status, chg->bat_info.dis_reason,
		chg->bat_info.batt_soc, chg->set_batt_full, chg->bat_info.batt_is_full,
		chg->esafe);

	if (!chg->charging) {
		/* disable charging */
		ret = max8998_write_reg(i2c, MAX8998_REG_CHGR2,
			(chg->esafe		<< MAX8998_SHIFT_ESAFEOUT) |
			(MAX8998_CHGTIME_DISABLE	<< MAX8998_SHIFT_FT) |
			(MAX8998_CHGEN_DISABLE	<< MAX8998_SHIFT_CHGEN));
		if (ret < 0)
			goto err;
	} else {
		/* enable charging */
		if (chg->cable_status == CABLE_TYPE_AC) {
			/* ac */
			ret = max8998_write_reg(i2c, MAX8998_REG_CHGR1,
				(MAX8998_RSTR_DISABLE	<< MAX8998_SHIFT_RSTR) |
				(MAX8998_ICHG_600	<< MAX8998_SHIFT_ICHG));
			if (ret < 0)
				goto err;

			ret = max8998_write_reg(i2c, MAX8998_REG_CHGR2,
				(chg->esafe		<< MAX8998_SHIFT_ESAFEOUT) |
				(MAX8998_CHGTIME_DISABLE	<< MAX8998_SHIFT_FT) |
				(MAX8998_CHGEN_ENABLE	<< MAX8998_SHIFT_CHGEN));
			if (ret < 0)
				goto err;
		} else if (chg->cable_status == CABLE_TYPE_USB) {
			/* usb */
			ret = max8998_write_reg(i2c, MAX8998_REG_CHGR1,
				(MAX8998_RSTR_DISABLE	<< MAX8998_SHIFT_RSTR) |
				(MAX8998_ICHG_380	<< MAX8998_SHIFT_ICHG));
			if (ret < 0)
				goto err;

			ret = max8998_write_reg(i2c, MAX8998_REG_CHGR2,
				(chg->esafe		<< MAX8998_SHIFT_ESAFEOUT) |
				(MAX8998_CHGTIME_DISABLE	<< MAX8998_SHIFT_FT) |
				(MAX8998_CHGEN_ENABLE	<< MAX8998_SHIFT_CHGEN));
			if (ret < 0)
				goto err;
		} else {
			/* unknown cable, treat to usb-like */
			ret = max8998_write_reg(i2c, MAX8998_REG_CHGR1,
				(MAX8998_RSTR_DISABLE	<< MAX8998_SHIFT_RSTR) |
				(MAX8998_ICHG_380	<< MAX8998_SHIFT_ICHG));
			if (ret < 0)
				goto err;

			ret = max8998_write_reg(i2c, MAX8998_REG_CHGR2,
				(chg->esafe		<< MAX8998_SHIFT_ESAFEOUT) |
				(MAX8998_CHGTIME_DISABLE	<< MAX8998_SHIFT_FT) |
				(MAX8998_CHGEN_ENABLE	<< MAX8998_SHIFT_CHGEN));
			if (ret < 0)
				goto err;
		}
	}

	prev_charging = chg->charging;
	prev_cable = chg->cable_status;

	return 0;
err:
	pr_err("max8998_read_reg error\n");
	return ret;
}

static int max8998_check_vdcin(struct chg_data *chg)
{
	struct i2c_client *i2c = chg->iodev->i2c;
	u8 data = 0;
	int ret;

	ret = max8998_read_reg(i2c, MAX8998_REG_STATUS2, &data);

	if (ret < 0) {
		pr_err("max8998_read_reg error\n");
		return ret;
	}

	return (data & MAX8998_MASK_VDCIN) ? 1 : 0;
}

static int max8998_check_valid_battery(struct chg_data *chg)
{
	struct i2c_client *i2c = chg->iodev->i2c;
	u8 data = 0;
	int ret;

	ret = max8998_read_reg(i2c, MAX8998_REG_STATUS2, &data);

	if (ret < 0) {
		pr_err("max8998_read_reg error\n");
		return ret;
	}

	return (data & MAX8998_MASK_DETBAT) ? 0 : 1;
}

static void max8998_set_cable(struct max8998_charger_callbacks *ptr,
				enum cable_type_t status)
{
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);
	chg->cable_status = status;
	//chg->lowbat_warning = false;
	bat_info("%s : cable_status = %d\n", __func__, status);
	//power_supply_changed(&chg->psy_ac);
	//power_supply_changed(&chg->psy_usb);
	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);
}

static bool max8998_set_esafe(struct max8998_charger_callbacks *ptr, u8 esafe)
{
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);
	if (esafe > 3) {
		pr_err("%s : esafe value must not be bigger than 3\n", __func__);
		return 0;
	}
	chg->esafe = esafe;
	max8998_update_reg(chg->iodev->i2c, MAX8998_REG_CHGR2,
		(esafe << MAX8998_SHIFT_ESAFEOUT), MAX8998_MASK_ESAFEOUT);
	bat_info("%s : esafe = %d\n", __func__, esafe);
	return 1;
}

static bool max8998_get_vdcin(struct max8998_charger_callbacks *ptr)
{
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);
	return (max8998_check_vdcin(chg) == 1);
}

static void max8998_set_jig(struct max8998_charger_callbacks *ptr,
				int status)
{
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);
	chg->jig_status = status;
	bat_info("%s : jig_status = %d\n", __func__, status);
}

static void max8998_lowbat_interrupt(struct max8998_charger_callbacks *ptr)
{
	struct chg_data *chg = container_of(ptr, struct chg_data, callbacks);
	union power_supply_propval value;

	if (chg->pdata &&
	    chg->pdata->psy_fuelgauge &&
	    chg->pdata->psy_fuelgauge->get_property) {
		chg->pdata->psy_fuelgauge->get_property(
			chg->pdata->psy_fuelgauge, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
		chg->bat_info.batt_vcell = value.intval;

		chg->pdata->psy_fuelgauge->get_property(
			chg->pdata->psy_fuelgauge, POWER_SUPPLY_PROP_CAPACITY, &value);
		if ((chg->bat_info.charging_status != POWER_SUPPLY_STATUS_DISCHARGING) ||
		    (chg->bat_info.batt_soc > value.intval))
			chg->bat_info.batt_soc = value.intval;
		chg->pdata->psy_fuelgauge->get_property(
			chg->pdata->psy_fuelgauge, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &value);
		chg->bat_info.batt_psoc = value.intval;
	}
	
	printk("[fg-int] p:%d, s:%d, v:%d, t:%d\n", chg->bat_info.batt_psoc, 
		chg->bat_info.batt_soc, chg->bat_info.batt_vcell, chg->bat_info.batt_temp);

	/* defense code from invalid low interrrupt */
	/* you have to check with pure soc level, not adjusted. */
	/* 200(==2%), means 1%(setting) + margin 1%, apply in case of 0x1f */
	if(chg->bat_info.batt_psoc < 200) {
		chg->bat_info.batt_soc=0;
		power_supply_changed(&chg->psy_bat);
	} else
		bat_info("%s: unknown case, invalid low interrupt!\n", __func__);
	bat_info("%s : lowbat_interrupt at %d\n", __func__, chg->bat_info.batt_psoc);
}

static void check_lpm_charging_mode(struct chg_data *chg)
{
	bat_dbg("%s : lpm_charging_mode(%d) lpm_checked(%d)\n", __func__, lpm_charging_mode, lpm_checked);
	if (readl(S5P_INFORM5)) {
		lpm_charging_mode = 1;
		chg->boot_complete = 1;
		/*
		if (max8998_check_vdcin(chg) != 1)
			if (pm_power_off)
				pm_power_off();
		*/
	} else
		lpm_charging_mode = 0;

	lpm_checked = 1;
	bat_info("%s : lpm_charging_mode(%d) lpm_checked(%d)\n", __func__, lpm_charging_mode, lpm_checked);
}

bool charging_mode_get(void)
{
	if(lpm_checked)
		return lpm_charging_mode;
	else
		return readl(S5P_INFORM5);
}
EXPORT_SYMBOL(charging_mode_get);

static int s3c_bat_get_property(struct power_supply *bat_ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct chg_data *chg = container_of(bat_ps, struct chg_data, psy_bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (chg->test_info.test_value == 777 || chg->test_info.test_value == 999)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else
			val->intval = chg->bat_info.charging_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chg->bat_info.batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chg->present;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chg->bat_info.batt_temp;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* battery is always online */
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (chg->pdata &&
			 chg->pdata->psy_fuelgauge &&
			 chg->pdata->psy_fuelgauge->get_property &&
			 chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge, psp, val) < 0)
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chg->bat_info.batt_soc == 0)
			val->intval = 0;
		else if (chg->set_batt_full)
			val->intval = 100;
		else if (chg->pdata &&
			 chg->pdata->psy_fuelgauge &&
			 chg->pdata->psy_fuelgauge->get_property &&
			 chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge, psp, val) < 0)
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int s3c_usb_get_property(struct power_supply *ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct chg_data *chg = container_of(ps, struct chg_data, psy_usb);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the USB charger is connected */
	val->intval = ((chg->cable_status == CABLE_TYPE_USB) &&
			max8998_check_vdcin(chg));

	return 0;
}

static int s3c_ac_get_property(struct power_supply *ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct chg_data *chg = container_of(ps, struct chg_data, psy_ac);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (chg->cable_status == CABLE_TYPE_AC);

	return 0;
}

static int s3c_bat_get_adc_data(enum adc_channel_type adc_ch)
{
	int adc_data;
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;

	for (i = 0; i < ADC_DATA_ARR_SIZE; i++) {
		adc_data = s3c_adc_get_adc_data(adc_ch);

		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (ADC_DATA_ARR_SIZE - 2);
}

/*
static unsigned long calculate_average_adc(enum adc_channel_type channel,
					   int adc, struct chg_data *chg)
{
	unsigned int cnt = 0;
	int total_adc = 0;
	int average_adc = 0;
	int index = 0;

	cnt = chg->adc_sample[channel].cnt;
	total_adc = chg->adc_sample[channel].total_adc;

	if (adc <= 0) {
		pr_err("%s : invalid adc : %d\n", __func__, adc);
		adc = chg->adc_sample[channel].average_adc;
	}

	if (cnt < ADC_TOTAL_COUNT) {
		chg->adc_sample[channel].adc_arr[cnt] = adc;
		chg->adc_sample[channel].index = cnt;
		chg->adc_sample[channel].cnt = ++cnt;

		total_adc += adc;
		average_adc = total_adc / cnt;
	} else {
		index = chg->adc_sample[channel].index;
		if (++index >= ADC_TOTAL_COUNT)
			index = 0;

		total_adc = total_adc - chg->adc_sample[channel].adc_arr[index] + adc;
		average_adc = total_adc / ADC_TOTAL_COUNT;

		chg->adc_sample[channel].adc_arr[index] = adc;
		chg->adc_sample[channel].index = index;
	}

	chg->adc_sample[channel].total_adc = total_adc;
	chg->adc_sample[channel].average_adc = average_adc;

	chg->bat_info.batt_temp_adc = average_adc;

	return average_adc;
}
*/

static unsigned long s3c_read_temp(struct chg_data *chg)
{
	int adc = 0;

	adc = s3c_bat_get_adc_data(S3C_ADC_TEMPERATURE);

	//return calculate_average_adc(S3C_ADC_TEMPERATURE, adc, chg);

	if (adc == 0)
		adc = chg->bat_info.batt_temp_adc;
    chg->bat_info.batt_temp_adc = adc;
    return adc;
}

static unsigned long s3c_rescale_temp_adc(struct chg_data *chg)
{
	int adc_tmp = chg->bat_info.batt_temp_adc;
	int adc_tmp1 = 0; 
	int adc_tmp2 = 0;

	adc_tmp1 = adc_tmp * 10;
	adc_tmp2 = (40950 - adc_tmp1);
	adc_tmp = adc_tmp2 / 100;
	if ((adc_tmp2 % 10) >= 5)
		adc_tmp += 1;

	chg->bat_info.batt_temp_radc = adc_tmp;
	
	return adc_tmp;
}

static int s3c_get_bat_temp(struct chg_data *chg)
{
	int temp = 0;
	int temp_adc = s3c_read_temp(chg);
	int rescale_adc = 0;
	int health = chg->bat_info.batt_health;
	int left_side = 0;
	int right_side = chg->pdata->adc_array_size - 1;
	int mid;
	static int high_cnt = 0;
	static int low_cnt = 0;

	while (left_side <= right_side) {
		mid = (left_side + right_side) / 2 ;
		if (mid == 0 ||
		    mid == chg->pdata->adc_array_size - 1 ||
		    (chg->pdata->adc_table[mid].adc_value <= temp_adc &&
		     chg->pdata->adc_table[mid+1].adc_value > temp_adc)) {
			temp = chg->pdata->adc_table[mid].temperature;
			break;
		} else if (temp_adc - chg->pdata->adc_table[mid].adc_value > 0)
			left_side = mid + 1;
		else
			right_side = mid - 1;
	}

	chg->bat_info.batt_temp = temp;

	/*
	if (temp >= HIGH_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT &&
		    health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			chg->bat_info.batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (temp <= HIGH_RECOVER_TEMP && temp >= LOW_RECOVER_TEMP) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
		    health == POWER_SUPPLY_HEALTH_COLD)
			chg->bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (temp <= LOW_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_COLD &&
		    health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			chg->bat_info.batt_health = POWER_SUPPLY_HEALTH_COLD;
	}
	*/

	rescale_adc = s3c_rescale_temp_adc(chg);

    if (chg->test_info.test_value == 1) {
        rescale_adc = HIGH_BLOCK_TEMP_ADC + 1;
        if (chg->cable_status == CABLE_TYPE_NONE)
            rescale_adc = HIGH_RECOVER_TEMP_ADC - 1;
		chg->bat_info.batt_temp_radc = rescale_adc;
    }
	
	if (chg->cable_status == CABLE_TYPE_NONE || chg->test_info.test_value == 999) {
		high_cnt = 0;
		low_cnt = 0;
		chg->bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
		//printk("%s : skip_hupdate (1)\n", __func__);
		goto skip_hupdate;
	}

	if (rescale_adc >= HIGH_BLOCK_TEMP_ADC) {
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT)
			if (high_cnt <= 3)
				high_cnt++;
	} else if (rescale_adc <= HIGH_RECOVER_TEMP_ADC &&
		rescale_adc >= LOW_RECOVER_TEMP_ADC) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
		    health == POWER_SUPPLY_HEALTH_COLD) {
			high_cnt = 0;
			low_cnt = 0;
		}
	} else if (rescale_adc <= LOW_BLOCK_TEMP_ADC) {
		if (health != POWER_SUPPLY_HEALTH_COLD)
			if (low_cnt <= 3)
				low_cnt++;
	}

	if (high_cnt > 3)
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (low_cnt > 3)
		health = POWER_SUPPLY_HEALTH_COLD;
	else
		health = POWER_SUPPLY_HEALTH_GOOD;

	if((chg->boot_complete == 0) || (chg->expt_tblocking != 0)) {
		if((health == POWER_SUPPLY_HEALTH_OVERHEAT)	&&
			(rescale_adc <= EVENT_HIGH_BLOCK_TEMP_ADC))
			//printk("%s : skip_hupdate (2)\n", __func__);
			goto skip_hupdate;
	}
	chg->bat_info.batt_health = health;

	//printk("%s : high_cnt = %d, low_cnt = %d, health = %d\n",
	//	__func__, high_cnt, low_cnt, health);

skip_hupdate:
	return temp;
}

static void s3c_check_vf(struct chg_data *chg)
{
	static int cnt = 0;
	int health = chg->bat_info.batt_health;
	int vf_state = BAT_DETECTED;
    
	if ((chg->cable_status != CABLE_TYPE_NONE) && 
		(max8998_check_valid_battery(chg) == BAT_NOT_DETECTED)) {
		if(cnt >= BAT_DET_CNT)
			vf_state = BAT_NOT_DETECTED;
		else
			cnt++;
	} else {
		vf_state = BAT_DETECTED;
		cnt=0;
	}

	if (vf_state == BAT_NOT_DETECTED) {
		if (chg->jig_status || (chg->test_info.test_value == 999))
			health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else
			health = POWER_SUPPLY_HEALTH_DEAD;
		chg->present = 0;
	} else
		chg->present = 1;

	chg->bat_info.batt_health = health;
}

static void check_chg_current(struct chg_data *chg)
{
	unsigned long chg_current_adc = 0;
	//unsigned long chg_current_temp = 0;
	//unsigned long chg_current_volt = 0;
	//unsigned long chg_current = 0;

	chg_current_adc = s3c_bat_get_adc_data(S3C_ADC_CHG_CURRENT);
	chg->bat_info.batt_current_adc = chg_current_adc;

	//chg_current_temp = chg_current_adc * ADC_12BIT_RESOLUTION;
	//chg_current_volt = chg_current_temp / ADC_12BIT_SCALE;
	//if((chg_current_temp%ADC_12BIT_SCALE) >= (ADC_12BIT_SCALE/2))
	//chg_current_volt+=1;

	//chg_current_temp = 0;
	//chg_current_temp = (chg_current_volt*100) / ADC_CURRENT_FACTOR;
	//chg_current = chg_current_temp / 10;
	//if((chg_current_temp % 10) >= 5)
	//	chg_current+=1;

	//chg->bat_info.batt_current = chg_current;
    
#if 0 /* for debug */
    bat_info("%s : chg_current = %d adc, %dmV, %dmA\n", __func__,
    			chg_current_adc, chg_current_volt, chg_current);
#endif
}

static void s3c_check_chg_current(struct chg_data *chg)
{
	static int cnt = 0;
	
    if (chg->charging) {
        check_chg_current(chg);

		if (chg->test_info.test_value == 2)
		{
			chg->bat_info.batt_current_adc = CURRENT_OF_FULL_CHG - 1;
			cnt = FULL_CHG_COND_COUNT;
		} else if (chg->test_info.test_value == 3) {
			chg->bat_info.batt_current_adc = CURRENT_OF_FULL_CHG + 1;
			cnt = 0;
		}

		if (chg->bat_info.batt_vcell >= FULL_CHARGE_COND_VOLTAGE) {
			if (chg->bat_info.batt_current_adc <= CURRENT_OF_FULL_CHG) {
				cnt++;
				if (cnt >= FULL_CHG_COND_COUNT) {
					chg->set_batt_full = true; /* ui full == 100% */
					chg->bat_info.batt_is_full = true; /* charger full */
					cnt = 0;
				}
			}
		} else 
			cnt = 0;
	}
    else
    {
		cnt = 0;
		//chg->bat_info.batt_current = 0;
		chg->bat_info.batt_current_adc = 0;
    }
	chg->test_info.full_count = cnt;
}

static int s3c_check_recharging(struct chg_data *chg)
{
	static int cnt = 0;

	if (chg->bat_info.batt_vcell > RECHARGE_COND_VOLTAGE) {
		cnt = 0;
		return 0;
	} else {
		cnt++;
		if (cnt >= 5) { /* (5-1)-times of polling interval */
			cnt = 0;
			chg->test_info.is_rechg_state = true;
			return 1;
		} else if (chg->bat_info.batt_vcell <= FULL_CHARGE_COND_VOLTAGE) {
			cnt = 0;
			chg->test_info.is_rechg_state = true;
			return 1;
		} else
			return 0;
	}
	chg->test_info.rechg_count = cnt;
}

static void s3c_bat_discharge_reason(struct chg_data *chg)
{
	int discharge_reason;
	ktime_t ktime;
	struct timespec cur_time;
	union power_supply_propval value;

	if (chg->pdata &&
	    chg->pdata->psy_fuelgauge &&
	    chg->pdata->psy_fuelgauge->get_property) {
		chg->pdata->psy_fuelgauge->get_property(
			chg->pdata->psy_fuelgauge, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
		chg->bat_info.batt_vcell = value.intval;

		chg->pdata->psy_fuelgauge->get_property(
			chg->pdata->psy_fuelgauge, POWER_SUPPLY_PROP_CAPACITY, &value);
		chg->bat_info.batt_presoc = chg->bat_info.batt_soc;
		if (chg->set_batt_full)
        	chg->bat_info.batt_soc = 100;
		else if (chg->charging || chg->bat_info.batt_soc > value.intval)
    		chg->bat_info.batt_soc = value.intval;

		chg->pdata->psy_fuelgauge->get_property(
			chg->pdata->psy_fuelgauge, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &value);
		chg->bat_info.batt_psoc = value.intval;

		//printk("[battery] soc = %d, psoc = %d\n",
		//	chg->bat_info.batt_soc, chg->bat_info.batt_psoc);
	}

	discharge_reason = chg->bat_info.dis_reason & 0x1f;

	if ((discharge_reason & DISCONNECT_BAT_FULL) &&
	    s3c_check_recharging(chg))
		chg->bat_info.dis_reason &= ~DISCONNECT_BAT_FULL;

	if ((discharge_reason & DISCONNECT_TEMP_OVERHEAT) &&
	    chg->bat_info.batt_temp_radc <= HIGH_RECOVER_TEMP_ADC)
		chg->bat_info.dis_reason &= ~DISCONNECT_TEMP_OVERHEAT;

	if ((discharge_reason & DISCONNECT_TEMP_FREEZE) &&
	    chg->bat_info.batt_temp_radc >= LOW_RECOVER_TEMP_ADC)
		chg->bat_info.dis_reason &= ~DISCONNECT_TEMP_FREEZE;

	if ((discharge_reason & DISCONNECT_OVER_TIME) &&
	    s3c_check_recharging(chg))
		chg->bat_info.dis_reason &= ~DISCONNECT_OVER_TIME;

	if ((discharge_reason & DISCONNECT_CF_ERROR) && chg->present)
		chg->bat_info.dis_reason &= ~DISCONNECT_CF_ERROR;

	if (chg->bat_info.batt_is_full)
		chg->bat_info.dis_reason |= DISCONNECT_BAT_FULL;

	if (chg->bat_info.batt_health != POWER_SUPPLY_HEALTH_GOOD)
		switch (chg->bat_info.batt_health) {
			case POWER_SUPPLY_HEALTH_OVERHEAT:
				chg->bat_info.dis_reason |= DISCONNECT_TEMP_OVERHEAT;
				break;
			case POWER_SUPPLY_HEALTH_COLD:
				chg->bat_info.dis_reason |= DISCONNECT_TEMP_FREEZE;
				break;
			case POWER_SUPPLY_HEALTH_DEAD:
			case POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
				chg->bat_info.dis_reason |= DISCONNECT_CF_ERROR;
				break;
			default:
				break;
		}		

	ktime = alarm_get_elapsed_realtime();
	cur_time = ktime_to_timespec(ktime);

	if (chg->discharging_time &&
	    cur_time.tv_sec > chg->discharging_time) {
		chg->set_charge_timeout = true;
		chg->bat_info.dis_reason |= DISCONNECT_OVER_TIME;
	}

	bat_dbg("bat(%d,%d) tmp(%d,%d) full(%d,%d) cable(%d) chg(%d) dis(%X)\n",
		chg->bat_info.batt_soc, chg->bat_info.batt_vcell/1000,
		chg->bat_info.batt_temp, chg->bat_info.batt_temp_adc,
		chg->set_batt_full, chg->bat_info.batt_is_full,
		chg->cable_status, chg->bat_info.charging_status, chg->bat_info.dis_reason);
}

extern int set_tsp_for_ta_detect(int state);
static int s3c_cable_status_update(struct chg_data *chg)
{
	int ret;
	bool vdc_status;
	ktime_t ktime;
	struct timespec cur_time;
	static bool prev_vdc_status = 0;

	//printk("%s >>\n", __func__);
	/* if max8998 has detected vdcin */
	if (max8998_check_vdcin(chg) == 1) {
		//printk("%s : vdcin\n", __func__);
		if (chg->cable_status == CABLE_TYPE_NONE) {
			printk("%s : unknown cable-in detected!\n", __func__);
			chg->cable_status = CABLE_TYPE_UNKNOWN;
		}
		vdc_status = 1;
		if (chg->bat_info.dis_reason) {
			/* have vdcin, but cannot charge */
			chg->charging = 0;
			ret = max8998_charging_control(chg);
			if (ret < 0)
				goto err;
			if (chg->bat_info.dis_reason & 
			    (DISCONNECT_TEMP_OVERHEAT | DISCONNECT_TEMP_FREEZE | DISCONNECT_CF_ERROR)) {
				chg->set_batt_full = false;
				chg->test_info.is_rechg_state = false;
			}
			chg->bat_info.charging_status = chg->set_batt_full ?
				POWER_SUPPLY_STATUS_FULL :
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			chg->discharging_time = 0;
			chg->bat_info.batt_is_full = false;
			goto update;
		} else if (chg->discharging_time == 0) {
			ktime = alarm_get_elapsed_realtime();
			cur_time = ktime_to_timespec(ktime);
			chg->discharging_time =
				(chg->set_batt_full || chg->set_charge_timeout) ?
				cur_time.tv_sec + TOTAL_RECHARGING_TIME :
				cur_time.tv_sec + TOTAL_CHARGING_TIME;
		}

		/* able to charge */
		chg->charging = 1;
		ret = max8998_charging_control(chg);
		if (ret < 0)
			goto err;

		/* check dock state */
		if (chg->pdata->get_dock_intval()) {
			switch (chg->cable_status) {
			case CABLE_TYPE_AC:
				if (chg->bat_info.is_dock_charge == false) {
					printk("%s : enable docking\n", __func__);
					chg->bat_info.is_dock_charge = true;
					chg->pdata->dock_cb(true);
					batt_set_temper_exception(chg, DOCK_TEMP_EXCEPT_BIT);
				}
				break;
			case CABLE_TYPE_USB:
				if (chg->bat_info.is_dock_usb == false) {
					printk("%s : disable usb\n", __func__);
					chg->bat_info.is_dock_usb = true;
					chg->pdata->dock_usbctrl(true);
				}
				break;
			default:
				break;
			}
		}

		chg->bat_info.charging_status = chg->set_batt_full ?
			POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
	} else {
		//printk("%s : no vdcin\n", __func__);
		if (chg->cable_status == CABLE_TYPE_UNKNOWN) {
			printk("%s : unknown cable-out detected!\n", __func__);
			chg->cable_status = CABLE_TYPE_NONE;
		}

		vdc_status = 0;
		/* no vdc in, not able to charge */
		chg->charging = 0;
		ret = max8998_charging_control(chg);
		if (ret < 0)
			goto err;

		/* check dock state */
		if (chg->bat_info.is_dock_charge == true) {
			printk("%s : disable docking\n", __func__);
			chg->bat_info.is_dock_charge = false;
			chg->pdata->dock_cb(false);
			batt_clear_temper_exception(chg, DOCK_TEMP_EXCEPT_BIT);
		}
		
		/* check dock usb state */
		if (chg->bat_info.is_dock_usb == true) {
			printk("%s : enable usb\n", __func__);
			chg->bat_info.is_dock_usb = false;
			chg->pdata->dock_usbctrl(false);
		}

		chg->bat_info.charging_status = POWER_SUPPLY_STATUS_DISCHARGING;

		chg->bat_info.batt_is_full = false;
		chg->set_charge_timeout = false;
		chg->set_batt_full = false;
		chg->bat_info.dis_reason = 0;
		chg->discharging_time = 0;
		chg->test_info.is_rechg_state = false;

		//if (lpm_charging_mode && pm_power_off)
		//	pm_power_off();
	}

update:
	if (vdc_status)
		wake_lock(&chg->vbus_wake_lock);
	else
		wake_lock_timeout(&chg->vbus_wake_lock, 5*HZ);

	//printk("%s <<\n", __func__);
	if (vdc_status != prev_vdc_status) {
		set_tsp_for_ta_detect(vdc_status);
		prev_vdc_status = vdc_status;
	}
	return 0;
err:
	return ret;
}

static void s3c_bat_work(struct work_struct *work)
{
	struct chg_data *chg = container_of(work, struct chg_data, bat_work);
	int ret;
	wake_lock(&chg->work_wake_lock);
	mutex_lock(&chg->mutex);

	//printk("%s >>\n", __func__);
	
	s3c_get_bat_temp(chg);
	s3c_check_vf(chg);
	s3c_check_chg_current(chg);
	s3c_bat_discharge_reason(chg);

	ret = s3c_cable_status_update(chg);
	if (ret < 0)
		goto err;

	if (chg->bat_info.batt_soc != chg->bat_info.batt_presoc)
		printk("[fg] p:%d, s1:%d, s2:%d, v:%d, t:%d\n", chg->bat_info.batt_psoc,
			chg->bat_info.batt_soc, chg->bat_info.batt_presoc, 
			chg->bat_info.batt_vcell, chg->bat_info.batt_temp_radc);
	
	mutex_unlock(&chg->mutex);

	power_supply_changed(&chg->psy_bat);

	mod_timer(&chg->bat_work_timer, jiffies + msecs_to_jiffies(BAT_POLLING_INTERVAL));

	wake_unlock(&chg->work_wake_lock);
	//printk("%s <<\n", __func__);
	return;
err:
	mutex_unlock(&chg->mutex);
	wake_unlock(&chg->work_wake_lock);
	pr_err("battery workqueue fail\n");
}

static void s3c_bat_work_timer_func(unsigned long param)
{
	struct chg_data *chg = (struct chg_data *)param;

	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);
}

static ssize_t s3c_bat_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct chg_data *chg = container_of(psy, struct chg_data, psy_bat);
	int i = 0;
	const ptrdiff_t off = attr - s3c_battery_attrs;
	union power_supply_propval value;

	switch (off) {
	case BATT_VOL:
		if (chg->pdata &&
		    chg->pdata->psy_fuelgauge &&
		    chg->pdata->psy_fuelgauge->get_property) {
			chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
			chg->bat_info.batt_vcell = value.intval;
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_vcell);
		break;
	case BATT_VOL_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 0);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_temp);
		break;
	case BATT_TEMP_ADC:
		chg->bat_info.batt_temp_adc = s3c_bat_get_adc_data(S3C_ADC_TEMPERATURE);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_temp_adc);
		break;
	case BATT_TEMP_RADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_temp_radc);
		break;
	case BATT_CURRENT_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_current_adc);
		break;
	case BATT_CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->cable_status);
		break;
	case BATT_FG_SOC:
		if (chg->pdata &&
		    chg->pdata->psy_fuelgauge &&
		    chg->pdata->psy_fuelgauge->get_property) {
			chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge,
				POWER_SUPPLY_PROP_CAPACITY, &value);
			chg->bat_info.batt_soc = value.intval;
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_soc);
		break;
	case BATT_FG_PSOC:
		if (chg->pdata &&
		    chg->pdata->psy_fuelgauge &&
		    chg->pdata->psy_fuelgauge->get_property) {
			chg->pdata->psy_fuelgauge->get_property(
				chg->pdata->psy_fuelgauge,
				POWER_SUPPLY_PROP_CAPACITY_LEVEL, &value);
			chg->bat_info.batt_psoc = value.intval;
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_psoc);
		break;
	case CHARGING_MODE_BOOTING:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", lpm_charging_mode);
		break;
	case BATT_TEMP_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->bat_info.batt_health);
		break;
	case BATT_FULL_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->set_batt_full);
		break;
	case BATT_TEMP_ADC_SPEC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "(HIGH: %d - %d,   LOW: %d - %d)\n", 
				HIGH_BLOCK_TEMP_ADC, HIGH_RECOVER_TEMP_ADC,
				LOW_BLOCK_TEMP_ADC, LOW_RECOVER_TEMP_ADC);
		break;
	case BATT_TEST_VALUE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->test_info.test_value);
		break;
	case BATT_HW_REVISION:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", HWREV);
		break;
	case BATT_ESUS_TEST:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", chg->test_info.test_esuspend);
		break;
	case BATT_TYPE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "SDI_SDI\n");
		break;
	default:
		i = -EINVAL;
	}

	return i;
}

static ssize_t s3c_bat_store_attrs(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct chg_data *chg = container_of(psy, struct chg_data, psy_bat);
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - s3c_battery_attrs;

	switch (off) {
	case BATT_RESET_SOC:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				/* use for factory test, disable low battery popup */
				chg->test_info.test_value = 777;
				/* change ftm sleep timeout value for SMD Function Test */
				if(is_cal_ftm_sleep != 1)
                      is_cal_ftm_sleep = 1;
				max17040_reset_soc();
			}
			else if (x == 2)
				max17040_reset_soc();
			ret = count;
		}
		break;
	case CHARGING_MODE_BOOTING:
		if (sscanf(buf, "%d\n", &x) == 1) {
			lpm_charging_mode = x;
			ret = count;
		}
		break;
	case BATT_TEST_VALUE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 0)
				chg->test_info.test_value = 0;
			else if (x == 1)
				chg->test_info.test_value = 1; // for temp warning event
            else if (x == 2)
				chg->test_info.test_value = 2; // for full event
			else if (x == 3)
				chg->test_info.test_value = 3; // for abs time event
			else if (x == 999) {
				chg->test_info.test_value = 999; // for pop-up disable
				/* make the charging possible in case of temper. blocking state */
				if((chg->bat_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) ||
					(chg->bat_info.batt_health == POWER_SUPPLY_HEALTH_COLD)) {
					chg->bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
					wake_lock(&chg->work_wake_lock);
					queue_work(chg->monitor_wqueue, &chg->bat_work);
				}
			} else
				chg->test_info.test_value = 0;
			ret = count;
		}
		break;
	case BATT_VOICE_CALL_2G:
	case BATT_VOICE_CALL_3G:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				isVoiceCall = 1;
				batt_set_temper_exception(chg, CALL_TEMP_EXCEPT_BIT);
			} else {
				isVoiceCall = 0;
				batt_clear_temper_exception(chg, CALL_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		break;
	case BATT_DMB_PLAY:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(chg, DMB_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(chg, DMB_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		break;
	case BATT_MUSIC_PLAY:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(chg, MUSIC_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(chg,MUSIC_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		break;
	case BATT_VIDEO_PLAY:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(chg, VIDEO_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(chg, VIDEO_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		break;
	case BATT_CAMERA_USE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(chg, CAMERA_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(chg, CAMERA_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		break;
	case BATT_INTERNET_USE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(chg, INTERNEL_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(chg, INTERNEL_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		break;
	case BATT_BOOTING:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if(x==1) {
				chg->boot_complete = 1;
				printk("[battery] boot completed! - iboot_completed = %d\n", chg->boot_complete);
			}
			ret = count;
		}
		break;
	case BATT_ESUS_TEST: /* early_suspend test */
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 0) {
				chg->test_info.test_esuspend = 0;
				wake_lock_timeout(&chg->test_wake_lock, 5 * HZ);
			} else {
				chg->test_info.test_esuspend = 1;
				wake_lock(&chg->test_wake_lock);
			}
			ret = count;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int s3c_bat_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(s3c_battery_attrs); i++) {
		rc = device_create_file(dev, &s3c_battery_attrs[i]);
		if (rc)
			goto s3c_attrs_failed;
	}
	goto succeed;

s3c_attrs_failed:
	while (i--)
		device_remove_file(dev, &s3c_battery_attrs[i]);
succeed:
	return rc;
}

static irqreturn_t max8998_int_work_func(int irq, void *max8998_chg)
{
	int ret;
	u8 data[MAX8998_NUM_IRQ_REGS];

	struct chg_data *chg = max8998_chg;
    struct i2c_client *i2c = chg->iodev->i2c;

	ret = max8998_bulk_read(i2c, MAX8998_REG_IRQ1, MAX8998_NUM_IRQ_REGS, data);
	if (ret < 0)
		goto err;

#if 0
	wake_lock(&chg->work_wake_lock);

	if (data[MAX8998_REG_IRQ3] & MAX8998_IRQ_TOPOFFR_MASK) {
		bat_info("%s : topoff intr(%d)\n", __func__, chg->set_batt_full);
		if (chg->set_batt_full)
			chg->bat_info.batt_is_full = true;
		else {
			chg->set_batt_full = true;

			if (chg->cable_status == CABLE_TYPE_AC)
				max8998_write_reg(i2c, MAX8998_REG_CHGR1,
					(MAX8998_TOPOFF_10	<< MAX8998_SHIFT_TOPOFF) |
					(MAX8998_RSTR_DISABLE	<< MAX8998_SHIFT_RSTR) |
					(MAX8998_ICHG_600	<< MAX8998_SHIFT_ICHG));
			else if (chg->cable_status == CABLE_TYPE_USB)
				max8998_write_reg(i2c, MAX8998_REG_CHGR1,
					(MAX8998_TOPOFF_25	<< MAX8998_SHIFT_TOPOFF) |
					(MAX8998_RSTR_DISABLE	<< MAX8998_SHIFT_RSTR) |
					(MAX8998_ICHG_475	<< MAX8998_SHIFT_ICHG));
		}
	}

	if (data[MAX8998_REG_IRQ4] & MAX8998_IRQ_LOBAT1_MASK)
		max8998_lowbat_warning(chg);

	if (data[MAX8998_REG_IRQ4] & MAX8998_IRQ_LOBAT2_MASK)
		max8998_lowbat_critical(chg);

	queue_work(chg->monitor_wqueue, &chg->bat_work);
#endif

	return IRQ_HANDLED;
err:
	pr_err("%s : pmic read error\n", __func__);
	return IRQ_HANDLED;
}

static int s3c_bat_read_proc(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	struct chg_data *chg = data;
	struct timespec cur_time;
	ktime_t ktime;
	int len = 0;

	ktime = alarm_get_elapsed_realtime();
	cur_time = ktime_to_timespec(ktime);

	len = sprintf(buf, "%lu, %u, %u, %u, %u, %u, %d, %u, %u, %d, %d, %d, %u, %u, %u, %u, %d, %d, %lu\n",
            //get_seconds(),
            cur_time.tv_sec,
            chg->bat_info.batt_psoc,
            chg->bat_info.batt_soc,
            chg->bat_info.batt_vcell,
            chg->bat_info.dis_reason,
            chg->bat_info.batt_current_adc,
            chg->test_info.rechg_count,
            chg->set_batt_full,
            chg->bat_info.batt_is_full,
            chg->charging,
            chg->test_info.is_rechg_state,
            chg->test_info.full_count,
            chg->bat_info.batt_temp_adc,
            chg->bat_info.batt_temp_radc,
            chg->bat_info.batt_health,
            chg->bat_info.charging_status,
            chg->present,
            chg->cable_status,
            chg->discharging_time);

    return len;
}

static void max8998_charger_early_suspend(struct early_suspend *h)
{
	struct chg_data *chg = container_of(h, struct chg_data, bat_early_suspend);
	
	if(chg->pdata->get_dock_intval() && !chg->pdata->get_dock_operation()) {
		printk("%s...\n", __func__);
		chg->pdata->dock_earlysuspend_ctrl(true);
		chg->is_dock_disabled = 1;
		wake_lock_timeout(&chg->dock_wake_lock, 3 * HZ);
	}

    return;
}

static void max8998_charger_late_resume(struct early_suspend *h)
{
	struct chg_data *chg = container_of(h, struct chg_data, bat_early_suspend);
	
	if(chg->is_dock_disabled &&
	   chg->pdata->get_dock_intval() &&
	   chg->cable_status == CABLE_TYPE_AC) {
		printk("%s...\n", __func__);
		chg->pdata->dock_earlysuspend_ctrl(false);
		wake_lock_timeout(&chg->dock_wake_lock, 3 * HZ);
	}
	chg->is_dock_disabled = 0;

    return;
}

static __devinit int max8998_charger_probe(struct platform_device *pdev)
{
	struct max8998_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max8998_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct chg_data *chg;
	struct i2c_client *i2c = iodev->i2c;
	int ret = 0;

	bat_info("%s : MAX8998 Charger Driver Loading\n", __func__);

	chg = kzalloc(sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->iodev = iodev;
	chg->pdata = pdata->charger;

	if (!chg->pdata || !chg->pdata->adc_table) {
		pr_err("%s : No platform data & adc_table supplied\n", __func__);
		ret = -EINVAL;
		goto err_bat_table;
	}

	chg->psy_bat.name = "battery",
	chg->psy_bat.type = POWER_SUPPLY_TYPE_BATTERY,
	chg->psy_bat.properties = max8998_battery_props,
	chg->psy_bat.num_properties = ARRAY_SIZE(max8998_battery_props),
	chg->psy_bat.get_property = s3c_bat_get_property,

	chg->psy_usb.name = "usb",
	chg->psy_usb.type = POWER_SUPPLY_TYPE_USB,
	chg->psy_usb.supplied_to = supply_list,
	chg->psy_usb.num_supplicants = ARRAY_SIZE(supply_list),
	chg->psy_usb.properties = s3c_power_properties,
	chg->psy_usb.num_properties = ARRAY_SIZE(s3c_power_properties),
	chg->psy_usb.get_property = s3c_usb_get_property,

	chg->psy_ac.name = "ac",
	chg->psy_ac.type = POWER_SUPPLY_TYPE_MAINS,
	chg->psy_ac.supplied_to = supply_list,
	chg->psy_ac.num_supplicants = ARRAY_SIZE(supply_list),
	chg->psy_ac.properties = s3c_power_properties,
	chg->psy_ac.num_properties = ARRAY_SIZE(s3c_power_properties),
	chg->psy_ac.get_property = s3c_ac_get_property,

	chg->present = 1;
	chg->bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	chg->bat_info.batt_is_full = false;
	chg->set_batt_full = false;
	chg->set_charge_timeout = false;

	chg->cable_status = CABLE_TYPE_NONE;
	chg->esafe = MAX8998_USB_VBUS_AP_ON;
	chg->jig_status = 0;
	chg->test_info.test_value = 0;
	chg->expt_tblocking = 0;
	chg->boot_complete = 0;
	chg->bat_info.batt_soc = 100;

	chg->bat_info.is_dock_charge = false;
	
	mutex_init(&chg->mutex);

	platform_set_drvdata(pdev, chg);

	ret = max8998_write_reg(i2c, MAX8998_REG_IRQM1,
		~(MAX8998_IRQ_DCINR_MASK | MAX8998_IRQ_DCINF_MASK));
	if (ret < 0)
		goto err_kfree;

	ret = max8998_write_reg(i2c, MAX8998_REG_IRQM2, 0xFF);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_write_reg(i2c, MAX8998_REG_IRQM3, 0xFF);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_write_reg(i2c, MAX8998_REG_IRQM4, 0xFF);
	if (ret < 0)
		goto err_kfree;

	/*
	ret = max8998_update_reg(i2c, MAX8998_REG_ONOFF3,
		(1 << MAX8998_SHIFT_ENBATTMON), MAX8998_MASK_ENBATTMON);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_update_reg(i2c, MAX8998_REG_LBCNFG1, 0x7, 0x37); //3.57V
	if (ret < 0)
		goto err_kfree;

	ret = max8998_update_reg(i2c, MAX8998_REG_LBCNFG2, 0x5, 0x37); //3.4V
	if (ret < 0)
		goto err_kfree;
	*/

	//max8998_lowbat_config(chg, 0);

	wake_lock_init(&chg->vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	wake_lock_init(&chg->work_wake_lock, WAKE_LOCK_SUSPEND, "max8998-charger");
	wake_lock_init(&chg->test_wake_lock, WAKE_LOCK_SUSPEND, "bat_esus_test");
	wake_lock_init(&chg->dock_wake_lock, WAKE_LOCK_SUSPEND, "dock_on_off");
	//wake_lock_init(&chg->lowbat_wake_lock, WAKE_LOCK_SUSPEND, "max8998-lowbat");

	INIT_WORK(&chg->bat_work, s3c_bat_work);
	setup_timer(&chg->bat_work_timer, s3c_bat_work_timer_func, (unsigned long)chg);

	chg->monitor_wqueue = create_freezeable_workqueue(dev_name(&pdev->dev));
	if (!chg->monitor_wqueue) {
		pr_err("%s : Failed to create freezeable workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_wake_lock;
	}

	check_lpm_charging_mode(chg);

	/* init power supplier framework */
	ret = power_supply_register(&pdev->dev, &chg->psy_bat);
	if (ret) {
		pr_err("%s : Failed to register power supply psy_bat\n", __func__);
		goto err_wqueue;
	}

	ret = power_supply_register(&pdev->dev, &chg->psy_usb);
	if (ret) {
		pr_err("%s : Failed to register power supply psy_usb\n", __func__);
		goto err_supply_unreg_bat;
	}

	ret = power_supply_register(&pdev->dev, &chg->psy_ac);
	if (ret) {
		pr_err("%s : Failed to register power supply psy_ac\n", __func__);
		goto err_supply_unreg_usb;
	}

	ret = request_threaded_irq(iodev->i2c->irq, NULL, max8998_int_work_func,
				   IRQF_TRIGGER_FALLING, "max8998-charger", chg);
	if (ret) {
		pr_err("%s : Failed to request pmic irq\n", __func__);
		goto err_supply_unreg_ac;
	}

	ret = enable_irq_wake(iodev->i2c->irq);
	if (ret) {
		pr_err("%s : Failed to enable pmic irq wake\n", __func__);
		goto err_irq;
	}

	ret = s3c_bat_create_attrs(chg->psy_bat.dev);
	if (ret) {
		pr_err("%s : Failed to create_attrs\n", __func__);
		goto err_irq;
	}

	chg->callbacks.set_cable = max8998_set_cable;
	chg->callbacks.set_esafe = max8998_set_esafe;
	chg->callbacks.get_vdcin = max8998_get_vdcin;
	chg->callbacks.set_jig = max8998_set_jig;
	chg->callbacks.lowbat_interrupt = max8998_lowbat_interrupt;
	if (chg->pdata->register_callbacks)
		chg->pdata->register_callbacks(&chg->callbacks);

	chg->bat_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	chg->bat_early_suspend.suspend = max8998_charger_early_suspend;
	chg->bat_early_suspend.resume = max8998_charger_late_resume;
	register_early_suspend(&chg->bat_early_suspend);

	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);

	chg->entry = create_proc_entry("batt_info_proc", S_IRUGO, NULL);
	if(!chg->entry) {
		pr_err("%s: Failed to create_proc_entry\n", __func__);
    } else {
    	chg->entry->read_proc = s3c_bat_read_proc;
		chg->entry->data = (struct chg_data *)chg;
    }

	return 0;

err_irq:
	free_irq(iodev->i2c->irq, NULL);
err_supply_unreg_ac:
	power_supply_unregister(&chg->psy_ac);
err_supply_unreg_usb:
	power_supply_unregister(&chg->psy_usb);
err_supply_unreg_bat:
	power_supply_unregister(&chg->psy_bat);
err_wqueue:
	destroy_workqueue(chg->monitor_wqueue);
	cancel_work_sync(&chg->bat_work);
err_wake_lock:
	wake_lock_destroy(&chg->work_wake_lock);
	wake_lock_destroy(&chg->vbus_wake_lock);
	wake_lock_destroy(&chg->test_wake_lock);
	//wake_lock_destroy(&chg->lowbat_wake_lock);
err_kfree:
	mutex_destroy(&chg->mutex);
err_bat_table:
	kfree(chg);
	return ret;
}

static int __devexit max8998_charger_remove(struct platform_device *pdev)
{
	struct chg_data *chg = platform_get_drvdata(pdev);

	remove_proc_entry("batt_info_proc", NULL);
	free_irq(chg->iodev->i2c->irq, NULL);
	flush_workqueue(chg->monitor_wqueue);
	destroy_workqueue(chg->monitor_wqueue);
	power_supply_unregister(&chg->psy_bat);
	power_supply_unregister(&chg->psy_usb);
	power_supply_unregister(&chg->psy_ac);

	wake_lock_destroy(&chg->work_wake_lock);
	wake_lock_destroy(&chg->vbus_wake_lock);
	wake_lock_destroy(&chg->test_wake_lock);
	//wake_lock_destroy(&chg->lowbat_wake_lock);
	mutex_destroy(&chg->mutex);
	kfree(chg);

	return 0;
}

static int max8998_charger_suspend(struct device *dev)
{
	struct chg_data *chg = dev_get_drvdata(dev);
	//max8998_lowbat_config(chg, 1);
	del_timer_sync(&chg->bat_work_timer);
	cancel_work_sync(&chg->bat_work);
	return 0;
}

static void max8998_charger_resume(struct device *dev)
{
	struct chg_data *chg = dev_get_drvdata(dev);
	//max8998_lowbat_config(chg, 0);
	wake_lock(&chg->work_wake_lock);
	queue_work(chg->monitor_wqueue, &chg->bat_work);
}

static const struct dev_pm_ops max8998_charger_pm_ops = {
	.prepare        = max8998_charger_suspend,
	.complete       = max8998_charger_resume,
};

static struct platform_driver max8998_charger_driver = {
	.driver = {
		.name = "max8998-charger",
		.owner = THIS_MODULE,
		.pm = &max8998_charger_pm_ops,
	},
	.probe = max8998_charger_probe,
	.remove = __devexit_p(max8998_charger_remove),
};

static int __init max8998_charger_init(void)
{
	return platform_driver_register(&max8998_charger_driver);
}

static void __exit max8998_charger_exit(void)
{
	platform_driver_unregister(&max8998_charger_driver);
}

late_initcall(max8998_charger_init);
module_exit(max8998_charger_exit);

MODULE_AUTHOR("Minsung Kim <ms925.kim@samsung.com>");
MODULE_DESCRIPTION("S3C6410 battery driver");
MODULE_LICENSE("GPL");

