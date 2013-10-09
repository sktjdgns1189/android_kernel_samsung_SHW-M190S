/* linux/driver/input/misc/taos.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/gp2a.h>
#include <plat/gpio-cfg.h>
#include "taos_triton.h"


/* Note about power vs enable/disable:
 *  The chip has two functions, proximity and ambient light sensing.
 *  There is no separate power enablement to the two functions (unlike
 *  the Capella CM3602/3623).
 *  This module implements two drivers: /dev/proximity and /dev/light.
 *  When either driver is enabled (via sysfs attributes), we give power
 *  to the chip.  When both are disabled, we remove power from the chip.
 *  In suspend, we remove power if light is disabled but not if proximity is
 *  enabled (proximity is allowed to wakeup from suspend).
 *
 *  There are no ioctls for either driver interfaces.  Output is via
 *  input device framework and control via sysfs attributes.
 */


#define taos_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)
//#define taos_dbgmsg(str, args...) printk("%s: " str, __func__, ##args)
#define TAOS_DEBUG 0

/*********** for debug **********************************************************/
#ifdef TAOS_DEBUG
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

#define ADC_BUFFER_NUM 	8

/* ADDSEL is LOW */
#define REGS_PROX		0x0 /* Read  Only */
#define REGS_GAIN		0x1 /* Write Only */
#define REGS_HYS		0x2 /* Write Only */
#define REGS_CYCLE		0x3 /* Write Only */
#define REGS_OPMOD		0x4 /* Write Only */

/* sensor type */
#define LIGHT           0
#define PROXIMITY	1
#define ALL		2

static u8 reg_defaults[5] = {
	0x00, /* PROX: read only register */
	0x08, /* GAIN: large LED drive level */
	0x40, /* HYS: receiver sensitivity */
	0x04, /* CYCLE: */
	0x01, /* OPMOD: normal operating mode */
};

struct taos_data;

struct taos_data *g_taos_data = NULL;

enum {
	LIGHT_ENABLED = BIT(0),
	PROXIMITY_ENABLED = BIT(1),
};

/* driver data */
struct taos_data {
	struct input_dev *proximity_input_dev;
	struct input_dev *light_input_dev;
	struct gp2a_platform_data *pdata;
	struct i2c_client *i2c_client;
	struct class *lightsensor_class;
	struct class *proxsensor_class;
	struct device *switch_cmd_dev;
	int irq;
	struct work_struct work_light;
	struct work_struct work_prox;	
	struct hrtimer timer;
	ktime_t light_poll_delay;
	int adc_value_buf[ADC_BUFFER_NUM];
	int adc_index_count;
	bool adc_buf_initialized;
	bool on;
	u8 power_state;
	struct mutex power_lock;
	struct wake_lock prx_wake_lock;
	struct workqueue_struct *wq;
	char val_state;
};

unsigned int taos_i2c_err = 0;
//void proximity_jack_control(int on);
extern int prox_factorymode;

/////////////////////////
#define TAOS_PRX_THRES_OUT 		400
#define TAOS_PRX_THRES_IN	    500		    // IMPORTANT : PRX_COUNT 0x0a
#define TAOS_PRX_THRES_MAX	    1024		// IMPORTANT : PRX_TIME 0xff

static bool light_enable = 0;
static bool proximity_enable = 0;

/* Als/prox info */
static struct taos_prox_info prox_cur_info;
static struct taos_prox_info *prox_cur_infop = &prox_cur_info;

static int cur_adc_value = 0;

static state_type cur_state = LIGHT_INIT;

static TAOS_CHIP_WORKING_STATUS taos_chip_status = TAOS_CHIP_UNKNOWN;
static TAOS_PRX_DISTANCE_STATUS taos_prox_dist = TAOS_PRX_DIST_UNKNOWN;

static int taos_prox_on(struct taos_data *taos);
static int taos_prox_off(struct taos_data *taos);
static int taos_chip_on(struct taos_data *taos);
static int taos_chip_off(struct taos_data *taos);
static int taos_read_als_data(struct taos_data *taos);
static int taos_get_lux(struct taos_data *taos);
static int taos_prox_poll(struct taos_data *taos, struct taos_prox_info *prxp);
/////////////////////////
int taos_i2c_read(struct taos_data *taos, u8 reg, u8 *val, unsigned int len )
{
	int err;
	u8 buf0[] = { 0 };
	u8 buf1[] = { 0xff };
	struct i2c_msg msg[2];
	struct i2c_client *client = taos->i2c_client;
//    gprintk("\n");
    taos_i2c_err = 0;

	#if 1 // E1
	if( (client == NULL) || (!client->adapter) )
	{
		pr_err("opt_i2c_read failed : i2c_client is NULL. (ALS hasn't initialized in a proper manner.)\n"); 
		return -ENODEV;
	}	
	#endif

	buf0[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = len;	
	msg[0].buf = buf0;

	msg[1].addr = client->addr;
	msg[1].flags = 1;	
	msg[1].len = len;	
	msg[1].buf = buf1;

	err = i2c_transfer(client->adapter, &msg[0], 1);
	err = i2c_transfer(client->adapter, &msg[1], 1);
	
	*val = buf1[0];
    if (err >= 0) return 0;
//printk(KERN_DEBUG "(%x)\n",client->addr);
    printk(KERN_DEBUG "%s %d i2c transfer error %d\n", __func__, __LINE__,err);
    taos_i2c_err = 1;
    
    return err;
}
int taos_i2c_write(struct taos_data *taos, u8 reg, u8 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 10;
	struct i2c_client *client = taos->i2c_client;

    taos_i2c_err = 0;	

	if ((client == NULL) || (!client->adapter)){
	    taos_i2c_err = 1;			
		return -ENODEV;
	}

	while (retry--) {
		data[0] = reg;
		data[1] = *val;

		msg->addr = client->addr;
		msg->flags = 0; /* write */
		msg->len = 2;
		msg->buf = data;

		err = i2c_transfer(client->adapter, msg, 1);

		if (err >= 0)
			return 0;
	}
    taos_i2c_err = 1;		
	return err;
}

static int taos_i2c_write_command(struct taos_data *taos, u8 reg)
{
    int err;
    struct i2c_msg msg[1];
    unsigned char data[1]; //unsigned char data[2];
    int retry = 10;
    struct i2c_client *client = taos->i2c_client;

    if( (client == NULL) || (!client->adapter) )
	{
        return -ENODEV;
    }

    while(retry--)
    {
	    data[0] = reg;
	    //data[1] = *val;

	    msg->addr = client->addr;
	    msg->flags = I2C_M_WR;
	    msg->len = 1; //msg->len = 2;
	    msg->buf = data;

	    err = i2c_transfer(client->adapter, msg, 1);

	    if (err >= 0) 
		{
			return 0;
		}
    }

    printk("%s %d i2c transfer error %d\n", __func__, __LINE__, err);
    return err;
}


static void taos_light_enable(struct taos_data *taos)
{
	taos_dbgmsg("starting poll timer, delay %lldns\n",
		    ktime_to_ns(taos->light_poll_delay));
	hrtimer_start(&taos->timer, taos->light_poll_delay, HRTIMER_MODE_REL);
}

static void taos_light_disable(struct taos_data *taos)
{
	taos_dbgmsg("cancelling poll timer\n");
	hrtimer_cancel(&taos->timer);
	cancel_work_sync(&taos->work_light);
}

static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return sprintf(buf, "%lld\n", ktime_to_ns(taos->light_poll_delay));
}


static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;
//	printk(KERN_ERR "called %s \n",__func__);
	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	taos_dbgmsg("new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(taos->light_poll_delay));
	mutex_lock(&taos->power_lock);
	if (new_delay != ktime_to_ns(taos->light_poll_delay)) {
		taos->light_poll_delay = ns_to_ktime(new_delay);
		if (taos->power_state & LIGHT_ENABLED) {
			taos_light_disable(taos);
			taos_light_enable(taos);
		}
	}
	mutex_unlock(&taos->power_lock);

	return size;
}

static ssize_t light_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",
		       (taos->power_state & LIGHT_ENABLED) ? 1 : 0);
}

static ssize_t proximity_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",
		       (taos->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}

static ssize_t light_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	bool new_value;
//	printk(KERN_ERR "called %s \n",__func__);
	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&taos->power_lock);
	taos_dbgmsg("new_value = %d, old state = %d\n",
		    new_value, (taos->power_state & LIGHT_ENABLED) ? 1 : 0);
	if (new_value && !(taos->power_state & LIGHT_ENABLED)) {
		if (!taos->power_state)
			{
			taos->pdata->power(true);
			taos_chip_on(taos);
			}
		taos->power_state |= LIGHT_ENABLED;
		taos_light_enable(taos);
	} else if (!new_value && (taos->power_state & LIGHT_ENABLED)) {
		taos_light_disable(taos);
		taos->power_state &= ~LIGHT_ENABLED;
		if (!taos->power_state)
			{
			taos_chip_off(taos);
			taos->pdata->power(false);
			}
	}
	mutex_unlock(&taos->power_lock);
	return size;
}

static ssize_t proximity_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	bool new_value;
//	printk(KERN_ERR "called %s \n",__func__);
	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&taos->power_lock);
	printk("[Proximity_enable_store] new_value = %d, old state = %d\n",
		    new_value, (taos->power_state & PROXIMITY_ENABLED) ? 1 : 0);
	if (new_value && !(taos->power_state & PROXIMITY_ENABLED)) {
		if (!taos->power_state)
			{
			taos->pdata->power(true);
			taos_chip_on(taos);
			}
		taos->power_state |= PROXIMITY_ENABLED;
//		enable_irq(taos->irq);
//		enable_irq_wake(taos->irq);
		taos_prox_on(taos);
//		taos_i2c_write(taos, REGS_GAIN, &reg_defaults[1]);
//		taos_i2c_write(taos, REGS_HYS, &reg_defaults[2]);
//		taos_i2c_write(taos, REGS_CYCLE, &reg_defaults[3]);
//		taos_i2c_write(taos, REGS_OPMOD, &reg_defaults[4]);
	} else if (!new_value && (taos->power_state & PROXIMITY_ENABLED)) {
//		disable_irq_wake(taos->irq);
//		disable_irq(taos->irq);
//		taos_i2c_write(taos, REGS_OPMOD, &reg_defaults[0]);
		taos_prox_off(taos);
		taos->power_state &= ~PROXIMITY_ENABLED;
		if (!taos->power_state)
			{
			taos_chip_off(taos);			
			taos->pdata->power(false);
			}
	}
	mutex_unlock(&taos->power_lock);
	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       light_enable_show, light_enable_store);

static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       proximity_enable_show, proximity_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

static int lightsensor_get_adcvalue(struct taos_data *taos)
{
	int i = 0;
	int j = 0;
	unsigned int adc_total = 0;
	int adc_avr_value;
	unsigned int adc_index = 0;
	unsigned int adc_max = 0;
	unsigned int adc_min = 0;
	int value = 0;

	/* get ADC */
	value = taos->pdata->light_adc_value();

	adc_index = (taos->adc_index_count++) % ADC_BUFFER_NUM;

	/*ADC buffer initialize (light sensor off ---> light sensor on) */
	if (!taos->adc_buf_initialized) {
		taos->adc_buf_initialized = true;
		for (j = 0; j < ADC_BUFFER_NUM; j++)
			taos->adc_value_buf[j] = value;
	} else
		taos->adc_value_buf[adc_index] = value;

	adc_max = taos->adc_value_buf[0];
	adc_min = taos->adc_value_buf[0];

	for (i = 0; i < ADC_BUFFER_NUM; i++) {
		adc_total += taos->adc_value_buf[i];

		if (adc_max < taos->adc_value_buf[i])
			adc_max = taos->adc_value_buf[i];

		if (adc_min > taos->adc_value_buf[i])
			adc_min = taos->adc_value_buf[i];
	}
	adc_avr_value = (adc_total-(adc_max+adc_min))/(ADC_BUFFER_NUM-2);

	if (taos->adc_index_count == ADC_BUFFER_NUM-1)
		taos->adc_index_count = 0;

	return adc_avr_value;
}

static int factory_adc = 0;
static void taos_work_func_light(struct work_struct *work)
{
	struct taos_data *taos = container_of(work, struct taos_data,
					      work_light);
//	int adc = lightsensor_get_adcvalue(taos);
//	factory_adc = adc;
	int adc=0;
	state_type level_state = LIGHT_INIT;	

	//read value 	
	adc = taos_read_als_data(taos);	
	#if 1 //B1
	gprintk("Optimized adc = %d \n",adc);
	gprintk("cur_state = %d\n",cur_state);
	gprintk("light_enable = %d\n",light_enable);
	#else
	gprintk("taos_work_func_light called adc=[%d]\n", adc);
	#endif

    ///////  Temp code By Arimy for bad board
    if(adc == 0)
    {
        gprintk("taos_read_als_data error -> temp code\n");
        adc = 363;
    }
	
    if(adc >= 15000)
    {
        level_state = LIGHT_LEVEL5;
    }
    else if(adc >= 1500 && adc < 15000)
    {
        level_state = LIGHT_LEVEL4;
    }
    else if(adc >= 150 && adc < 1500)
    {
        level_state = LIGHT_LEVEL3;
    }
    else if(adc >= 15 && adc < 150)
    {
        level_state = LIGHT_LEVEL2;
    }
    else if(adc < 15)
    {
        level_state = LIGHT_LEVEL1;
    }
//	if(!lightsensor_test)		
	{
//		gprintk("backlight_level = %d\n", backlight_level); //Temp
		cur_state = level_state;	
	}
	factory_adc = adc;
	input_report_abs(taos->light_input_dev, ABS_MISC, adc);
	input_sync(taos->light_input_dev);
}

static void taos_work_func_prox(struct work_struct *work)
{
	struct taos_data *taos = container_of(work, struct taos_data,
					      work_prox);
	int i=0;
	int ret= 0;
	u8 prox_int_thresh[4];

	gprintk("\n");

	/* read proximity value. */
	if ((ret = taos_prox_poll(taos, prox_cur_infop)) < 0) 
	{
		gprintk( "call to prox_poll() failed in taos_prox_poll_timer_func()\n");
		return;
	}

	switch ( taos_prox_dist ) /* HERE : taos_prox_dist => PREVIOUS STATUS */
	{
	case TAOS_PRX_DIST_IN:
		
		printk("TAOS_PRX_DIST_IN  ==> OUT \n");

		/* Condition for detecting IN */
		prox_int_thresh[0] = 0;
		prox_int_thresh[1] = 0;
		prox_int_thresh[2] = (TAOS_PRX_THRES_IN)	 	& 0xff;
		prox_int_thresh[3] = (TAOS_PRX_THRES_IN >> 8) 	& 0xff;

		/* Current status */
		taos_prox_dist = TAOS_PRX_DIST_OUT;	  /* HERE : taos_prox_dist => CURRENT STATUS */
		break;
		
	case TAOS_PRX_DIST_OUT:
	case TAOS_PRX_DIST_UNKNOWN:
	default:
		
		printk("TAOS_PRX_DIST_OUT ==> IN \n");
		
		/* Condition for detecting OUT */
		prox_int_thresh[0] = (TAOS_PRX_THRES_OUT) 			& 0xff;
		prox_int_thresh[1] = (TAOS_PRX_THRES_OUT >> 8) 		& 0xff;
		prox_int_thresh[2] = (TAOS_PRX_THRES_MAX) 			& 0xff;
		prox_int_thresh[3] = (TAOS_PRX_THRES_MAX >> 8) 		& 0xff;
		
		/* Current status */
		taos_prox_dist = TAOS_PRX_DIST_IN;	 /* HERE : taos_prox_dist => CURRENT STATUS */
		break;
	}

	for (i = 0; i < 4; i++) 
	{
		ret = taos_i2c_write(taos, (CMD_REG|(PRX_MINTHRESHLO + i)), &prox_int_thresh[i]);
		if ( ret < 0)
		{
			gprintk( "i2c_smbus_write_byte_data failed in taos_chip_on, err = %d\n", ret);
		}
	}	

		/* Interrupt clearing */
	ret = taos_i2c_write_command(taos, (CMD_REG|CMD_SPL_FN|CMD_PROXALS_INTCLR));
	if ( ret < 0)
	{
		gprintk( "i2c_smbus_write_byte_data failed in taos_irq_handler(), err = %d\n", ret);
	}	
	
	/* 0 is close, 1 is far */
	input_report_abs(taos->proximity_input_dev, ABS_DISTANCE, !prox_cur_infop->prox_event);
	input_sync(taos->proximity_input_dev);
	return;
}
/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart taos_timer_func(struct hrtimer *timer)
{
	struct taos_data *taos = container_of(timer, struct taos_data, timer);
	queue_work(taos->wq, &taos->work_light);
	hrtimer_forward_now(&taos->timer, taos->light_poll_delay);
	return HRTIMER_RESTART;
}

/* interrupt happened due to transition/change of near/far proximity state */
irqreturn_t taos_irq_handler(int irq, void *data)
{
	struct taos_data *ip = data;
//gprintk("taos prox int\n");
	queue_work(ip->wq, &ip->work_prox);
	
/*	u8 setting;
	int val = gpio_get_value(ip->pdata->p_out);
	if (val < 0) {
		pr_err("%s: gpio_get_value error %d\n", __func__, val);
		return IRQ_HANDLED;
	}

	if (val != ip->val_state) {
		if (val)
			setting = 0x40;
		else
			setting = 0x20;
		taos_i2c_write(ip, REGS_HYS, &setting);
	}

	ip->val_state = val;
	pr_err("taos: proximity val = %d\n", val);

	
	input_report_abs(ip->proximity_input_dev, ABS_DISTANCE, val);
	input_sync(ip->proximity_input_dev);
*/	
	wake_lock_timeout(&ip->prx_wake_lock, 3*HZ);

	return IRQ_HANDLED;
}

static int taos_setup_irq(struct taos_data *taos)
{
	int rc = -EIO;
	struct gp2a_platform_data *pdata = taos->pdata;
	int irq;

	taos_dbgmsg("start\n");

	rc = gpio_request(pdata->p_out, "gpio_proximity_out");
	if (rc < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
			__func__, pdata->p_out, rc);
		return rc;
	}

	rc = gpio_direction_input(pdata->p_out);
	if (rc < 0) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, pdata->p_out, rc);
		goto err_gpio_direction_input;
	}

	s3c_gpio_setpull(pdata->p_out, S3C_GPIO_PULL_UP);
	
	/* interrupt hander register. */
	taos->irq = -1;
//	irq = GPIO_TAOS_ALS_INT;
	irq = gpio_to_irq(pdata->p_out);
	rc = request_threaded_irq(irq, NULL,
			 taos_irq_handler,
			 /*IRQF_TRIGGER_RISING |*/ IRQF_TRIGGER_FALLING,
			 "proximity_int",
			 taos);
	if (rc < 0) {
		pr_err("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			pdata->p_out, rc);
		goto err_request_irq;
	}

	/* start with interrupts disabled */
	disable_irq(irq);
	taos->irq = irq;

	taos_dbgmsg("success\n");

	goto done;

err_request_irq:
err_gpio_direction_input:
	gpio_free(pdata->p_out);
done:
	return rc;
}

static ssize_t lightsensor_file_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int adc = 0;
	int sum = 0; 	// Temp
	int i = 0;		// Temp
	
//	printk(KERN_ERR "called %s -%d\n",__func__, factory_adc);
/*
	if(!light_enable)
	{
		#if 1 //low-pass filter
		for(i = 0; i < 10; i++)
		{
			adc = taos_read_als_data(); //lightsensor_get_adcvalue();
			msleep(20);
			sum += adc;
		}
		adc = sum/10;
		#else
		adc = taos_read_als_data(); 
		#endif		
		gprintk("called %s	- subdued alarm(adc : %d)\n",__func__,adc);
		return sprintf(buf,"%d\n",adc);
	}
	else
	{
		printk("called %s	- *#0589#\n",__func__);

	return sprintf(buf,"%d\n",cur_adc_value);
	}
*/
	return sprintf(buf, "%d\n", factory_adc);
}

static ssize_t proxsensor_file_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	unsigned int detect = 1;
    int ret = 0;
    struct taos_prox_info prxinfo;
    struct taos_prox_info *prxp = &prxinfo;
    

	ret = taos_prox_poll(taos, prxp);
	
	if(prxp->prox_event)
	{
		detect = 0;
	}

	//detect = gpio_get_value(GPIO_PS_VOUT);
//	detect = 0;
//	printk(KERN_ERR "called %s (%d)\n",__func__, detect);
	return sprintf(buf,"%u\n",detect);
	
/*	
	struct taos_data *ip = dev_get_drvdata(dev);
	int val = gpio_get_value(ip->pdata->p_out);

	//gprintk("called %s \n",__func__);
	printk(KERN_ERR "called %s -%d\n",__func__,val);
	return sprintf(buf,"%u\n",val);
*/	
	
}

static ssize_t taossensor_i2c_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
//	printk("called %s \n",__func__);

	unsigned int i2c_state;

	i2c_state = taos_i2c_err;	

	return sprintf(buf,"%u\n",i2c_state);
}

static ssize_t proxsensor_factory_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",
		       (taos->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}

static ssize_t proxsensor_factory_mode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	bool new_value;
//	printk(KERN_ERR "called %s \n",__func__);
	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&taos->power_lock);
	taos_dbgmsg("new_value = %d, old state = %d\n",
		    new_value, (taos->power_state & PROXIMITY_ENABLED) ? 1 : 0);

	if (new_value) {
		prox_factorymode = 1;
	} else {
		prox_factorymode = 0;
	}

	if (new_value && !(taos->power_state & PROXIMITY_ENABLED)) {
		if (!taos->power_state)
			{
			taos->pdata->power(true);
			taos_chip_on(taos);
			}
		taos->power_state |= PROXIMITY_ENABLED;
		enable_irq(taos->irq);
		enable_irq_wake(taos->irq);
//		taos_i2c_write(taos, REGS_GAIN, &reg_defaults[1]);
//		taos_i2c_write(taos, REGS_HYS, &reg_defaults[2]);
//		taos_i2c_write(taos, REGS_CYCLE, &reg_defaults[3]);
//		taos_i2c_write(taos, REGS_OPMOD, &reg_defaults[4]);
	} else if (!new_value && (taos->power_state & PROXIMITY_ENABLED)) {
		disable_irq_wake(taos->irq);
		disable_irq(taos->irq);
//		taos_i2c_write(taos, REGS_OPMOD, &reg_defaults[0]);
		taos->power_state &= ~PROXIMITY_ENABLED;
		if (!taos->power_state)
			{
			taos_chip_off(taos);			
			taos->pdata->power(false);
			}
	}
	mutex_unlock(&taos->power_lock);
	return size;
}
static DEVICE_ATTR(proxsensor_factory_mode,0644, proxsensor_factory_mode_show, 
	proxsensor_factory_mode_store);

static DEVICE_ATTR(taossensor_i2c_state,0644, taossensor_i2c_state_show, NULL);
static DEVICE_ATTR(lightsensor_file_state, 0644, lightsensor_file_state_show,
	NULL);
static DEVICE_ATTR(proxsensor_file_state,0644, proxsensor_file_state_show, NULL);

/*
void proximity_jack_control(int on)
{
	struct taos_data *taos = g_taos_data;
	bool new_value = on;

    if (taos == NULL)
        return 0;
    
	mutex_lock(&taos->power_lock);
	taos_dbgmsg("%s :: %d >> %d\n", __func__, 
		    new_value, (taos->power_state & PROXIMITY_ENABLED) ? 1 : 0);
	
	if (new_value && !(taos->power_state & PROXIMITY_ENABLED)) {
		if (!taos->power_state)
			taos->pdata->power(true);
		taos->power_state |= PROXIMITY_ENABLED;
		enable_irq(taos->irq);
		enable_irq_wake(taos->irq);
		taos_i2c_write(taos, REGS_GAIN, &reg_defaults[1]);
		taos_i2c_write(taos, REGS_HYS, &reg_defaults[2]);
		taos_i2c_write(taos, REGS_CYCLE, &reg_defaults[3]);
		taos_i2c_write(taos, REGS_OPMOD, &reg_defaults[4]);
	} else if (!new_value && (taos->power_state & PROXIMITY_ENABLED)) {
		disable_irq_wake(taos->irq);
		disable_irq(taos->irq);
		taos_i2c_write(taos, REGS_OPMOD, &reg_defaults[0]);
		taos->power_state &= ~PROXIMITY_ENABLED;
		if (!taos->power_state)
			taos->pdata->power(false);
	}
	mutex_unlock(&taos->power_lock);

	return;
}
*/

static const struct file_operations light_fops = {
	.owner  = THIS_MODULE,
};

static struct miscdevice light_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "light",
    .fops   = &light_fops,
};

static int taos_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct input_dev *input_dev;
	struct taos_data *taos;
	struct gp2a_platform_data *pdata = client->dev.platform_data;
//printk(KERN_ERR "taos_i2c_probe\n");
	if (!pdata) {
		pr_err("%s: missing pdata!\n", __func__);
		return ret;
	}
	if (!pdata->power || !pdata->light_adc_value) {
		pr_err("%s: incomplete pdata!\n", __func__);
		return ret;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		return ret;
	}

	taos = kzalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (!taos) {
		pr_err("%s: failed to alloc memory for module data\n",
		       __func__);
		return -ENOMEM;
	}

	g_taos_data = taos;

	taos->pdata = pdata;
	taos->i2c_client = client;
	i2c_set_clientdata(client, taos);

	/* wake lock init */
	wake_lock_init(&taos->prx_wake_lock, WAKE_LOCK_SUSPEND,
		       "prx_wake_lock");
	mutex_init(&taos->power_lock);

	ret = taos_setup_irq(taos);
	if (ret) {
		pr_err("%s: could not setup irq\n", __func__);
		goto err_setup_irq;
	}

	/* allocate proximity input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		goto err_input_allocate_device_proximity;
	}
	taos->proximity_input_dev = input_dev;
	input_set_drvdata(input_dev, taos);
	input_dev->name = "proximity_sensor";
	input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	taos_dbgmsg("registering proximity input device\n");
	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_proximity;
	}
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &proximity_attribute_group);
	if (ret) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_proximity;
	}

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&taos->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	taos->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	taos->timer.function = taos_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	taos->wq = create_singlethread_workqueue("taos_wq");
	if (!taos->wq) {
		ret = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}
	/* this is the thread function we run on the work queue */
	INIT_WORK(&taos->work_light, taos_work_func_light);
	INIT_WORK(&taos->work_prox, taos_work_func_prox);	

	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(input_dev, taos);
	input_dev->name = "light_sensor";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

	taos_dbgmsg("registering lightsensor-level input device\n");
	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	taos->light_input_dev = input_dev;
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &light_attribute_group);
	if (ret) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	/* set sysfs for light sensor */

	ret = misc_register(&light_device);
	if (ret)
		pr_err(KERN_ERR "misc_register failed - light\n");

	taos->lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(taos->lightsensor_class))
		pr_err("Failed to create class(lightsensor)!\n");

	taos->switch_cmd_dev = device_create(taos->lightsensor_class,
		NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(taos->switch_cmd_dev))
		pr_err("Failed to create device(switch_cmd_dev)!\n");

	if (device_create_file(taos->switch_cmd_dev,
		&dev_attr_lightsensor_file_state) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_lightsensor_file_state.attr.name);

	taos->proxsensor_class = class_create(THIS_MODULE, "proxsensor");
	if (IS_ERR(taos->proxsensor_class))
		pr_err("Failed to create class(proxsensor)!\n");	
	
	taos->switch_cmd_dev = device_create(taos->proxsensor_class,
		NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(taos->switch_cmd_dev))
		pr_err("Failed to create device(switch_cmd_dev)!\n");	
	
	if (device_create_file(taos->switch_cmd_dev, &dev_attr_proxsensor_file_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_proxsensor_file_state.attr.name);
	if (device_create_file(taos->switch_cmd_dev, &dev_attr_taossensor_i2c_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_taossensor_i2c_state.attr.name);


	if (device_create_file(taos->switch_cmd_dev, &dev_attr_proxsensor_factory_mode) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_proxsensor_factory_mode.attr.name);
		

	
	dev_set_drvdata(taos->switch_cmd_dev, taos);

	input_report_abs(taos->proximity_input_dev, ABS_DISTANCE, 1);
	input_sync(taos->proximity_input_dev);

	goto done;

	/* error, unwind it all */
err_sysfs_create_group_light:
	input_unregister_device(taos->light_input_dev);
err_input_register_device_light:
err_input_allocate_device_light:
	destroy_workqueue(taos->wq);
err_create_workqueue:
	sysfs_remove_group(&taos->proximity_input_dev->dev.kobj,
			   &proximity_attribute_group);
err_sysfs_create_group_proximity:
	input_unregister_device(taos->proximity_input_dev);
err_input_register_device_proximity:
err_input_allocate_device_proximity:
	free_irq(taos->irq, 0);
	gpio_free(taos->pdata->p_out);
err_setup_irq:
	mutex_destroy(&taos->power_lock);
	wake_lock_destroy(&taos->prx_wake_lock);
	kfree(taos);
done:
	return ret;
}

static int taos_suspend(struct device *dev)
{
	/* We disable power only if proximity is disabled.  If proximity
	   is enabled, we leave power on because proximity is allowed
	   to wake up device.  We remove power without changing
	   taos->power_state because we use that state in resume.
	*/
	struct i2c_client *client = to_i2c_client(dev);
	struct taos_data *taos = i2c_get_clientdata(client);
	if (taos->power_state & LIGHT_ENABLED)
		taos_light_disable(taos);
	if (taos->power_state == LIGHT_ENABLED)
		{
		taos_chip_off(taos);		
		taos->pdata->power(false);
		}
	return 0;
}

static int taos_resume(struct device *dev)
{
	/* Turn power back on if we were before suspend. */
	struct i2c_client *client = to_i2c_client(dev);
	struct taos_data *taos = i2c_get_clientdata(client);
	if (taos->power_state == LIGHT_ENABLED)
		{
		taos->pdata->power(true);
		taos_chip_on(taos);
		}
	if (taos->power_state & LIGHT_ENABLED)
		taos_light_enable(taos);
	return 0;
}

static int taos_i2c_remove(struct i2c_client *client)
{
	struct taos_data *taos = i2c_get_clientdata(client);
	sysfs_remove_group(&taos->light_input_dev->dev.kobj,
			   &light_attribute_group);
	input_unregister_device(taos->light_input_dev);
	sysfs_remove_group(&taos->proximity_input_dev->dev.kobj,
			   &proximity_attribute_group);
	input_unregister_device(taos->proximity_input_dev);
	free_irq(taos->irq, NULL);
	gpio_free(taos->pdata->p_out);
	if (taos->power_state) {
		if (taos->power_state & LIGHT_ENABLED)
			taos_light_disable(taos);
		taos->pdata->power(false);
	}
	destroy_workqueue(taos->wq);
	mutex_destroy(&taos->power_lock);
	wake_lock_destroy(&taos->prx_wake_lock);
	kfree(taos);
	return 0;
}

static const struct i2c_device_id taos_device_id[] = {
	{"taos", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, taos_device_id);

static const struct dev_pm_ops taos_pm_ops = {
	.suspend = taos_suspend,
	.resume = taos_resume
};

static struct i2c_driver taos_i2c_driver = {
	.driver = {
		.name = "taos",
		.owner = THIS_MODULE,
		.pm = &taos_pm_ops
	},
	.probe		= taos_i2c_probe,
	.remove		= taos_i2c_remove,
	.id_table	= taos_device_id,
};

extern unsigned int HWREV;
#define HWREV_TAOS 5
static int __init taos_init(void)
{
	int ret;

    if(HWREV > HWREV_TAOS)
    {
        printk("No taos Device %d\n", HWREV);
        return -1;	   
    }

     ret = i2c_add_driver(&taos_i2c_driver);	

	return ret;
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&taos_i2c_driver);
}
/////////////
/*
 	 * ALS_DATA - request for current ambient light data. If correctly
 	 * enabled and valid data is available at the device, the function
 	 * for lux conversion is called, and the result returned if valid.
*/
//#define	ADC_BUFFER_NUM			8
static int adc_value_buf[ADC_BUFFER_NUM] = {0, 0, 0, 0, 0, 0};
static int taos_read_als_data(struct taos_data *taos)
{
		int ret = 0;
		int lux_val=0;
		
		u8 reg_val;
		#if 1 // D1 -average filter	
		int i=0;
		int j=0;
		unsigned int adc_total = 0;
		static int adc_avr_value = 0;
		unsigned int adc_index = 0;
		static unsigned int adc_index_count = 0;
		unsigned int adc_max = 0;
		unsigned int adc_min = 0;
		#endif
	
		if ((ret = taos_i2c_read(taos, (CMD_REG | CNTRL), &reg_val,1))< 0)
		{
			gprintk( "opt_i2c_read to cmd reg failed in taos_read_als_data\n");			
			return 0; //r0 return (ret);
		}

		if ((reg_val & (CNTL_ADC_ENBL | CNTL_PWRON)) != (CNTL_ADC_ENBL | CNTL_PWRON))
		{
		    gprintk("taos PowerOn error [%x]\n", reg_val);
			return 0; //r0 return (-ENODATA);
		}
		if ((lux_val = taos_get_lux(taos)) < 0)
		{
			gprintk( "taos_get_lux() returned error %d in taos_read_als_data\n", lux_val);
		}

//		gprintk("** Returned lux_val  = [%d]\n", lux_val);

		/* QUICK FIX : multiplu lux_val by 4 to make the value similar to ADC value. */
		lux_val = lux_val * 5;
		
		if (lux_val >= MAX_LUX)
		{
			lux_val = MAX_LUX; //return (MAX_LUX);
		}
		
		cur_adc_value = lux_val; 
//		gprintk("taos_read_als_data cur_adc_value = %d (0x%x)\n", cur_adc_value, cur_adc_value);

		#if 1 // D1 -average filter		
			adc_index = (adc_index_count++)%ADC_BUFFER_NUM;		

			if(cur_state == LIGHT_INIT) //ADC buffer initialize (light sensor off ---> light sensor on)
			{
				for(j = 0; j<ADC_BUFFER_NUM; j++)
					adc_value_buf[j] = lux_val; // value;
			}
		    else
		    {
		    	adc_value_buf[adc_index] = lux_val; //value;
			}
			
			adc_max = adc_value_buf[0];
			adc_min = adc_value_buf[0];

			for(i = 0; i <ADC_BUFFER_NUM; i++)
			{
				adc_total += adc_value_buf[i];

				if(adc_max < adc_value_buf[i])
					adc_max = adc_value_buf[i];
							
				if(adc_min > adc_value_buf[i])
					adc_min = adc_value_buf[i];
			}
			adc_avr_value = (adc_total-(adc_max+adc_min))/(ADC_BUFFER_NUM-2);
			
			if(adc_index_count == ADC_BUFFER_NUM-1)
				adc_index_count = 0;

			return adc_avr_value;
		#else
			return (lux_val);
		#endif

}


static int taos_prox_on(struct taos_data *taos)
{
    int ret = 0;
    int fail_num = 0;
    u8 temp_val;
	u8 reg_cntrl;

    if(proximity_enable)
        {
        printk( "proximity_enable is already on!\n");
        return 0;
        }
   
		reg_cntrl = CNTL_INTPROXPON_ENBL ; 
		if ((ret = taos_i2c_write(taos,(CMD_REG|CNTRL), &reg_cntrl))< 0)
		{
			printk( "opt_i2c_write to ctrl reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}
		mdelay(20); // more than 12 ms

		/* interrupt clearing */
		ret = taos_i2c_write_command(taos,(CMD_REG|CMD_SPL_FN|CMD_PROXALS_INTCLR));
		if ( ret < 0)
		{
			printk( "opt_taos_i2c_write_command failed in taos_chip_on, err = %d\n", ret);
			fail_num++; // 
		}

		
		if ( fail_num == 0) 
		{			
//printk("[%s] INT\n",__func__);
			/* interrupt enable */
			//enable_irq(GPIO_TAOS_ALS_INT);	
			enable_irq(taos->irq);
			enable_irq_wake(taos->irq);
			proximity_enable = 1;
        }
   		else
		{
			printk( "I2C failed in taos_prox_on\n");
		}

        return ret;
}

static int taos_prox_off(struct taos_data *taos)
{
    int ret = 0;
    int fail_num = 0;
    u8 temp_val;
	u8 reg_cntrl;
//printk("[%s] \n",__func__);
    if(!proximity_enable)
        {
        printk( "proximity_enable is already off!\n");
        return 0;
        }
    
		reg_cntrl = CNTL_ALSPON_ENBL;//CNTL_INTPROXPON_ENBL ; 
		if ((ret = taos_i2c_write(taos,(CMD_REG|CNTRL), &reg_cntrl))< 0)
		{
			printk( "opt_i2c_write to ctrl reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}
                
		if ( fail_num == 0) 
		{			
//		printk("\n");
			/* interrupt disable */
//            disable_irq(GPIO_TAOS_ALS_INT);
		disable_irq_wake(taos->irq);
		disable_irq(taos->irq);
			proximity_enable = 0;
        }
   		else
		{
			printk( "I2C failed in taos_prox_off\n");
		}

        return ret;
}

/*
 	 * ALS_ON - called to set the device in ambient light sense mode.
 	 * configured values of light integration time, inititial interrupt
 	 * filter, gain, and interrupt thresholds (if interrupt driven) are
 	 * initialized. Then power, adc, (and interrupt if needed) are enabled.
*/

static int taos_chip_on(struct taos_data *taos)
{
	int i = 0;
	int ret = 0;		
	int num_try_init = 0;
	int fail_num = 0;
	u8 temp_val;
	u8 reg_cntrl;
	u8 prox_int_thresh[4];
//printk("[%s] \n",__func__);
	for ( num_try_init=0; num_try_init<3 ; num_try_init ++)
	{
		fail_num = 0;
	
		temp_val = 	CNTL_REG_CLEAR;
		if ((ret = taos_i2c_write(taos,(CMD_REG|CNTRL),&temp_val))< 0)
		{
			gprintk( "opt_i2c_write to clr ctrl reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}		

		//Changed by Bob Stricklin 10/21/2010
		temp_val = 0xed; //working temp_val = 0xde;//temp_val = 50 mS Intergration time;
		if ((ret = taos_i2c_write(taos,(CMD_REG|ALS_TIME), &temp_val))< 0)
		{
			gprintk( "opt_i2c_write to als time reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}

		temp_val = 0xff; //Not working temp_val = 0xff; //temp_val = 0xee;
		if ((ret = taos_i2c_write(taos,(CMD_REG|PRX_TIME), &temp_val))< 0) 			
		{
			gprintk( "opt_i2c_write to prx time reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}

		temp_val =0xff; //temp_val =0xf2;
		if ((ret = taos_i2c_write(taos,(CMD_REG|WAIT_TIME), &temp_val)) < 0)			
		{
			gprintk( "opt_i2c_write to wait time reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}		

		temp_val = 0x33; // or 0x30
		if ((ret = taos_i2c_write(taos,(CMD_REG|INTERRUPT), &temp_val)) < 0)			
		{
			gprintk( "opt_i2c_write to interrupt reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}			

		temp_val = 0x0;
		if ((ret = taos_i2c_write(taos,(CMD_REG|PRX_CFG), &temp_val)) < 0)
		{
			gprintk( "opt_i2c_write to prox cfg reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}		
		
		temp_val = 0x0a;//0x06;  //0x0a; 
		if ((ret = taos_i2c_write(taos,(CMD_REG|PRX_COUNT), &temp_val)) < 0)			
		{
			gprintk( "opt_i2c_write to prox cnt reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}
				
		//Changed by Bob Stricklin 10/21/2010
		temp_val =0x20;   //Gain = 1X
		if ((ret = taos_i2c_write(taos,(CMD_REG|GAIN), &temp_val)) < 0)			
		{
			gprintk( "opt_i2c_write to prox gain reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}

		/* Setting for proximity interrupt */			
		prox_int_thresh[0] = 0;
		prox_int_thresh[1] = 0;
		prox_int_thresh[2] = (TAOS_PRX_THRES_IN)	 	& 0xff;
		prox_int_thresh[3] = (TAOS_PRX_THRES_IN >> 8) 	& 0xff;

		
		for (i = 0; i < 4; i++) 
		{
			ret = taos_i2c_write(taos,(CMD_REG|(PRX_MINTHRESHLO + i)), &prox_int_thresh[i]);
			if ( ret < 0)
			{
				gprintk( "opt_i2c_write failed in taos_chip_on, err = %d\n", ret);
				fail_num++; // 
			}
		}				
				
		reg_cntrl = CNTL_ALSPON_ENBL;//CNTL_INTPROXPON_ENBL ; 
		if ((ret = taos_i2c_write(taos,(CMD_REG|CNTRL), &reg_cntrl))< 0)
		{
			gprintk( "opt_i2c_write to ctrl reg failed in taos_chip_on.\n");
			fail_num++; // return (ret);
		}
		mdelay(20); // more than 12 ms

		/* interrupt clearing */
/*        
		ret = opt_taos_i2c_write_command((CMD_REG|CMD_SPL_FN|CMD_PROXALS_INTCLR));
		if ( ret < 0)
		{
			gprintk( "opt_taos_i2c_write_command failed in taos_chip_on, err = %d\n", ret);
			fail_num++; // 
		}
*/		
		if ( fail_num == 0) 
		{			
		gprintk("\n");
			/* interrupt enable */
//			enable_irq(GPIO_TAOS_ALS_INT);			
//			proximity_enable = 1;
			taos_chip_status = TAOS_CHIP_WORKING;
			
			break;	// Init Success 
		}
		else
		{
			printk( "I2C failed in taos_chip_on, # of trial=[%d], # of fail I2C=[%d]\n", num_try_init, fail_num);
		}
	}
gprintk("\n");			
	return (ret);


}


/*
 	 * PROX_ON - called to set the device in proximity sense mode. After
 	 * clearing device, als and proximity integration times are set, as
 	 * well as the wait time, interrupt filter, pulse count, and gain.
 	 * If in interrupt driven mode, interrupt thresholds are set. Then
 	 * the power, prox, (and interrupt, if needed) bits are enabled. If
 	 * the mode is polled, then the polling kernel timer is started.
*/
/*
 	 * ALS_OFF - called to stop ambient light sense mode of operation.
 	 * Clears the filter history, and clears the control register.
*/
static int taos_chip_off(struct taos_data *taos)
{
		int ret = 0;
		u8 reg_cntrl;

//printk("[%s] \n",__func__);

		#if 1 // working interrupt
		/* interrupt disable */
//		disable_irq(GPIO_TAOS_ALS_INT);
		#endif

		reg_cntrl = CNTL_REG_CLEAR;
		
		if ((ret = taos_i2c_write(taos,(CMD_REG | CNTRL), &reg_cntrl))< 0)
		{
			gprintk( "opt_i2c_write to ctrl reg failed in taos_chip_off\n");
			return (ret);
		}
		
		proximity_enable = 0;
		taos_chip_status = TAOS_CHIP_SLEEP;
			
		return (ret);
}


/*
 * Reads and calculates current lux value. The raw ch0 and ch1 values of the
 * ambient light sensed in the last integration cycle are read from the device.
 * Time scale factor array values are adjusted based on the integration time.
 * The raw values are multiplied by a scale factor, and device gain is obtained
 * using gain index. Limit checks are done next, then the ratio of a multiple
 * of ch1 value, to the ch0 value, is calculated. The array triton_lux_datap
 * declared above is then scanned to find the first ratio value that is just
 * above the ratio we just calculated. The ch0 and ch1 multiplier constants in
 * the array are then used along with the time scale factor array values, to
 * calculate the milli-lux value, which is then scaled down to lux.
 */
static int taos_get_lux(struct taos_data *taos)
{
	int i=0;
	int ret = 0;
	u8 chdata[4];

	int irdata = 0;
	int cleardata = 0;	
	int irdata_4 = 0;
	int ratio_for_lux = 0;
	int irfactor = 0;
	int calculated_lux = 0;
 // Changed by Bob Stricklin 10/24/10 to Integration time = 50 mS  Gain to 1X but these are not used anyplace in the code.
 int integration_time = 50; //working int integration_time = 100; // settingsettingsettingsettingsettingsettingsettingsettingsettingsettingsettingsetting
int als_gain = 1   ;// settingsettingsettingsettingsettingsettingsettingsettingsettingsettingsettingsetting
	for (i = 0; i < 4; i++) 
	{
		if ((ret = taos_i2c_read(taos,(CMD_REG |(ALS_CHAN0LO + i)),&chdata[i],1))< 0)
		{
			gprintk( "opt_i2c_read() to (CMD_REG |(ALS_CHAN0LO + i) regs failed in taos_get_lux()\n");
			return 0; //r0 return (ret);
		}
	}
	
	cleardata = 256 * chdata[1] + chdata[0];
	irdata = 256 * chdata[3] + chdata[2];
// Changed next 33 lines by Bob Stricklin 10/25/10 to add in more Lux Equations
if(cleardata > 0)
ratio_for_lux = (irdata * 100)  /  (cleardata );
else{
ratio_for_lux = 106;
calculated_lux =1;
}
if(ratio_for_lux < 10){
    calculated_lux =(( 262 * cleardata) - (529 * irdata))/100;
    ratio_for_lux = 101;
}
if(ratio_for_lux < 20){
    calculated_lux =(( 300 * cleardata) - (44 * irdata))/100;
    ratio_for_lux = 102;
}
if(ratio_for_lux < 30){
    calculated_lux =(( 800 * cleardata) - (250 * irdata))/100;
    ratio_for_lux = 103;
}
if(ratio_for_lux < 40){
    calculated_lux =(( 398 * cleardata) - (99 * irdata))/100;
    ratio_for_lux = 104;
}
if(ratio_for_lux < 46){
    calculated_lux =(( 100 * cleardata) - (186 * irdata))/110;
    ratio_for_lux = 105;
}
if(ratio_for_lux < 100){
	if(cleardata > 16000){
		irdata_4 = irdata/4;
		calculated_lux =(((( 4 * irdata_4 * irdata_4)/100000) + ((83 * irdata_4)/400) - 50)*16)/10;
       }
	else
		calculated_lux =(( 100 * cleardata) - (186 * irdata))/125;
    ratio_for_lux = 106;
}
#if 0 // TAOS Debug
int myratio = (irdata * 100)  /  (cleardata);
printk( "chdata[10] = ,%2x%2x, chdata[32]= ,%2x%2x,  cleardata=,%d, irdata=,%d, ratio_for_lux=,%d, myratio=%d, calculated_lux=,%d\n",chdata[1],chdata[0],chdata[3],chdata[2],cleardata,irdata,ratio_for_lux,myratio,calculated_lux);
//looks like the gain register is not programed properly so adding a register dump here to view the contents of Gain Register.
//int taos_register_dump(void);
#endif

    // Check values for over and underflow
    if(calculated_lux > MAX_LUX) calculated_lux = MAX_LUX; // Case where lux is over the top limit which is 32768
    if(calculated_lux < 1) calculated_lux = 1;             // Never lets lux go to zero or negative

	return (calculated_lux);
}


/*
 * Proximity poll function - if valid data is available, read and form the ch0
 * and prox data values, check for limits on the ch0 value, and check the prox
 * data against the current thresholds, to set the event status accordingly.
 */
static int taos_prox_poll(struct taos_data *taos, struct taos_prox_info *prxp)
{
	int i;
	int ret = 0;
	int event = 0;
	
	u8 status;
	u8 chdata[6];

	//gprintk("\n");	
	
	if ((ret = taos_i2c_read(taos,(CMD_REG | STATUS), &status,1))< 0)
	{
		gprintk( "opt_i2c_read() to (CMD_REG | STATUS) regs failed in taos_prox_poll()\n");
		return (ret);
	}	

	for (i = 0; i < 6; i++) 
	{
		if ((ret = taos_i2c_read(taos,(CMD_REG |(ALS_CHAN0LO + i)),&chdata[i],1))< 0)
		{
			gprintk( "opt_i2c_read() to (CMD_REG |(ALS_CHAN0LO + i) regs failed in taos_prox_poll()\n");
			return (ret);
		}
	}
	
	prxp->prox_data = chdata[5];
	prxp->prox_data <<= 8;
	prxp->prox_data |= chdata[4];

	gprintk("prxp->prox_data                      =                  [%d (0x%x)]\n", (int) prxp->prox_data, (int) prxp->prox_data);
	//printk("prxp->prox_data=[%d (0x%x)]\n", (int) prxp->prox_data, (int) prxp->prox_data);

	if ( prxp->prox_data > TAOS_PRX_THRES_IN)
	{
		event = 1;
	}
    else if( prxp->prox_data < TAOS_PRX_THRES_OUT)
	{
		event = 0;
	}
    else
    {
        if(taos_prox_dist == TAOS_PRX_DIST_IN)
            event = 1;
        else
            event = 0;
    }
		
	prxp->prox_event = event;

	return (ret);
}


/////////////
module_init(taos_init);
module_exit(taos_exit);

MODULE_AUTHOR("mjchen@sta.samsung.com");
MODULE_DESCRIPTION("Optical Sensor driver for taosp002a00f");
MODULE_LICENSE("GPL");
