

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/30pin_con.h>
#include <linux/switch.h>

#include <asm/irq.h>
#include <mach/adc.h>

#define SUBJECT "30PIN_CON"

#define ACC_CONDEV_DBG(format, ...) \
	printk(KERN_DEBUG "[ "SUBJECT " (%s) ] " format "\n", \
		__func__, ## __VA_ARGS__);

#define ACCESSORY_ID 			7
#define DETECTION_INTR_DELAY	(get_jiffies_64() + (HZ*15)) /* 20s */



extern unsigned int HWREV;

#ifdef CONFIG_MHL_SII9234
#include "sii9234.h"
extern void TVout_LDO_ctrl(int enable);
#endif

static struct acc_con_info *g_acc_info = NULL;

static int connector_detect_change(void)
{
	int i;
	u32 adc = 0, adc_sum = 0;
	u32 adc_buff[5] = {0};
	u32 adc_min = 0;
	u32 adc_max = 0;

	for (i = 0; i < 5; i++) {
		/*change this reading ADC function  */
		adc_buff[i] = s3c_adc_get_adc_data(ACCESSORY_ID);;
		adc_sum += adc_buff[i];
		if (i == 0) {
			adc_min = adc_buff[0];
			adc_max = adc_buff[0];
		} else {
			if (adc_max < adc_buff[i])
				adc_max = adc_buff[i];
			else if (adc_min > adc_buff[i])
				adc_min = adc_buff[i];
		}
		msleep(20);
	}
	adc = (adc_sum - adc_max - adc_min)/3;
	//ACC_CONDEV_DBG("ACCESSORY_ID : ADC value = %d\n", adc);
	return (int)adc;
}

#define ADC_12BIT_RESOLUTION        8056    /* 3300mV/4096 = 0.805664063, * scale factor */
#define ADC_12BIT_SCALE             10000   /* scale factor */

static unsigned int acc_get_accessory_id_vol(void)
{
	int acc_adc = 0;
	unsigned long acc_id_temp = 0;
	unsigned long acc_vol = 0;

	acc_adc = connector_detect_change();

	acc_id_temp = acc_adc * ADC_12BIT_RESOLUTION;
    acc_vol = acc_id_temp / ADC_12BIT_SCALE;
    if((acc_id_temp%ADC_12BIT_SCALE) >= (ADC_12BIT_SCALE/2))
    {
        acc_vol+=1;
    }

	return acc_vol;
}

static void acc_jig_check(struct acc_con_info *acc, int acc_adc)	
{
	unsigned long acc_id_temp = 0;
	unsigned long acc_vol = 0;
	
	if(acc_adc)
	{
		acc_id_temp = acc_adc * ADC_12BIT_RESOLUTION;
	    acc_vol = acc_id_temp / ADC_12BIT_SCALE;
	    if((acc_id_temp%ADC_12BIT_SCALE) >= (ADC_12BIT_SCALE/2))
	    {
	        acc_vol+=1;
	    }
		acc->accessory_id_vol = acc_vol;
		
		if((2600<acc_vol) && (3000>acc_vol)) //Jig Cable
		{
			acc->jig_connected = 1;
		} else
		{
			acc->jig_connected = 0;
		}
	}
	else
	{
		acc->jig_connected = 0;
	}
}

static ssize_t MHD_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res;
//	TVout_LDO_ctrl(true);
	if(!MHD_HW_IsOn())
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
		MHD_HW_Off();		
	}
	else
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
	}
	
	count = sprintf(buf,"%d\n", res );
//	TVout_LDO_ctrl(false);
	return count;
}

static ssize_t MHD_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}


static ssize_t acc_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct acc_con_info *acc = dev_get_drvdata(dev);

	int count;
	int connected = 0;	
	if (acc->current_dock == DOCK_DESK)
		connected |= (0x1<<0);
	else if (acc->current_dock == DOCK_KEYBOARD) 
		connected |= (0x1<<1);
	
	if (acc->current_accessory == ACCESSORY_CARMOUNT)
		connected |= (0x1<<2);
	else if (acc->current_accessory == ACCESSORY_LINEOUT)
		connected |= (0x1<<4);

	if (gpio_get_value(acc->pdata->hdmi_hpd_gpio))
		connected |= (0x1<<6);

	count = sprintf(buf,"%d\n", connected );
	//ACC_CONDEV_DBG("%x",connected);
	return count;
}

static ssize_t acc_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}


static ssize_t MHD_factory_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct acc_con_info *acc = dev_get_drvdata(dev);

	printk("called %s \n", __func__);

	return sprintf(buf,"%u\n", acc->mhl_factory_mode);
}

static ssize_t MHD_factory_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct acc_con_info *acc = dev_get_drvdata(dev);
	int value;
	
	sscanf(buf, "%d", &value);
	
	printk(KERN_INFO "[MHD] Set Factory Mode = %d \n", value);

	if(value==1) {
		acc->mhl_factory_mode = 1;
		msleep(200);
	}
	else {
		acc->mhl_factory_mode = 0;
	}

	return size;
}

static ssize_t acc_id_vol_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct acc_con_info *acc = dev_get_drvdata(dev);
	int count;

	acc->accessory_id_vol = acc_get_accessory_id_vol();
	count = sprintf(buf,"%d\n", acc->accessory_id_vol);

	return count;
}

static ssize_t acc_id_vol_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//printk("input data --> %s\n", buf);

	return size;
}

static ssize_t acc_jig_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct acc_con_info *acc = dev_get_drvdata(dev);
	int count;
	int adc_val = 0;

	adc_val = connector_detect_change();
	acc_jig_check(acc, adc_val);
	count = sprintf(buf,"%d\n", acc->jig_connected);

	return count;
}

static ssize_t acc_jig_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//printk("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(acc_file, S_IRUGO, acc_check_read, acc_check_write);

static DEVICE_ATTR(MHD_file, S_IRUGO, MHD_check_read, MHD_check_write);

static DEVICE_ATTR(MHD_factory, S_IRGRP |S_IWGRP | S_IRUSR | S_IWUSR,
	MHD_factory_read, MHD_factory_write);

static DEVICE_ATTR(accessory_id_vol, S_IRUGO, 	acc_id_vol_read, acc_id_vol_write);

static DEVICE_ATTR(acc_jig_status, S_IRUGO, 	acc_jig_read, acc_jig_write);

/*
static void acc_dock_check(struct acc_con_info *acc, bool connected)
{
	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	if (acc->current_dock == DOCK_KEYBOARD)
		env_ptr = "DOCK=keyboard";
	else if (acc->current_dock == DOCK_DESK)
		env_ptr = "DOCK=desk";
	else
		env_ptr = "DOCK=unknown";

	if (!connected) {
		stat_ptr = "STATE=offline";
		acc->current_dock = DOCK_NONE;
	} else {
		stat_ptr = "STATE=online";
	}

	envp[0] = env_ptr;
	envp[1] = stat_ptr;
	envp[2] = NULL;
	kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
	ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
}
*/

void acc_con_intr_handle(struct work_struct *_work)
{
	struct acc_con_info *acc;
	int cur_state;

	acc = container_of(_work, struct acc_con_info, acc_con_work.work);

	ACC_CONDEV_DBG("");
	
	cur_state = gpio_get_value(acc->pdata->dock_irq_gpio);
	if (cur_state == 1) {
		if (acc->current_dock == DOCK_NONE) {
			ACC_CONDEV_DBG("docking station detached : enable irq");
			enable_irq(acc->dock_irq);
			return;
		}

		ACC_CONDEV_DBG("docking station detached!!!");

		if (acc->pdata->dock_cb)
			acc->pdata->dock_cb(false);
		
		/* current_dock state is cleared inside dock_cb func. */

	} else if (0 == cur_state) {
		ACC_CONDEV_DBG("docking station attached!!!");

		if (acc->current_dock == DOCK_DESK) {
			ACC_CONDEV_DBG("dock state is same as previous one. ignore!!!");
			enable_irq(acc->dock_irq);
			return;
		}

		acc->current_dock = DOCK_DESK;
		
		if (acc->pdata->dock_cb)
			acc->pdata->dock_cb(true);
	}
	enable_irq(acc->dock_irq);
}

irqreturn_t acc_con_interrupt(int irq, void *ptr)
{
	struct acc_con_info *acc = ptr;
	ACC_CONDEV_DBG("");
	disable_irq_nosync(irq);
	schedule_delayed_work(&acc->acc_con_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static void acc_con_interrupt_init(struct acc_con_info *acc)
{
	int ret;
	ACC_CONDEV_DBG("");

	acc->dock_irq = gpio_to_irq(acc->pdata->dock_irq_gpio);
	ret = request_irq(acc->dock_irq, acc_con_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"dock_detect", acc);
	if (ret) {
		ACC_CONDEV_DBG("request_irq(dock_irq) return : %d\n", ret);
	}

	ACC_CONDEV_DBG("GPIO_DOCK_INT : default value is %d ",
		gpio_get_value(acc->pdata->dock_irq_gpio));

	/* start with interrupt disabled */
	disable_irq(acc->dock_irq);
}

void acc_notified(struct acc_con_info *acc, int acc_adc)
{
#if 0	/* TODO */
	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	if (acc_adc != false) {
		if ((2100 < acc_adc) && (2300 > acc_adc)) {
			env_ptr = "ACCESSORY=OTG";
			acc->current_accessory = ACCESSORY_OTG;
		} else if ((900 < acc_adc) && (1100 > acc_adc)) {
			env_ptr = "ACCESSORY=lineout";
			acc->current_accessory = ACCESSORY_LINEOUT;
		} else if ((1300 < acc_adc) && (1500 > acc_adc)) {
			env_ptr = "ACCESSORY=carmount";
			acc->current_accessory = ACCESSORY_CARMOUNT;
		} else {
			env_ptr = "ACCESSORY=unknown";
			acc->current_accessory = ACCESSORY_UNKNOWN;
		}
		stat_ptr = "STATE=online";
		envp[0] = env_ptr;
		envp[1] = stat_ptr;
		envp[2] = NULL;
		if (acc->current_accessory == ACCESSORY_OTG) {
			if (acc->pdata->usb_ldo_en)
				acc->pdata->usb_ldo_en(1);
			if (acc->pdata->otg_en)
				acc->pdata->otg_en(1);
		}
		kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
		ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
	} else {
		if (acc->current_accessory == ACCESSORY_OTG)
			env_ptr = "ACCESSORY=OTG";
		else if (acc->current_accessory == ACCESSORY_LINEOUT)
			env_ptr = "ACCESSORY=lineout";
		else if (acc->current_accessory == ACCESSORY_CARMOUNT)
			env_ptr = "ACCESSORY=carmount";
		else
			env_ptr = "ACCESSORY=unknown";

		if ((acc->current_accessory == ACCESSORY_OTG) &&
			acc->pdata->otg_en)
			acc->pdata->otg_en(0);
		else {
			if (acc->pdata->acc_power)
				acc->pdata->acc_power(0);
		}

		acc->current_accessory = ACCESSORY_NONE;
		stat_ptr = "STATE=offline";
		envp[0] = env_ptr;
		envp[1] = stat_ptr;
		envp[2] = NULL;
		kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
		ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
	}
#endif
}

void acc_ID_intr_handle(struct work_struct *_work)
{
	struct acc_con_info *acc;
	int acc_ID_val, adc_val;

	ACC_CONDEV_DBG("");
	acc = container_of(_work, struct acc_con_info, acc_ID_work.work);
	acc_ID_val = gpio_get_value(acc->pdata->accessory_irq_gpio);
	ACC_CONDEV_DBG("GPIO_ACCESSORY_INT is %d", acc_ID_val);

	/* toggle the level, if HWREV is below revision 6 */
	if(HWREV < 6) {
		pr_info("HWREV is %d : toggle Accessory intr %d", HWREV, acc_ID_val);
		acc_ID_val != acc_ID_val;
	}

	if (acc_ID_val == 1) {
		ACC_CONDEV_DBG("Accessory detached");

		acc_notified(acc, false);
		set_irq_type(acc->accessory_irq, IRQ_TYPE_EDGE_FALLING);
	} else if (acc_ID_val == 0) {
		msleep(420); /* workaround for jack */
		ACC_CONDEV_DBG("Accessory attached");
		
		adc_val = connector_detect_change();
		acc_notified(acc, adc_val);
		set_irq_type(acc->accessory_irq, IRQ_TYPE_EDGE_RISING);
	}
	enable_irq(acc->accessory_irq);
}

irqreturn_t acc_ID_interrupt(int irq, void *ptr)
{
	struct acc_con_info *acc = ptr;
	ACC_CONDEV_DBG("");
	disable_irq_nosync(irq);
	schedule_delayed_work(&acc->acc_ID_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

void acc_ID_interrupt_init(struct acc_con_info *acc)
{
	int ret;

	ACC_CONDEV_DBG("");

	acc->accessory_irq = gpio_to_irq(acc->pdata->accessory_irq_gpio);
	ret = request_irq(acc->accessory_irq, acc_ID_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"accessory_detect", acc);
	if (ret) {
		ACC_CONDEV_DBG("request_irq(accessory_irq) return : %d\n", ret);
	}
	
	ACC_CONDEV_DBG("GPIO_ACCESSORY_INT : default value is %d (HWREV>=6, active low)",
		gpio_get_value(acc->pdata->accessory_irq_gpio));

	/* start with interrupt disabled */
	disable_irq(acc->accessory_irq);
}

static int acc_con_probe(struct platform_device *pdev)
{
	struct acc_con_info *acc;
	struct acc_con_platform_data *pdata = pdev->dev.platform_data;
	int	retval;

	ACC_CONDEV_DBG("");

	if (pdata == NULL) {
		pr_err("%s: no pdata\n", __func__);
		return -ENODEV;
	}

	acc = kzalloc(sizeof(struct acc_con_info), GFP_KERNEL);
	if (!acc)
		return -ENOMEM;

	acc->pdata = pdata;
	acc->current_dock = DOCK_NONE;
	acc->current_accessory = ACCESSORY_NONE;
	acc->mhl_irq = gpio_to_irq(pdata->mhl_irq_gpio);

	if (acc->pdata->cfg_gpio)
		acc->pdata->cfg_gpio();

	dev_set_drvdata(&pdev->dev, acc);

	acc->acc_dev = &pdev->dev;

#ifdef CONFIG_MHL_SII9234
	retval = i2c_add_driver(&SII9234A_i2c_driver);
	if (retval)
		pr_err("[MHL SII9234A] can't add i2c driver\n");
	else
		pr_info("[MHL SII9234A] add i2c driver\n");

	retval = i2c_add_driver(&SII9234B_i2c_driver);
	if (retval)
		pr_err("[MHL SII9234B] can't add i2c driver\n");
	else
		pr_info("[MHL SII9234B] add i2c driver\n");

	retval = i2c_add_driver(&SII9234C_i2c_driver);
	if (retval)
		pr_err("[MHL SII9234C] can't add i2c driver\n");
	else
		pr_info("[MHL SII9234C] add i2c driver\n");

	retval = i2c_add_driver(&SII9234_i2c_driver);
	if (retval)
		pr_err("[MHL SII9234] can't add i2c driver\n");
	else
		pr_info("[MHL SII9234] add i2c driver\n");

	/*MHD_HW_Off(); */
	/*sii9234_tpi_init(); */
	/*MHD_Read_deviceID(); */
#endif	

	if (device_create_file(acc->acc_dev, &dev_attr_MHD_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_MHD_file.attr.name);

	if (device_create_file(acc->acc_dev, &dev_attr_acc_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_acc_file.attr.name);

	if (device_create_file(acc->acc_dev, &dev_attr_MHD_factory) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_MHD_factory.attr.name);

	if (device_create_file(acc->acc_dev, &dev_attr_accessory_id_vol) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_accessory_id_vol.attr.name);

	if (device_create_file(acc->acc_dev, &dev_attr_acc_jig_status) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_acc_jig_status.attr.name);

	INIT_DELAYED_WORK(&acc->acc_con_work, acc_con_intr_handle);
	acc_con_interrupt_init(acc);

	INIT_DELAYED_WORK(&acc->acc_ID_work, acc_ID_intr_handle);
	acc_ID_interrupt_init(acc);

	g_acc_info = acc;

	/* run initially to determine initial dock/accessory state */
	schedule_delayed_work(&acc->acc_con_work, msecs_to_jiffies(0));
	schedule_delayed_work(&acc->acc_ID_work, msecs_to_jiffies(0));
	return 0;
}

static int acc_con_remove(struct platform_device *pdev)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");
#ifdef CONFIG_MHL_SII9234
	i2c_del_driver(&SII9234A_i2c_driver);
	i2c_del_driver(&SII9234B_i2c_driver);
	i2c_del_driver(&SII9234C_i2c_driver);
	i2c_del_driver(&SII9234_i2c_driver);
#endif	
	cancel_delayed_work_sync(&acc->acc_con_work);
	cancel_delayed_work_sync(&acc->acc_ID_work);
	
	kfree(acc);
	return 0;
}

static int acc_con_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");
	disable_irq(acc->dock_irq);
#if 0	/* TVOUT is controlled by charger callback */	
#ifdef CONFIG_MHL_SII9234
	if (acc->current_dock == DOCK_DESK)
		MHD_HW_Off();   /*call MHL deinit */
#endif
#endif
	return 0;
}

static int acc_con_resume(struct platform_device *pdev)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	int cur_state;
	ACC_CONDEV_DBG("");
#if 0	/* TVOUT is controlled by charger callback */		
#ifdef CONFIG_MHL_SII9234
	if (acc->current_dock == DOCK_DESK)
		sii9234_tpi_init();  /* call MHL init */
#endif
#endif
	schedule_delayed_work(&acc->acc_con_work, msecs_to_jiffies(0));
	cur_state = gpio_get_value(acc->pdata->dock_irq_gpio);
	ACC_CONDEV_DBG("dock_state = %d dock_irq = %d",
		acc->current_dock, cur_state);
	return 0;
}

static struct platform_driver acc_con_driver = {
	.probe		= acc_con_probe,
	.remove		= acc_con_remove,
	.suspend	= acc_con_suspend,
	.resume		= acc_con_resume,
	.driver		= {
		.name		= "acc_con",
		.owner		= THIS_MODULE,
	},
};

static int __init acc_con_init(void)
{
	ACC_CONDEV_DBG("");
	return platform_driver_register(&acc_con_driver);
}

static void __exit acc_con_exit(void)
{
	platform_driver_unregister(&acc_con_driver);
}

late_initcall(acc_con_init);
module_exit(acc_con_exit);

MODULE_AUTHOR("Kyungrok Min <gyoungrok.min@samsung.com>");
MODULE_DESCRIPTION("acc connector driver");
MODULE_LICENSE("GPL");
