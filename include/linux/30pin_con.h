/*
 * Copyright (C) 2008 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_ACC_CONN_H
#define __ASM_ARCH_ACC_CONN_H

enum accessory_type {
	ACCESSORY_NONE = 0,
	ACCESSORY_CARMOUNT,
	ACCESSORY_LINEOUT,
	ACCESSORY_UNKNOWN,
};

enum dock_type {
	DOCK_NONE = 0,
	DOCK_DESK,
	DOCK_KEYBOARD,
};

struct acc_con_info {
	struct device *acc_dev;
	struct acc_con_platform_data *pdata;
	struct delayed_work acc_con_work;
	struct delayed_work acc_ID_work;
	enum accessory_type current_accessory;
	enum dock_type current_dock;
	int accessory_irq;
	int dock_irq;
	int mhl_irq;
	int mhl_factory_mode;
	int accessory_id_vol;
	int jig_connected;
};

struct acc_con_platform_data {
	void (*otg_en) (int active);
	void (*acc_power) (int active);
	void (*usb_ldo_en) (int active);
	void (*dock_cb) (bool attached);
	void (*cfg_gpio) (void);
	int     accessory_irq_gpio;
	int     dock_irq_gpio;
	int     mhl_irq_gpio;
	int     hdmi_hpd_gpio;
};

#endif
