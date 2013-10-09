/*
 * linux/drivers/power/s3c6410_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DRIVER_NAME	"sec-battery"

/*
 * Battery Table
 */
#define BATT_CAL		2447	/* 3.60V */

#define BATT_MAXIMUM		406	/* 4.176V */
#define BATT_FULL		353	/* 4.10V  */
#define BATT_SAFE_RECHARGE	353	/* 4.10V */
#define BATT_ALMOST_FULL	188	/* 3.8641V */
#define BATT_HIGH		112	/* 3.7554V */
#define BATT_MED		66	/* 3.6907V */
#define BATT_LOW		43	/* 3.6566V */
#define BATT_CRITICAL		8	/* 3.6037V */
#define BATT_MINIMUM		(-28)	/* 3.554V */
#define BATT_OFF		(-128)	/* 3.4029V */

/*
 * ADC channel
 */
enum adc_channel_type{
	S3C_ADC_VOLTAGE = 0,
	S3C_ADC_CHG_CURRENT = 2,
	S3C_ADC_EAR = 3,
	S3C_ADC_TEMPERATURE = 6,
	S3C_ADC_V_F,
	ENDOFADC
};

enum {
	BATT_VOL = 0,
	BATT_VOL_ADC,
	BATT_TEMP,
	BATT_TEMP_ADC,
	BATT_TEMP_RADC,
	BATT_CURRENT_ADC,
	BATT_CHARGING_SOURCE,
	BATT_FG_SOC,
	BATT_FG_PSOC,
	BATT_RESET_SOC,
	CHARGING_MODE_BOOTING,
	BATT_TEMP_CHECK,
	BATT_FULL_CHECK,
	BATT_TEMP_ADC_SPEC,
	BATT_TEST_VALUE,
	BATT_VOICE_CALL_2G,
	BATT_VOICE_CALL_3G,
	BATT_DMB_PLAY,
	BATT_MUSIC_PLAY,
	BATT_VIDEO_PLAY,
	BATT_CAMERA_USE,
	BATT_INTERNET_USE,
	BATT_BOOTING,
	BATT_HW_REVISION,
	BATT_ESUS_TEST,
	BATT_TYPE,
};

enum {
	BAT_NOT_DETECTED,
	BAT_DETECTED
};

#define TOTAL_CHARGING_TIME	(5*60*60)	/* 5 hours */
#define TOTAL_RECHARGING_TIME	  (2*60*60)	/* 2 hours */

#define COMPENSATE_VIBRATOR		19
#define COMPENSATE_CAMERA		25
#define COMPENSATE_MP3			17
#define COMPENSATE_VIDEO		28
#define COMPENSATE_VOICE_CALL_2G	13
#define COMPENSATE_VOICE_CALL_3G	14
#define COMPENSATE_DATA_CALL		25
#define COMPENSATE_LCD			0
#define COMPENSATE_TA			0
#define COMPENSATE_CAM_FALSH		0
#define COMPENSATE_BOOTING		52

#define SOC_LB_FOR_POWER_OFF		27

#define RECHARGE_COND_VOLTAGE		4110000
#define FULL_CHARGE_COND_VOLTAGE    4000000

//#define RECHARGE_COND_TIME		(30*1000)	/* 30 seconds */

#define BAT_DET_CNT			3
#define FULL_CHG_COND_COUNT 		4

#define CURRENT_OF_FULL_CHG         316     /* 170mA => (code*1.5)mV , refer to code table. */ 

//#define ADC_12BIT_RESOLUTION        8056    /* 3300mV/4096 = 0.805664063, * scale factor */
//#define ADC_12BIT_SCALE             10000   /* scale factor */
//#define ADC_CURRENT_FACTOR          15 /* 1mA = 1.5mV */

/* abnormal temperature charging blocking adc */
#define EVENT_HIGH_BLOCK_TEMP_ADC	385
#define HIGH_BLOCK_TEMP_ADC			380
#define HIGH_RECOVER_TEMP_ADC		358
#define LOW_BLOCK_TEMP_ADC			249
#define LOW_RECOVER_TEMP_ADC		263

/* abnormal temperature charging blocking exception */
#define CALL_TEMP_EXCEPT_BIT        0
#define DMB_TEMP_EXCEPT_BIT         1
#define MUSIC_TEMP_EXCEPT_BIT       2
#define VIDEO_TEMP_EXCEPT_BIT       3
#define CAMERA_TEMP_EXCEPT_BIT      4
#define INTERNEL_TEMP_EXCEPT_BIT    5
#define DOCK_TEMP_EXCEPT_BIT    	6