/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
/* *****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kgd1spXXmipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 * Author:
 * -------
 * Dream Yeh (MTK08783)
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
/* #include <asm/system.h> */
/* #include <linux/xlog.h> */

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5kgd1sp_mipi_raw.h"

/*WDR auto ration mode*/
/* #define ENABLE_WDR_AUTO_RATION */
#define OFF_READY_DELAY 25
/****************************Modify following Strings for debug****************************/
#define PFX "s5kgd1sp_camera_sensor"
#define LOG_1 LOG_INF("s5kgd1sp,MIPI 4LANE\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(fmt, args...)	pr_err(PFX "[%s] " fmt, __func__, ##args)
/* static int first_flag = 1; */
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KGD1SP_SENSOR_ID,

	.checksum_value = 0x318134c,	/*Check by Dream */

	.pre = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
		#if 0
	.cap = {
		.pclk = 3092909760,	/*record different mode's pclk */
		.linelength = 20144,	/*record different mode's linelength */
		.framelength = 5118,	/*record different mode's framelength */
		.startx = 0,	/*record different mode's startx of grabwindow */
		.starty = 0,	/*record different mode's starty of grabwindow */
		.grabwindow_width = 6560,	/*record different mode's width of grabwindow */
		.grabwindow_height = 4920,	/*record different mode's height of grabwindow */
		/*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 85,
		/*       following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
		},
		#endif
		.cap = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
	.normal_video = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
	.hs_video = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
	.slim_video = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
	.margin = 6,
	.min_shutter = 2,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 5,	/* support sensor mode num */

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 0,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,	/* SENSOR_OUTPUT_FORMAT_RAW_Gr, */
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	/* .i2c_speed = 100, */
	.i2c_addr_table = {0x20, 0xff},
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 0,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */

	.hdr_mode = KAL_FALSE,	/* HDR Mode : 0: disable HDR, 1:IHDR, 2:HDR, 9:ZHDR */

	.i2c_write_id = 0x20,
};

/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
{6560, 4920, 0, 0, 6560, 4920, 3264, 2448, 0000, 0000, 3264, 2448, 0, 0, 3264, 2448},	/* Preview */
/*{6560, 4920, 0, 0, 6560, 4920, 6560, 4920, 0000, 0000, 6560, 4920, 0, 0, 6560, 4920},*/	/* capture */
{6560, 4920, 0, 0, 6560, 4920, 3264, 2448, 0000, 0000, 3264, 2448, 0, 0, 3264, 2448},	/* capture */
{6560, 4920, 0, 0, 6560, 4920, 3264, 2448, 0000, 0000, 3264, 2448, 0, 0, 3264, 2448},	/* video */
{6560, 4920, 0, 0, 6560, 4920, 3264, 2448, 0000, 0000, 3264, 2448, 0, 0, 3264, 2448},	/* hight speed video */
{6560, 4920, 0, 0, 6560, 4920, 3264, 2448, 0000, 0000, 3264, 2448, 0, 0, 3264, 2448}
};				/* slim video */





static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	LOG_INF("frame_length = %d, line_length = %d\n",
		imgsensor.frame_length,	imgsensor.line_length);

	write_cmos_sensor(0x0340, imgsensor.frame_length);
	/* write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF); */
	write_cmos_sensor(0x0342, imgsensor.line_length);
	/* write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF); */

}				/*      set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
}



static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/* kal_int16 dummy_line; */
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable = %d\n",
				framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
	    (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	/* kal_uint32 frame_length = 0; */


	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter =
	    (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
			? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0340, imgsensor.frame_length);
			/* write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF); */
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length);
		/* write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF); */
	}

	/* Update Shutter */
	/* write_cmos_sensor(0x0104, 0x01);   //group hold */
	write_cmos_sensor(0x0202, shutter);
	/* write_cmos_sensor(0x0203, shutter & 0xFF); */
	/*write_cmos_sensor(0x021E, shutter);*/
	/* write_cmos_sensor(0x0104, 0x00);   //group hold */

	LOG_INF("shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}				/*      write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	/* LOG_INF("shutter =%d\n", shutter); */
	write_shutter(shutter);
}				/*      set_shutter */


static void hdr_write_shutter(kal_uint16 le, kal_uint16 se)
{
	unsigned int iRation;
	unsigned long flags;
	/* kal_uint16 realtime_fps = 0; */
	/* kal_uint32 frame_length = 0; */
	/* LOG_INF("enter xxxx  set_shutter, shutter =%d\n", shutter); */
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = le;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	if (!le)
		le = 1;		/*avoid 0 */

	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);

	le = (le < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : le;
	le = (le > (imgsensor_info.max_frame_length - imgsensor_info.margin))
			? (imgsensor_info.max_frame_length - imgsensor_info.margin) : le;

	/* Frame length :4000 C340 */
	/* write_cmos_sensor(0x6028,0x4000); */
	/* write_cmos_sensor(0x602A,0xC340 ); */
	write_cmos_sensor(0x0340, imgsensor.frame_length);

	/* SET LE/SE ration */
	/* iRation = (((LE + SE/2)/SE) >> 1 ) << 1 ; */
	iRation = ((10 * le / se) + 5) / 10;
	if (iRation < 2)
		iRation = 1;
	else if (iRation < 4)
		iRation = 2;
	else if (iRation < 8)
		iRation = 4;
	else if (iRation < 16)
		iRation = 8;
	else if (iRation < 32)
		iRation = 16;
	else
		iRation = 1;

	/*set ration for auto */
	iRation = 0x100 * iRation;
#if defined(ENABLE_WDR_AUTO_RATION)
	/*LE / SE ration ,  0x218/0x21a =  LE Ration */
	/*0x218 =0x400, 0x21a=0x100, LE/SE = 4x */
	write_cmos_sensor(0x0218, iRation);
	write_cmos_sensor(0x021a, 0x100);
#endif
	/*Short exposure */
	write_cmos_sensor(0x0202, se);
	/*Log exposure ratio */
	write_cmos_sensor(0x021e, le);

	LOG_INF("HDR set shutter LE=%d, SE=%d, iRation=0x%x\n", le, se, iRation);

}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X  */
	/* [4:9] = M meams M X       */
	/* Total gain = M + N /16 X   */

	/*  */
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

	reg_gain = gain >> 1;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n", gain, reg_gain);

	/* write_cmos_sensor_8(0x0104, 0x01); */
	write_cmos_sensor(0x0204, reg_gain);
	/*write_cmos_sensor(0x0220, reg_gain);*/

	/* write_cmos_sensor_8(0x0104, 0x00); */

	return gain;
}				/*      set_gain  */


/* defined but not used */
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {

		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length = imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;


		/* Extend frame length first */

		write_cmos_sensor(0x0340, imgsensor.frame_length);	/* or 0x380e? */


		write_cmos_sensor(0x602A, 0x021e);
		write_cmos_sensor(0x6f12, le);
		write_cmos_sensor(0x602A, 0x0202);
		write_cmos_sensor(0x6f12, se);


		set_gain(gain);
	}

}


#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor_8(0x0101, 0x00);	/* Gr */
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor_8(0x0101, 0x01);	/* R */
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor_8(0x0101, 0x02);	/* B */
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor_8(0x0101, 0x03);	/* Gb */
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}
#endif

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/



static void sensor_init(void)
{
	LOG_INF("E\n");
#if 0
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0000, 0x0002);
	write_cmos_sensor(0x0000, 0x0841);
	write_cmos_sensor(0x6010, 0x0001);
	write_cmos_sensor(0x6014, 0x0001);
	mDELAY(10);

	write_cmos_sensor(0x6242, 0x0E00);
write_cmos_sensor(0x6028, 0x2001);
write_cmos_sensor(0x602A, 0x518C);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0549);
write_cmos_sensor(0x6F12, 0x0448);
write_cmos_sensor(0x6F12, 0x054A);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0xD006);
write_cmos_sensor(0x6F12, 0x101A);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xD406);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB5BD);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x6100);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x68C0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0xF800);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xFF4F);
write_cmos_sensor(0x6F12, 0x0446);
write_cmos_sensor(0x6F12, 0x0878);
write_cmos_sensor(0x6F12, 0x85B0);
write_cmos_sensor(0x6F12, 0x9246);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x67D0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x52FE);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x63D0);
write_cmos_sensor(0x6F12, 0x0027);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x4DFE);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x61D0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x4EFE);
write_cmos_sensor(0x6F12, 0x060A);
write_cmos_sensor(0x6F12, 0xB7FA);
write_cmos_sensor(0x6F12, 0x87F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2005);
write_cmos_sensor(0x6F12, 0x092D);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0925);
write_cmos_sensor(0x6F12, 0x093D);
write_cmos_sensor(0x6F12, 0xEF40);
write_cmos_sensor(0x6F12, 0xB6FA);
write_cmos_sensor(0x6F12, 0x86F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2003);
write_cmos_sensor(0x6F12, 0x07F0);
write_cmos_sensor(0x6F12, 0xFF09);
write_cmos_sensor(0x6F12, 0x092B);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0923);
write_cmos_sensor(0x6F12, 0x093B);
write_cmos_sensor(0x6F12, 0xDE40);
write_cmos_sensor(0x6F12, 0x06F0);
write_cmos_sensor(0x6F12, 0xFF08);
write_cmos_sensor(0x6F12, 0x0021);
write_cmos_sensor(0x6F12, 0xEB46);
write_cmos_sensor(0x6F12, 0xFE4A);
write_cmos_sensor(0x6F12, 0x0898);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x8000);
write_cmos_sensor(0x6F12, 0x02EB);
write_cmos_sensor(0x6F12, 0xC00C);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x8100);
write_cmos_sensor(0x6F12, 0x0CEB);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x4306);
write_cmos_sensor(0x6F12, 0x36F9);
write_cmos_sensor(0x6F12, 0x822F);
write_cmos_sensor(0x6F12, 0xB6F9);
write_cmos_sensor(0x6F12, 0x0270);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x4506);
write_cmos_sensor(0x6F12, 0xBF1A);
write_cmos_sensor(0x6F12, 0xB6F9);
write_cmos_sensor(0x6F12, 0x0A00);
write_cmos_sensor(0x6F12, 0xB6F9);
write_cmos_sensor(0x6F12, 0x0C60);
write_cmos_sensor(0x6F12, 0x07FB);
write_cmos_sensor(0x6F12, 0x08F7);
write_cmos_sensor(0x6F12, 0x8037);
write_cmos_sensor(0x6F12, 0x361A);
write_cmos_sensor(0x6F12, 0x06FB);
write_cmos_sensor(0x6F12, 0x09F6);
write_cmos_sensor(0x6F12, 0x8036);
write_cmos_sensor(0x6F12, 0x02EB);
write_cmos_sensor(0x6F12, 0x2722);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x2620);
write_cmos_sensor(0x6F12, 0x1044);
write_cmos_sensor(0x6F12, 0x4BF8);
write_cmos_sensor(0x6F12, 0x2100);
write_cmos_sensor(0x6F12, 0x00FA);
write_cmos_sensor(0x6F12, 0x0AF0);
write_cmos_sensor(0x6F12, 0x4BF8);
write_cmos_sensor(0x6F12, 0x2100);
write_cmos_sensor(0x6F12, 0x491C);
write_cmos_sensor(0x6F12, 0x0429);
write_cmos_sensor(0x6F12, 0xD9D3);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x0099);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0401);
write_cmos_sensor(0x6F12, 0x0199);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0401);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0801);
write_cmos_sensor(0x6F12, 0x0299);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0801);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0C01);
write_cmos_sensor(0x6F12, 0x0399);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0C01);
write_cmos_sensor(0x6F12, 0x09B0);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF08F);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF1FD);
write_cmos_sensor(0x6F12, 0x060A);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF3FD);
write_cmos_sensor(0x6F12, 0x070A);
write_cmos_sensor(0x6F12, 0x99E7);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xF843);
write_cmos_sensor(0x6F12, 0x1C46);
write_cmos_sensor(0x6F12, 0x0093);
write_cmos_sensor(0x6F12, 0x0E46);
write_cmos_sensor(0x6F12, 0x1546);
write_cmos_sensor(0x6F12, 0x1346);
write_cmos_sensor(0x6F12, 0x0746);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xD449);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xEAFD);
write_cmos_sensor(0x6F12, 0xD249);
write_cmos_sensor(0x6F12, 0x2B46);
write_cmos_sensor(0x6F12, 0x0422);
write_cmos_sensor(0x6F12, 0x1831);
write_cmos_sensor(0x6F12, 0x3846);
write_cmos_sensor(0x6F12, 0x0094);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xE2FD);
write_cmos_sensor(0x6F12, 0xCE49);
write_cmos_sensor(0x6F12, 0x2B46);
write_cmos_sensor(0x6F12, 0x0822);
write_cmos_sensor(0x6F12, 0x3031);
write_cmos_sensor(0x6F12, 0x3846);
write_cmos_sensor(0x6F12, 0x0094);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xDAFD);
write_cmos_sensor(0x6F12, 0x96F8);
write_cmos_sensor(0x6F12, 0x5A02);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x7DD0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC0FD);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x79D0);
write_cmos_sensor(0x6F12, 0x4FF0);
write_cmos_sensor(0x6F12, 0x0009);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xBAFD);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x74D0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xBBFD);
write_cmos_sensor(0x6F12, 0x070A);
write_cmos_sensor(0x6F12, 0xB7FA);
write_cmos_sensor(0x6F12, 0x87F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2006);
write_cmos_sensor(0x6F12, 0x092E);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0926);
write_cmos_sensor(0x6F12, 0xB9FA);
write_cmos_sensor(0x6F12, 0x89F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2005);
write_cmos_sensor(0x6F12, 0x093E);
write_cmos_sensor(0x6F12, 0x092D);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0925);
write_cmos_sensor(0x6F12, 0xF740);
write_cmos_sensor(0x6F12, 0x093D);
write_cmos_sensor(0x6F12, 0x29FA);
write_cmos_sensor(0x6F12, 0x05F0);
write_cmos_sensor(0x6F12, 0x07F0);
write_cmos_sensor(0x6F12, 0xFF08);
write_cmos_sensor(0x6F12, 0xC7B2);
write_cmos_sensor(0x6F12, 0xDFF8);
write_cmos_sensor(0x6F12, 0xD892);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x8002);
write_cmos_sensor(0x6F12, 0x09EB);
write_cmos_sensor(0x6F12, 0x4201);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4603);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x2221);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x24C1);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4503);
write_cmos_sensor(0x6F12, 0xACEB);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x33F8);
write_cmos_sensor(0x6F12, 0xFA1F);
write_cmos_sensor(0x6F12, 0x0CFB);
write_cmos_sensor(0x6F12, 0x08FC);
write_cmos_sensor(0x6F12, 0x5B88);
write_cmos_sensor(0x6F12, 0x0CF1);
write_cmos_sensor(0x6F12, 0x800C);
write_cmos_sensor(0x6F12, 0x5B1A);
write_cmos_sensor(0x6F12, 0x7B43);
write_cmos_sensor(0x6F12, 0x8033);
write_cmos_sensor(0x6F12, 0x02EB);
write_cmos_sensor(0x6F12, 0x2C22);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x2321);
write_cmos_sensor(0x6F12, 0x5143);
write_cmos_sensor(0x6F12, 0x54F8);
write_cmos_sensor(0x6F12, 0x2020);
write_cmos_sensor(0x6F12, 0x090B);
write_cmos_sensor(0x6F12, 0x4A43);
write_cmos_sensor(0x6F12, 0x110B);
write_cmos_sensor(0x6F12, 0x44F8);
write_cmos_sensor(0x6F12, 0x2010);
write_cmos_sensor(0x6F12, 0x401C);
write_cmos_sensor(0x6F12, 0x0428);
write_cmos_sensor(0x6F12, 0xD8D3);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x8002);
write_cmos_sensor(0x6F12, 0x09EB);
write_cmos_sensor(0x6F12, 0x4201);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4603);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0xC221);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0xC4C1);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4503);
write_cmos_sensor(0x6F12, 0xACEB);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x9A11);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x9C31);
write_cmos_sensor(0x6F12, 0x0CFB);
write_cmos_sensor(0x6F12, 0x08FC);
write_cmos_sensor(0x6F12, 0x5B1A);
write_cmos_sensor(0x6F12, 0x7B43);
write_cmos_sensor(0x6F12, 0x0CF1);
write_cmos_sensor(0x6F12, 0x800C);
write_cmos_sensor(0x6F12, 0x8033);
write_cmos_sensor(0x6F12, 0x02EB);
write_cmos_sensor(0x6F12, 0x2C22);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x2321);
write_cmos_sensor(0x6F12, 0x5143);
write_cmos_sensor(0x6F12, 0x0A0B);
write_cmos_sensor(0x6F12, 0x04EB);
write_cmos_sensor(0x6F12, 0x8001);
write_cmos_sensor(0x6F12, 0x401C);
write_cmos_sensor(0x6F12, 0x0B69);
write_cmos_sensor(0x6F12, 0x5343);
write_cmos_sensor(0x6F12, 0x1A0B);
write_cmos_sensor(0x6F12, 0x0A61);
write_cmos_sensor(0x6F12, 0x0428);
write_cmos_sensor(0x6F12, 0xD7D3);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x4AFD);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x2BD1);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x8002);
write_cmos_sensor(0x6F12, 0x01E0);
write_cmos_sensor(0x6F12, 0x26E0);
write_cmos_sensor(0x6F12, 0x27E0);
write_cmos_sensor(0x6F12, 0x09EB);
write_cmos_sensor(0x6F12, 0x4201);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4603);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x7221);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x74C1);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4503);
write_cmos_sensor(0x6F12, 0xACEB);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x4A11);
write_cmos_sensor(0x6F12, 0xB3F8);
write_cmos_sensor(0x6F12, 0x4C31);
write_cmos_sensor(0x6F12, 0x0CFB);
write_cmos_sensor(0x6F12, 0x08FC);
write_cmos_sensor(0x6F12, 0x5B1A);
write_cmos_sensor(0x6F12, 0x7B43);
write_cmos_sensor(0x6F12, 0x0CF1);
write_cmos_sensor(0x6F12, 0x800C);
write_cmos_sensor(0x6F12, 0x8033);
write_cmos_sensor(0x6F12, 0x02EB);
write_cmos_sensor(0x6F12, 0x2C22);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x2321);
write_cmos_sensor(0x6F12, 0x5143);
write_cmos_sensor(0x6F12, 0x0A0B);
write_cmos_sensor(0x6F12, 0x04EB);
write_cmos_sensor(0x6F12, 0x8001);
write_cmos_sensor(0x6F12, 0x401C);
write_cmos_sensor(0x6F12, 0x0B6A);
write_cmos_sensor(0x6F12, 0x5343);
write_cmos_sensor(0x6F12, 0x1A0B);
write_cmos_sensor(0x6F12, 0x0A62);
write_cmos_sensor(0x6F12, 0x0428);
write_cmos_sensor(0x6F12, 0xD4D3);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF883);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x22FD);
write_cmos_sensor(0x6F12, 0x070A);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x24FD);
write_cmos_sensor(0x6F12, 0x4FEA);
write_cmos_sensor(0x6F12, 0x1029);
write_cmos_sensor(0x6F12, 0x5CE7);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xF34F);
write_cmos_sensor(0x6F12, 0xA7B0);
write_cmos_sensor(0x6F12, 0x0D46);
write_cmos_sensor(0x6F12, 0x24A9);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x23FD);
write_cmos_sensor(0x6F12, 0x24A9);
write_cmos_sensor(0x6F12, 0x0220);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1FFD);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x0446);
write_cmos_sensor(0x6F12, 0x6B49);
write_cmos_sensor(0x6F12, 0x1D90);
write_cmos_sensor(0x6F12, 0x05EB);
write_cmos_sensor(0x6F12, 0xC500);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x8000);
write_cmos_sensor(0x6F12, 0x2690);
write_cmos_sensor(0x6F12, 0x05EB);
write_cmos_sensor(0x6F12, 0x4500);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4002);
write_cmos_sensor(0x6F12, 0x2592);
write_cmos_sensor(0x6F12, 0x644F);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0x2552);
write_cmos_sensor(0x6F12, 0x5543);
write_cmos_sensor(0x6F12, 0x07EB);
write_cmos_sensor(0x6F12, 0x850B);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0xC00A);
write_cmos_sensor(0x6F12, 0x2798);
write_cmos_sensor(0x6F12, 0x28B9);
write_cmos_sensor(0x6F12, 0x3819);
write_cmos_sensor(0x6F12, 0x00F5);
write_cmos_sensor(0x6F12, 0x0050);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0x5809);
write_cmos_sensor(0x6F12, 0x00B1);
write_cmos_sensor(0x6F12, 0x0120);
write_cmos_sensor(0x6F12, 0x8146);
write_cmos_sensor(0x6F12, 0x2025);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xE3FC);
write_cmos_sensor(0x6F12, 0x012C);
write_cmos_sensor(0x6F12, 0x02D0);
write_cmos_sensor(0x6F12, 0x022C);
write_cmos_sensor(0x6F12, 0x06D0);
write_cmos_sensor(0x6F12, 0x08E0);
write_cmos_sensor(0x6F12, 0x0228);
write_cmos_sensor(0x6F12, 0x01D0);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x04D1);
write_cmos_sensor(0x6F12, 0x4025);
write_cmos_sensor(0x6F12, 0x02E0);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x00D1);
write_cmos_sensor(0x6F12, 0x8025);
write_cmos_sensor(0x6F12, 0x0AEB);
write_cmos_sensor(0x6F12, 0xC406);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0xB713);
write_cmos_sensor(0x6F12, 0x706C);
write_cmos_sensor(0x6F12, 0x6343);
write_cmos_sensor(0x6F12, 0x0BEB);
write_cmos_sensor(0x6F12, 0x8303);
write_cmos_sensor(0x6F12, 0xC288);
write_cmos_sensor(0x6F12, 0x4188);
write_cmos_sensor(0x6F12, 0x43F2);
write_cmos_sensor(0x6F12, 0x7800);
write_cmos_sensor(0x6F12, 0x1844);
write_cmos_sensor(0x6F12, 0x8046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xE3FC);
write_cmos_sensor(0x6F12, 0x02A8);
write_cmos_sensor(0x6F12, 0xCDE9);
write_cmos_sensor(0x6F12, 0x0090);
write_cmos_sensor(0x6F12, 0x4146);
write_cmos_sensor(0x6F12, 0x1DAA);
write_cmos_sensor(0x6F12, 0x2846);
write_cmos_sensor(0x6F12, 0x736C);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xDFFC);
write_cmos_sensor(0x6F12, 0xB6F8);
write_cmos_sensor(0x6F12, 0x4010);
write_cmos_sensor(0x6F12, 0x02A8);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xDFFC);
write_cmos_sensor(0x6F12, 0x16A8);
write_cmos_sensor(0x6F12, 0x0090);
write_cmos_sensor(0x6F12, 0x97F8);
write_cmos_sensor(0x6F12, 0xC323);
write_cmos_sensor(0x6F12, 0x4B46);
write_cmos_sensor(0x6F12, 0x4349);
write_cmos_sensor(0x6F12, 0x2846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xDBFC);
write_cmos_sensor(0x6F12, 0x2598);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x4406);
write_cmos_sensor(0x6F12, 0x16A8);
write_cmos_sensor(0x6F12, 0x36F8);
write_cmos_sensor(0x6F12, 0x581F);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xD8FC);
write_cmos_sensor(0x6F12, 0x4FF0);
write_cmos_sensor(0x6F12, 0xFF30);
write_cmos_sensor(0x6F12, 0x3188);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xD8FC);
write_cmos_sensor(0x6F12, 0x1EA9);
write_cmos_sensor(0x6F12, 0x97F8);
write_cmos_sensor(0x6F12, 0xC303);
write_cmos_sensor(0x6F12, 0xCDE9);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x4B46);
write_cmos_sensor(0x6F12, 0x394A);
write_cmos_sensor(0x6F12, 0x2946);
write_cmos_sensor(0x6F12, 0x1DA8);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xD2FC);
write_cmos_sensor(0x6F12, 0x2698);
write_cmos_sensor(0x6F12, 0x04EB);
write_cmos_sensor(0x6F12, 0x4401);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x8106);
write_cmos_sensor(0x6F12, 0x1EA8);
write_cmos_sensor(0x6F12, 0x36F8);
write_cmos_sensor(0x6F12, 0x6C1F);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xCDFC);
write_cmos_sensor(0x6F12, 0x21A9);
write_cmos_sensor(0x6F12, 0x97F8);
write_cmos_sensor(0x6F12, 0xC303);
write_cmos_sensor(0x6F12, 0xCDE9);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x4B46);
write_cmos_sensor(0x6F12, 0x2E4A);
write_cmos_sensor(0x6F12, 0x2946);
write_cmos_sensor(0x6F12, 0x1DA8);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC7FC);
write_cmos_sensor(0x6F12, 0x318D);
write_cmos_sensor(0x6F12, 0x21A8);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC8FC);
write_cmos_sensor(0x6F12, 0x641C);
write_cmos_sensor(0x6F12, 0x032C);
write_cmos_sensor(0x6F12, 0x89D3);
write_cmos_sensor(0x6F12, 0x29B0);
write_cmos_sensor(0x6F12, 0x8CE6);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xF041);
write_cmos_sensor(0x6F12, 0x8046);
write_cmos_sensor(0x6F12, 0x2748);
write_cmos_sensor(0x6F12, 0x0C46);
write_cmos_sensor(0x6F12, 0x0D89);
write_cmos_sensor(0x6F12, 0x0169);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0E0C);
write_cmos_sensor(0x6F12, 0x8FB2);
write_cmos_sensor(0x6F12, 0x3946);
write_cmos_sensor(0x6F12, 0x3046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xBAFC);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0x4046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xBBFC);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x3946);
write_cmos_sensor(0x6F12, 0x3046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB1FC);
write_cmos_sensor(0x6F12, 0x6089);
write_cmos_sensor(0x6F12, 0x2189);
write_cmos_sensor(0x6F12, 0xC0F5);
write_cmos_sensor(0x6F12, 0x8072);
write_cmos_sensor(0x6F12, 0x6A43);
write_cmos_sensor(0x6F12, 0x01FB);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x8030);
write_cmos_sensor(0x6F12, 0xC0F3);
write_cmos_sensor(0x6F12, 0x0F20);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF081);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xF041);
write_cmos_sensor(0x6F12, 0x0546);
write_cmos_sensor(0x6F12, 0xC088);
write_cmos_sensor(0x6F12, 0x1600);
write_cmos_sensor(0x6F12, 0x8846);
write_cmos_sensor(0x6F12, 0x04D0);
write_cmos_sensor(0x6F12, 0x05F6);
write_cmos_sensor(0x6F12, 0x0C04);
write_cmos_sensor(0x6F12, 0x41F6);
write_cmos_sensor(0x6F12, 0x0C01);
write_cmos_sensor(0x6F12, 0x03E0);
write_cmos_sensor(0x6F12, 0x05F1);
write_cmos_sensor(0x6F12, 0x0C04);
write_cmos_sensor(0x6F12, 0x41F2);
write_cmos_sensor(0x6F12, 0x0C01);
write_cmos_sensor(0x6F12, 0x6A78);
write_cmos_sensor(0x6F12, 0x2944);
write_cmos_sensor(0x6F12, 0xEAB3);
write_cmos_sensor(0x6F12, 0xC0F5);
write_cmos_sensor(0x6F12, 0x8072);
write_cmos_sensor(0x6F12, 0x1203);
write_cmos_sensor(0x6F12, 0x0023);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x025B);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x025B);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x025B);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x0DE0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0xF000);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x9C68);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x9A00);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xE7A0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x2870);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x29C0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x60E0);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x025B);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x045B);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x045B);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x025C);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x025C);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x085B);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x085B);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x065C);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x065C);
write_cmos_sensor(0x6F12, 0x00E0);
write_cmos_sensor(0x6F12, 0x11E0);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x045C);
write_cmos_sensor(0x6F12, 0x5B1C);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x045C);
write_cmos_sensor(0x6F12, 0x34F8);
write_cmos_sensor(0x6F12, 0x025C);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x0025);
write_cmos_sensor(0x6F12, 0x2D0A);
write_cmos_sensor(0x6F12, 0x21F8);
write_cmos_sensor(0x6F12, 0x025C);
write_cmos_sensor(0x6F12, 0x802B);
write_cmos_sensor(0x6F12, 0xB3D3);
write_cmos_sensor(0x6F12, 0x99E7);
write_cmos_sensor(0x6F12, 0xF748);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x4069);
write_cmos_sensor(0x6F12, 0x87B2);
write_cmos_sensor(0x6F12, 0x040C);
write_cmos_sensor(0x6F12, 0x3946);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x38FC);
write_cmos_sensor(0x6F12, 0x3246);
write_cmos_sensor(0x6F12, 0x4146);
write_cmos_sensor(0x6F12, 0x2846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x3DFC);
write_cmos_sensor(0x6F12, 0x3946);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF041);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x2CBC);
write_cmos_sensor(0x6F12, 0x70B5);
write_cmos_sensor(0x6F12, 0x0646);
write_cmos_sensor(0x6F12, 0xEB48);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x8169);
write_cmos_sensor(0x6F12, 0x0C0C);
write_cmos_sensor(0x6F12, 0x8DB2);
write_cmos_sensor(0x6F12, 0x2946);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x21FC);
write_cmos_sensor(0x6F12, 0x3046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x2DFC);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x2946);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x19FC);
write_cmos_sensor(0x6F12, 0xE449);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0x8060);
write_cmos_sensor(0x6F12, 0x0880);
write_cmos_sensor(0x6F12, 0xE24A);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0xD661);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x911C);
write_cmos_sensor(0x6F12, 0x0880);
write_cmos_sensor(0x6F12, 0x121D);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0x9C51);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1080);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1080);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1080);
write_cmos_sensor(0x6F12, 0x901C);
write_cmos_sensor(0x6F12, 0x0180);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0xD74B);
write_cmos_sensor(0x6F12, 0x10B5);
write_cmos_sensor(0x6F12, 0xD3F8);
write_cmos_sensor(0x6F12, 0x7424);
write_cmos_sensor(0x6F12, 0x002A);
write_cmos_sensor(0x6F12, 0x0ED0);
write_cmos_sensor(0x6F12, 0xD3F8);
write_cmos_sensor(0x6F12, 0x0843);
write_cmos_sensor(0x6F12, 0x00FB);
write_cmos_sensor(0x6F12, 0x0410);
write_cmos_sensor(0x6F12, 0xD349);
write_cmos_sensor(0x6F12, 0x8C88);
write_cmos_sensor(0x6F12, 0x4443);
write_cmos_sensor(0x6F12, 0xC888);
write_cmos_sensor(0x6F12, 0xD3F8);
write_cmos_sensor(0x6F12, 0x1413);
write_cmos_sensor(0x6F12, 0xB4FB);
write_cmos_sensor(0x6F12, 0xF0F0);
write_cmos_sensor(0x6F12, 0x401A);
write_cmos_sensor(0x6F12, 0xB0FB);
write_cmos_sensor(0x6F12, 0xF2F0);
write_cmos_sensor(0x6F12, 0x10BD);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xFF4F);
write_cmos_sensor(0x6F12, 0x89B0);
write_cmos_sensor(0x6F12, 0x9146);
write_cmos_sensor(0x6F12, 0x16AA);
write_cmos_sensor(0x6F12, 0xC749);
write_cmos_sensor(0x6F12, 0x92E8);
write_cmos_sensor(0x6F12, 0x0111);
write_cmos_sensor(0x6F12, 0xDDE9);
write_cmos_sensor(0x6F12, 0x1A47);
write_cmos_sensor(0x6F12, 0xDDE9);
write_cmos_sensor(0x6F12, 0x1D6A);
write_cmos_sensor(0x6F12, 0x0978);
write_cmos_sensor(0x6F12, 0xDDF8);
write_cmos_sensor(0x6F12, 0x64B0);
write_cmos_sensor(0x6F12, 0x1D46);
write_cmos_sensor(0x6F12, 0x11B1);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0xFFF7);
write_cmos_sensor(0x6F12, 0xD6FF);
write_cmos_sensor(0x6F12, 0xC449);
write_cmos_sensor(0x6F12, 0x0AF0);
write_cmos_sensor(0x6F12, 0xFF03);
write_cmos_sensor(0x6F12, 0x1FFA);
write_cmos_sensor(0x6F12, 0x89F2);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0x9E51);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0x8C01);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0x9081);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xC2C1);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0xB0B1);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xB441);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xE671);
write_cmos_sensor(0x6F12, 0x1C98);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0xD401);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xD861);
write_cmos_sensor(0x6F12, 0x01F5);
write_cmos_sensor(0x6F12, 0xC271);
write_cmos_sensor(0x6F12, 0x0C46);
write_cmos_sensor(0x6F12, 0x6846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC9FB);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0x6846);
write_cmos_sensor(0x6F12, 0x1F9A);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC9FB);
write_cmos_sensor(0x6F12, 0xB348);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x6F12, 0x38B1);
write_cmos_sensor(0x6F12, 0xDDE9);
write_cmos_sensor(0x6F12, 0x0912);
write_cmos_sensor(0x6F12, 0x6846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC6FB);
write_cmos_sensor(0x6F12, 0x08B1);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC8FB);
write_cmos_sensor(0x6F12, 0x0021);
write_cmos_sensor(0x6F12, 0xAE48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC9FB);
write_cmos_sensor(0x6F12, 0xAD48);
write_cmos_sensor(0x6F12, 0x0121);
write_cmos_sensor(0x6F12, 0x2430);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC4FB);
write_cmos_sensor(0x6F12, 0xAA48);
write_cmos_sensor(0x6F12, 0x0221);
write_cmos_sensor(0x6F12, 0x4830);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xBFFB);
write_cmos_sensor(0x6F12, 0x0DB0);
write_cmos_sensor(0x6F12, 0x59E5);
write_cmos_sensor(0x6F12, 0xA548);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0xC110);
write_cmos_sensor(0x6F12, 0x19B1);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0x5700);
write_cmos_sensor(0x6F12, 0x1128);
write_cmos_sensor(0x6F12, 0x01D0);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x7047);
write_cmos_sensor(0x6F12, 0x0120);
write_cmos_sensor(0x6F12, 0x7047);
write_cmos_sensor(0x6F12, 0x10B5);
write_cmos_sensor(0x6F12, 0xE8B1);
write_cmos_sensor(0x6F12, 0xA149);
write_cmos_sensor(0x6F12, 0x2822);
write_cmos_sensor(0x6F12, 0xA1F5);
write_cmos_sensor(0x6F12, 0x5670);
write_cmos_sensor(0x6F12, 0x0446);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xADFB);
write_cmos_sensor(0x6F12, 0x964A);
write_cmos_sensor(0x6F12, 0x9949);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x1070);
write_cmos_sensor(0x6F12, 0x0888);
write_cmos_sensor(0x6F12, 0x50B1);
write_cmos_sensor(0x6F12, 0x4888);
write_cmos_sensor(0x6F12, 0x30B9);
write_cmos_sensor(0x6F12, 0xFFF7);
write_cmos_sensor(0x6F12, 0xE1FF);
write_cmos_sensor(0x6F12, 0x28B1);
write_cmos_sensor(0x6F12, 0x9348);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0x7E04);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x0120);
write_cmos_sensor(0x6F12, 0x1070);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x9FFB);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA1FB);
write_cmos_sensor(0x6F12, 0x06E0);
write_cmos_sensor(0x6F12, 0x9249);
write_cmos_sensor(0x6F12, 0x0E22);
write_cmos_sensor(0x6F12, 0x1831);
write_cmos_sensor(0x6F12, 0xA1F5);
write_cmos_sensor(0x6F12, 0x5670);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x8FFB);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x9CFB);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x9FFB);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA2FB);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0x1040);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA3BB);
write_cmos_sensor(0x6F12, 0x70B5);
write_cmos_sensor(0x6F12, 0x8A4D);
write_cmos_sensor(0x6F12, 0x8A4C);
write_cmos_sensor(0x6F12, 0x2888);
write_cmos_sensor(0x6F12, 0x2081);
write_cmos_sensor(0x6F12, 0xA081);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA0FB);
write_cmos_sensor(0x6F12, 0x2080);
write_cmos_sensor(0x6F12, 0x2888);
write_cmos_sensor(0x6F12, 0xE080);
write_cmos_sensor(0x6F12, 0x0320);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x9FFB);
write_cmos_sensor(0x6F12, 0x0820);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA1FB);
write_cmos_sensor(0x6F12, 0x844D);
write_cmos_sensor(0x6F12, 0xA080);
write_cmos_sensor(0x6F12, 0x287D);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x06D1);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x2875);
write_cmos_sensor(0x6F12, 0x8148);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x9CFB);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x9FFB);
write_cmos_sensor(0x6F12, 0x287D);
write_cmos_sensor(0x6F12, 0x58BB);
write_cmos_sensor(0x6F12, 0x7F4C);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x6709);
write_cmos_sensor(0x6F12, 0x68B9);
write_cmos_sensor(0x6F12, 0x7E48);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x6F12, 0x50B9);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x6C09);
write_cmos_sensor(0x6F12, 0x38B9);
write_cmos_sensor(0x6F12, 0x7148);
write_cmos_sensor(0x6F12, 0x007A);
write_cmos_sensor(0x6F12, 0x10B1);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x5C09);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x18B1);
write_cmos_sensor(0x6F12, 0x1021);
write_cmos_sensor(0x6F12, 0x7448);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x8CFB);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x6909);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x18B1);
write_cmos_sensor(0x6F12, 0x4021);
write_cmos_sensor(0x6F12, 0x7048);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x83FB);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x86FB);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x38B1);
write_cmos_sensor(0x6F12, 0x0C21);
write_cmos_sensor(0x6F12, 0x6B48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x7AFB);
write_cmos_sensor(0x6F12, 0x0121);
write_cmos_sensor(0x6F12, 0x6948);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x76FB);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x7EFB);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x04D0);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0x7040);
write_cmos_sensor(0x6F12, 0x6748);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x7BBB);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0x8908);
write_cmos_sensor(0x6F12, 0x8900);
write_cmos_sensor(0x6F12, 0x41EA);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x82B2);
write_cmos_sensor(0x6F12, 0x3E21);
write_cmos_sensor(0x6F12, 0x46F2);
write_cmos_sensor(0x6F12, 0x4420);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x75BB);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xF041);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0x7A71);
write_cmos_sensor(0x6F12, 0xB0FB);
write_cmos_sensor(0x6F12, 0xF1F2);
write_cmos_sensor(0x6F12, 0xB0FB);
write_cmos_sensor(0x6F12, 0xF1F5);
write_cmos_sensor(0x6F12, 0x01FB);
write_cmos_sensor(0x6F12, 0x1207);
write_cmos_sensor(0x6F12, 0x0024);
write_cmos_sensor(0x6F12, 0x4F4E);
write_cmos_sensor(0x6F12, 0x06E0);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0xE801);
write_cmos_sensor(0x6F12, 0x4843);
write_cmos_sensor(0x6F12, 0x400B);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x67FB);
write_cmos_sensor(0x6F12, 0x641C);
write_cmos_sensor(0x6F12, 0x706E);
write_cmos_sensor(0x6F12, 0xAC42);
write_cmos_sensor(0x6F12, 0x4FEA);
write_cmos_sensor(0x6F12, 0x9000);
write_cmos_sensor(0x6F12, 0xF3D3);
write_cmos_sensor(0x6F12, 0x7843);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF041);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x4010);
write_cmos_sensor(0x6F12, 0x400B);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x59BB);
write_cmos_sensor(0x6F12, 0x70B5);
write_cmos_sensor(0x6F12, 0x0024);
write_cmos_sensor(0x6F12, 0x504D);
write_cmos_sensor(0x6F12, 0x0CE0);
write_cmos_sensor(0x6F12, 0xA0F5);
write_cmos_sensor(0x6F12, 0x7F42);
write_cmos_sensor(0x6F12, 0xFE3A);
write_cmos_sensor(0x6F12, 0x13D0);
write_cmos_sensor(0x6F12, 0x521E);
write_cmos_sensor(0x6F12, 0x14D0);
write_cmos_sensor(0x6F12, 0x91F8);
write_cmos_sensor(0x6F12, 0x4410);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x50FB);
write_cmos_sensor(0x6F12, 0x641C);
write_cmos_sensor(0x6F12, 0x142C);
write_cmos_sensor(0x6F12, 0x05D2);
write_cmos_sensor(0x6F12, 0x05EB);
write_cmos_sensor(0x6F12, 0x8401);
write_cmos_sensor(0x6F12, 0xB1F8);
write_cmos_sensor(0x6F12, 0x4200);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0xECD1);
write_cmos_sensor(0x6F12, 0x3849);
write_cmos_sensor(0x6F12, 0x0420);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0x3A08);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0x91F8);
write_cmos_sensor(0x6F12, 0x4400);
write_cmos_sensor(0x6F12, 0x05E0);
write_cmos_sensor(0x6F12, 0x91F8);
write_cmos_sensor(0x6F12, 0x4400);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0x7A71);
write_cmos_sensor(0x6F12, 0x10FB);
write_cmos_sensor(0x6F12, 0x01F0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x3CFB);
write_cmos_sensor(0x6F12, 0xE5E7);
write_cmos_sensor(0x6F12, 0x70B5);
write_cmos_sensor(0x6F12, 0x0024);
write_cmos_sensor(0x6F12, 0x3C4D);
write_cmos_sensor(0x6F12, 0x0CE0);
write_cmos_sensor(0x6F12, 0xA0F5);
write_cmos_sensor(0x6F12, 0x7F42);
write_cmos_sensor(0x6F12, 0xFE3A);
write_cmos_sensor(0x6F12, 0x13D0);
write_cmos_sensor(0x6F12, 0x521E);
write_cmos_sensor(0x6F12, 0x14D0);
write_cmos_sensor(0x6F12, 0x91F8);
write_cmos_sensor(0x6F12, 0x9410);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x28FB);
write_cmos_sensor(0x6F12, 0x641C);
write_cmos_sensor(0x6F12, 0x142C);
write_cmos_sensor(0x6F12, 0x05D2);
write_cmos_sensor(0x6F12, 0x05EB);
write_cmos_sensor(0x6F12, 0x8401);
write_cmos_sensor(0x6F12, 0xB1F8);
write_cmos_sensor(0x6F12, 0x9200);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0xECD1);
write_cmos_sensor(0x6F12, 0x2449);
write_cmos_sensor(0x6F12, 0x0220);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0x3A08);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0x91F8);
write_cmos_sensor(0x6F12, 0x9400);
write_cmos_sensor(0x6F12, 0x05E0);
write_cmos_sensor(0x6F12, 0x91F8);
write_cmos_sensor(0x6F12, 0x9400);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0x7A71);
write_cmos_sensor(0x6F12, 0x10FB);
write_cmos_sensor(0x6F12, 0x01F0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x14FB);
write_cmos_sensor(0x6F12, 0xE5E7);
write_cmos_sensor(0x6F12, 0xF8B5);
write_cmos_sensor(0x6F12, 0x1B4C);
write_cmos_sensor(0x6F12, 0xB4F8);
write_cmos_sensor(0x6F12, 0x3A08);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x6FD1);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x4C08);
write_cmos_sensor(0x6F12, 0x8047);
write_cmos_sensor(0x6F12, 0x1848);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0x7A71);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0xEE01);
write_cmos_sensor(0x6F12, 0x10FB);
write_cmos_sensor(0x6F12, 0x01F0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x01FB);
write_cmos_sensor(0x6F12, 0x0220);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x03FB);
write_cmos_sensor(0x6F12, 0x0120);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x05FB);
write_cmos_sensor(0x6F12, 0x1E4B);
write_cmos_sensor(0x6F12, 0x0021);
write_cmos_sensor(0x6F12, 0x5983);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xED00);
write_cmos_sensor(0x6F12, 0x18B1);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x401E);
write_cmos_sensor(0x6F12, 0x8240);
write_cmos_sensor(0x6F12, 0x5A83);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x0F01);
write_cmos_sensor(0x6F12, 0x70B1);
write_cmos_sensor(0x6F12, 0x84F8);
write_cmos_sensor(0x6F12, 0xCC18);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF9FA);
write_cmos_sensor(0x6F12, 0x144D);
write_cmos_sensor(0x6F12, 0x0646);
write_cmos_sensor(0x6F12, 0x3046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF9FA);
write_cmos_sensor(0x6F12, 0xD5F8);
write_cmos_sensor(0x6F12, 0x9811);
write_cmos_sensor(0x6F12, 0x8142);
write_cmos_sensor(0x6F12, 0x24D2);
write_cmos_sensor(0x6F12, 0x0121);
write_cmos_sensor(0x6F12, 0x23E0);
write_cmos_sensor(0x6F12, 0x2AE0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x60E0);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x9D62);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x68C0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0xF000);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x45B0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x12F0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x46B4);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x6D34);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xED50);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x61F0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x2390);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x3A70);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xBA00);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xED60);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x9420);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x4CD0);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x7000);
write_cmos_sensor(0x6F12, 0x0021);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xCC08);
write_cmos_sensor(0x6F12, 0x8DF8);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x9DF8);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0843);
write_cmos_sensor(0x6F12, 0xCAD0);
write_cmos_sensor(0x6F12, 0xCE48);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0xB900);
write_cmos_sensor(0x6F12, 0x8007);
write_cmos_sensor(0x6F12, 0x0CD0);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x1402);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0xF855);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0x2846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x97FA);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0x2846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x92FA);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF840);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB6BA);
write_cmos_sensor(0x6F12, 0xF8BD);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xFE4F);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB6FA);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB9FA);
write_cmos_sensor(0x6F12, 0x0128);
write_cmos_sensor(0x6F12, 0x7DD1);
write_cmos_sensor(0x6F12, 0xBF4C);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xA700);
write_cmos_sensor(0x6F12, 0x0090);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xA800);
write_cmos_sensor(0x6F12, 0xDF34);
write_cmos_sensor(0x6F12, 0x0190);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB2FA);
write_cmos_sensor(0x6F12, 0xBB4F);
write_cmos_sensor(0x6F12, 0x0646);
write_cmos_sensor(0x6F12, 0xBB4D);
write_cmos_sensor(0x6F12, 0x97F8);
write_cmos_sensor(0x6F12, 0xA800);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x40F4);
write_cmos_sensor(0x6F12, 0x0070);
write_cmos_sensor(0x6F12, 0xA5F8);
write_cmos_sensor(0x6F12, 0xB201);
write_cmos_sensor(0x6F12, 0xDFF8);
write_cmos_sensor(0x6F12, 0xE0A2);
write_cmos_sensor(0x6F12, 0x6B46);
write_cmos_sensor(0x6F12, 0x3246);
write_cmos_sensor(0x6F12, 0xBAF8);
write_cmos_sensor(0x6F12, 0xEC11);
write_cmos_sensor(0x6F12, 0x4FF2);
write_cmos_sensor(0x6F12, 0xB010);
write_cmos_sensor(0x6F12, 0x05F5);
write_cmos_sensor(0x6F12, 0xC275);
write_cmos_sensor(0x6F12, 0x9E37);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA0FA);
write_cmos_sensor(0x6F12, 0x0E20);
write_cmos_sensor(0x6F12, 0xA077);
write_cmos_sensor(0x6F12, 0xA885);
write_cmos_sensor(0x6F12, 0xB97A);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x6F12, 0x48EA);
write_cmos_sensor(0x6F12, 0x4100);
write_cmos_sensor(0x6F12, 0xE887);
write_cmos_sensor(0x6F12, 0xB988);
write_cmos_sensor(0x6F12, 0x6B46);
write_cmos_sensor(0x6F12, 0x3246);
write_cmos_sensor(0x6F12, 0x4FF2);
write_cmos_sensor(0x6F12, 0xC010);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x90FA);
write_cmos_sensor(0x6F12, 0x0026);
write_cmos_sensor(0x6F12, 0xA677);
write_cmos_sensor(0x6F12, 0xAE87);
write_cmos_sensor(0x6F12, 0x25F8);
write_cmos_sensor(0x6F12, 0x128C);
write_cmos_sensor(0x6F12, 0x4FF0);
write_cmos_sensor(0x6F12, 0x0109);
write_cmos_sensor(0x6F12, 0x84F8);
write_cmos_sensor(0x6F12, 0x1D90);
write_cmos_sensor(0x6F12, 0x84F8);
write_cmos_sensor(0x6F12, 0x1F90);
write_cmos_sensor(0x6F12, 0xBAF8);
write_cmos_sensor(0x6F12, 0xEA21);
write_cmos_sensor(0x6F12, 0x0198);
write_cmos_sensor(0x6F12, 0x3988);
write_cmos_sensor(0x6F12, 0x4243);
write_cmos_sensor(0x6F12, 0x6B46);
write_cmos_sensor(0x6F12, 0x4FF2);
write_cmos_sensor(0x6F12, 0x7010);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x7BFA);
write_cmos_sensor(0x6F12, 0x25F8);
write_cmos_sensor(0x6F12, 0x149C);
write_cmos_sensor(0x6F12, 0x9F4C);
write_cmos_sensor(0x6F12, 0xA048);
write_cmos_sensor(0x6F12, 0x84F8);
write_cmos_sensor(0x6F12, 0xCE6C);
write_cmos_sensor(0x6F12, 0x0680);
write_cmos_sensor(0x6F12, 0x9F48);
write_cmos_sensor(0x6F12, 0x0680);
write_cmos_sensor(0x6F12, 0xA5F8);
write_cmos_sensor(0x6F12, 0x0E80);
write_cmos_sensor(0x6F12, 0x9E4E);
write_cmos_sensor(0x6F12, 0x6B46);
write_cmos_sensor(0x6F12, 0x2821);
write_cmos_sensor(0x6F12, 0x7288);
write_cmos_sensor(0x6F12, 0x4FF2);
write_cmos_sensor(0x6F12, 0x9010);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x68FA);
write_cmos_sensor(0x6F12, 0x0220);
write_cmos_sensor(0x6F12, 0xA881);
write_cmos_sensor(0x6F12, 0xA5F8);
write_cmos_sensor(0x6F12, 0x1E80);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xCE0C);
write_cmos_sensor(0x6F12, 0xF021);
write_cmos_sensor(0x6F12, 0x0201);
write_cmos_sensor(0x6F12, 0x4BF2);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x20FA);
write_cmos_sensor(0x6F12, 0xA4F5);
write_cmos_sensor(0x6F12, 0x8045);
write_cmos_sensor(0x6F12, 0x2878);
write_cmos_sensor(0x6F12, 0xE8B1);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xCE0C);
write_cmos_sensor(0x6F12, 0xF021);
write_cmos_sensor(0x6F12, 0x0201);
write_cmos_sensor(0x6F12, 0x4BF2);
write_cmos_sensor(0x6F12, 0x0800);
write_cmos_sensor(0x6F12, 0x04F6);
write_cmos_sensor(0x6F12, 0xCE44);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x12FA);
write_cmos_sensor(0x6F12, 0x2078);
write_cmos_sensor(0x6F12, 0xF021);
write_cmos_sensor(0x6F12, 0x0201);
write_cmos_sensor(0x6F12, 0x4BF2);
write_cmos_sensor(0x6F12, 0x0A00);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x0BFA);
write_cmos_sensor(0x6F12, 0xA079);
write_cmos_sensor(0x6F12, 0xE179);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x00E0);
write_cmos_sensor(0x6F12, 0x0EE0);
write_cmos_sensor(0x6F12, 0x40EA);
write_cmos_sensor(0x6F12, 0x0130);
write_cmos_sensor(0x6F12, 0xB5F8);
write_cmos_sensor(0x6F12, 0xDC12);
write_cmos_sensor(0x6F12, 0x0843);
write_cmos_sensor(0x6F12, 0x8549);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0x0601);
write_cmos_sensor(0x6F12, 0x8548);
write_cmos_sensor(0x6F12, 0x7288);
write_cmos_sensor(0x6F12, 0xB0F8);
write_cmos_sensor(0x6F12, 0xF412);
write_cmos_sensor(0x6F12, 0x1144);
write_cmos_sensor(0x6F12, 0xA0F8);
write_cmos_sensor(0x6F12, 0xF412);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xFE8F);
write_cmos_sensor(0x6F12, 0x10B5);
write_cmos_sensor(0x6F12, 0x804C);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xFC10);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0xFA00);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x2EFA);
write_cmos_sensor(0x6F12, 0x81B2);
write_cmos_sensor(0x6F12, 0xA4F8);
write_cmos_sensor(0x6F12, 0x2A12);
write_cmos_sensor(0x6F12, 0x8807);
write_cmos_sensor(0x6F12, 0x02D0);
write_cmos_sensor(0x6F12, 0x0E20);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x2BFA);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0xA4F8);
write_cmos_sensor(0x6F12, 0x2C02);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x1612);
write_cmos_sensor(0x6F12, 0xA4F8);
write_cmos_sensor(0x6F12, 0x2612);
write_cmos_sensor(0x6F12, 0xC807);
write_cmos_sensor(0x6F12, 0x02D0);
write_cmos_sensor(0x6F12, 0x4520);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1FFA);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x22FA);
write_cmos_sensor(0x6F12, 0xA4F8);
write_cmos_sensor(0x6F12, 0x2802);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x05FA);
write_cmos_sensor(0x6F12, 0x0128);
write_cmos_sensor(0x6F12, 0x16D1);
write_cmos_sensor(0x6F12, 0x6F48);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x6F12, 0x98B1);
write_cmos_sensor(0x6F12, 0x6B48);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x1622);
write_cmos_sensor(0x6F12, 0x4188);
write_cmos_sensor(0x6F12, 0x6248);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0xC000);
write_cmos_sensor(0x6F12, 0xC140);
write_cmos_sensor(0x6F12, 0x1144);
write_cmos_sensor(0x6F12, 0xA4F8);
write_cmos_sensor(0x6F12, 0x2612);
write_cmos_sensor(0x6F12, 0xB4F8);
write_cmos_sensor(0x6F12, 0x2812);
write_cmos_sensor(0x6F12, 0x4FF6);
write_cmos_sensor(0x6F12, 0xFF72);
write_cmos_sensor(0x6F12, 0x02EA);
write_cmos_sensor(0x6F12, 0x4101);
write_cmos_sensor(0x6F12, 0xC140);
write_cmos_sensor(0x6F12, 0xA4F8);
write_cmos_sensor(0x6F12, 0x2812);
write_cmos_sensor(0x6F12, 0xB4F8);
write_cmos_sensor(0x6F12, 0x2C32);
write_cmos_sensor(0x6F12, 0xB4F8);
write_cmos_sensor(0x6F12, 0x2A22);
write_cmos_sensor(0x6F12, 0xB4F8);
write_cmos_sensor(0x6F12, 0x2812);
write_cmos_sensor(0x6F12, 0xB4F8);
write_cmos_sensor(0x6F12, 0x2602);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0x1040);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xFEB9);
write_cmos_sensor(0x6F12, 0x70B5);
write_cmos_sensor(0x6F12, 0x5A48);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xC069);
write_cmos_sensor(0x6F12, 0x86B2);
write_cmos_sensor(0x6F12, 0x040C);
write_cmos_sensor(0x6F12, 0x3146);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x27F9);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF7F9);
write_cmos_sensor(0x6F12, 0x0546);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x3146);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1FF9);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xCCF9);
write_cmos_sensor(0x6F12, 0x0128);
write_cmos_sensor(0x6F12, 0x01D0);
write_cmos_sensor(0x6F12, 0x2846);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0x5148);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0xD512);
write_cmos_sensor(0x6F12, 0x5048);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x6F12, 0x00B1);
write_cmos_sensor(0x6F12, 0x1821);
write_cmos_sensor(0x6F12, 0x4B4C);
write_cmos_sensor(0x6F12, 0x6180);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xE5F9);
write_cmos_sensor(0x6F12, 0x6188);
write_cmos_sensor(0x6F12, 0xC140);
write_cmos_sensor(0x6F12, 0x4FF6);
write_cmos_sensor(0x6F12, 0xF870);
write_cmos_sensor(0x6F12, 0xC91D);
write_cmos_sensor(0x6F12, 0x0140);
write_cmos_sensor(0x6F12, 0x3F48);
write_cmos_sensor(0x6F12, 0x6180);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0xC000);
write_cmos_sensor(0x6F12, 0xC140);
write_cmos_sensor(0x6F12, 0x4819);
write_cmos_sensor(0x6F12, 0x3F49);
write_cmos_sensor(0x6F12, 0xB1F8);
write_cmos_sensor(0x6F12, 0xEA11);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0x10B5);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF6);
write_cmos_sensor(0x6F12, 0x6331);
write_cmos_sensor(0x6F12, 0x4348);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xD2F9);
write_cmos_sensor(0x6F12, 0x3D4C);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF6);
write_cmos_sensor(0x6F12, 0x7F21);
write_cmos_sensor(0x6F12, 0x6060);
write_cmos_sensor(0x6F12, 0x4048);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xCAF9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF6);
write_cmos_sensor(0x6F12, 0xEB01);
write_cmos_sensor(0x6F12, 0xA060);
write_cmos_sensor(0x6F12, 0x3D48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC3F9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x8F61);
write_cmos_sensor(0x6F12, 0xE060);
write_cmos_sensor(0x6F12, 0x3B48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xBCF9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x1161);
write_cmos_sensor(0x6F12, 0xA061);
write_cmos_sensor(0x6F12, 0x3848);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB5F9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x6551);
write_cmos_sensor(0x6F12, 0x3648);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xAFF9);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x0870);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x1351);
write_cmos_sensor(0x6F12, 0x3348);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA6F9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x7341);
write_cmos_sensor(0x6F12, 0x3148);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA0F9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x6B41);
write_cmos_sensor(0x6F12, 0x2F48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x9AF9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x3541);
write_cmos_sensor(0x6F12, 0x2D48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x94F9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0xF131);
write_cmos_sensor(0x6F12, 0x2B48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x8EF9);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0xBF20);
write_cmos_sensor(0x6F12, 0x2949);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x8864);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x9711);
write_cmos_sensor(0x6F12, 0x2848);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x84F9);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x1511);
write_cmos_sensor(0x6F12, 0x2648);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x7EF9);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0xAFF6);
write_cmos_sensor(0x6F12, 0x4F01);
write_cmos_sensor(0x6F12, 0xE061);
write_cmos_sensor(0x6F12, 0x2348);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x77F9);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0xAFF6);
write_cmos_sensor(0x6F12, 0x1901);
write_cmos_sensor(0x6F12, 0x2061);
write_cmos_sensor(0x6F12, 0x2148);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x70F9);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0xE931);
write_cmos_sensor(0x6F12, 0x6061);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0x1040);
write_cmos_sensor(0x6F12, 0x1D48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x67B9);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x1120);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x45B0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x12F0);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0xF000);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0xF000);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xDA00);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0xF4F2);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0xB40C);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x60E0);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0xE000);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x68C0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x9A00);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x7547);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x7361);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x276F);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x574D);
write_cmos_sensor(0x6F12, 0x0002);
write_cmos_sensor(0x6F12, 0x432D);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0xB127);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x7E15);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x6D83);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x7DCF);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x33DB);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x3423);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x5D90);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0ACB);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0xA89B);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0xCB99);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0xCCC9);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x984B);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0xD91C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xCD3C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xDD3C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xD53C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F2);
write_cmos_sensor(0x6F12, 0x3D3C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0xE91C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x46F2);
write_cmos_sensor(0x6F12, 0xAF4C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x46F2);
write_cmos_sensor(0x6F12, 0xEB4C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x46F2);
write_cmos_sensor(0x6F12, 0x915C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F6);
write_cmos_sensor(0x6F12, 0x311C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F6);
write_cmos_sensor(0x6F12, 0x4F2C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F6);
write_cmos_sensor(0x6F12, 0x952C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0xFB7C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F6);
write_cmos_sensor(0x6F12, 0x130C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x3D5C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x555C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x391C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4CF6);
write_cmos_sensor(0x6F12, 0x993C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4CF6);
write_cmos_sensor(0x6F12, 0xC94C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x45F2);
write_cmos_sensor(0x6F12, 0x4D7C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x43F6);
write_cmos_sensor(0x6F12, 0xB76C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x43F6);
write_cmos_sensor(0x6F12, 0xE96C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xA90C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xB92C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xF72C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x46F2);
write_cmos_sensor(0x6F12, 0xED1C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x45F6);
write_cmos_sensor(0x6F12, 0xE51C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4BF2);
write_cmos_sensor(0x6F12, 0x770C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0xC76C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4AF2);
write_cmos_sensor(0x6F12, 0xC17C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4AF2);
write_cmos_sensor(0x6F12, 0x797C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x691C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x41F2);
write_cmos_sensor(0x6F12, 0x331C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xE53C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x554C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0x6D7C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0x975C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0x476C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0xBF6C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0xC55C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x43F6);
write_cmos_sensor(0x6F12, 0xBB1C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x551C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0xB95C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F6);
write_cmos_sensor(0x6F12, 0xD70C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0xCF5C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0x715C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x49F2);
write_cmos_sensor(0x6F12, 0x1F5C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0x735C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0xA35C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4EF6);
write_cmos_sensor(0x6F12, 0xF34C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x45F2);
write_cmos_sensor(0x6F12, 0xC70C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0x692C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0x8F6C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F6);
write_cmos_sensor(0x6F12, 0x451C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0x3F6C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x42F2);
write_cmos_sensor(0x6F12, 0x113C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xC36C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x43F2);
write_cmos_sensor(0x6F12, 0x374C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4AF6);
write_cmos_sensor(0x6F12, 0x9B0C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F6);
write_cmos_sensor(0x6F12, 0x290C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4BF6);
write_cmos_sensor(0x6F12, 0x152C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0841);
write_cmos_sensor(0x6F12, 0x02E5);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0FFF);
write_cmos_sensor(0x602A, 0xF000);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0004);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0xF00A);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0xF14A);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0xF19A);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0xF122);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0xF172);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0xF1C2);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x6F12, 0x0008);
write_cmos_sensor(0x6F12, 0x13C0);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6028, 0x2000);
write_cmos_sensor(0x602A, 0x2500);
write_cmos_sensor(0x6F12, 0x0080);
write_cmos_sensor(0x602A, 0x10B8);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x602A, 0x1EE0);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x602A, 0x2870);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x250A);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x23A0);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x3022);
write_cmos_sensor(0x6F12, 0x1281);
write_cmos_sensor(0x602A, 0x32E8);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x54A2);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x120E);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x1212);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x2860);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x3220);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x1226);
write_cmos_sensor(0x6F12, 0x0301);
write_cmos_sensor(0x602A, 0x29C8);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x32EC);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x12BE);
write_cmos_sensor(0x6F12, 0x0101);
write_cmos_sensor(0x602A, 0x3034);
write_cmos_sensor(0x6F12, 0x049B);
write_cmos_sensor(0x602A, 0x1230);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x1232);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x602A, 0x1236);
write_cmos_sensor(0x6F12, 0x01FF);
write_cmos_sensor(0x602A, 0x123A);
write_cmos_sensor(0x6F12, 0x0004);
write_cmos_sensor(0x602A, 0x123E);
write_cmos_sensor(0x6F12, 0xF45A);
write_cmos_sensor(0x602A, 0x1EE2);
write_cmos_sensor(0x6F12, 0x19CD);
write_cmos_sensor(0x602A, 0x115E);
write_cmos_sensor(0x6F12, 0x0048);
write_cmos_sensor(0x602A, 0x131C);
write_cmos_sensor(0x6F12, 0x2400);
write_cmos_sensor(0x602A, 0x2872);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x1314);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x20DE);
write_cmos_sensor(0x6F12, 0x0003);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x602A, 0x2108);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x602A, 0x1EDC);
write_cmos_sensor(0x6F12, 0x5008);
write_cmos_sensor(0x602A, 0x138E);
write_cmos_sensor(0x6F12, 0x13C0);
write_cmos_sensor(0x602A, 0x1392);
write_cmos_sensor(0x6F12, 0x0038);
write_cmos_sensor(0x602A, 0x21B6);
write_cmos_sensor(0x6F12, 0x0002);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x2550);
write_cmos_sensor(0x6F12, 0x193C);
write_cmos_sensor(0x6028, 0x4000);
write_cmos_sensor(0x0BC0, 0x0040);
write_cmos_sensor(0x0FE8, 0x49C1);
write_cmos_sensor(0x0FEA, 0x0040);
write_cmos_sensor(0x0BC8, 0x0001);
write_cmos_sensor(0x0B0A, 0x0101);
write_cmos_sensor(0x0BC6, 0x0000);
write_cmos_sensor(0x0B06, 0x0101);
write_cmos_sensor(0xF446, 0x000C);
write_cmos_sensor(0xF448, 0x0018);
write_cmos_sensor(0xF450, 0x0010);
write_cmos_sensor(0xF44E, 0x0000);
write_cmos_sensor(0xF468, 0xE000);
#endif

write_cmos_sensor(0x6028, 0x4000);
write_cmos_sensor(0x0000, 0x0007);
write_cmos_sensor(0x0000, 0x0841);
write_cmos_sensor(0x6010, 0x0001);
mDELAY(25);


write_cmos_sensor(0x6214, 0xF9F0);
write_cmos_sensor(0x6218, 0xE150);
write_cmos_sensor(0x6242, 0x0E00);
write_cmos_sensor(0x6028, 0x2001);
write_cmos_sensor(0x602A, 0x518C);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0549);
write_cmos_sensor(0x6F12, 0x0448);
write_cmos_sensor(0x6F12, 0x054A);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0xD006);
write_cmos_sensor(0x6F12, 0x101A);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xD406);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x59BA);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x5880);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x68C0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0xF800);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xFF4F);
write_cmos_sensor(0x6F12, 0x0446);
write_cmos_sensor(0x6F12, 0x0878);
write_cmos_sensor(0x6F12, 0x85B0);
write_cmos_sensor(0x6F12, 0x9246);
write_cmos_sensor(0x6F12, 0x0F46);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x6FD0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xABFA);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x6BD0);
write_cmos_sensor(0x6F12, 0x0026);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA6FA);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x69D0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA7FA);
write_cmos_sensor(0x6F12, 0x050A);
write_cmos_sensor(0x6F12, 0xB6FA);
write_cmos_sensor(0x6F12, 0x86F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2002);
write_cmos_sensor(0x6F12, 0x092A);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0922);
write_cmos_sensor(0x6F12, 0x093A);
write_cmos_sensor(0x6F12, 0xD640);
write_cmos_sensor(0x6F12, 0xB5FA);
write_cmos_sensor(0x6F12, 0x85F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xF6B2);
write_cmos_sensor(0x6F12, 0x0928);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0920);
write_cmos_sensor(0x6F12, 0x0938);
write_cmos_sensor(0x6F12, 0xC540);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x8008);
write_cmos_sensor(0x6F12, 0x0898);
write_cmos_sensor(0x6F12, 0x05F0);
write_cmos_sensor(0x6F12, 0xFF09);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0xC003);
write_cmos_sensor(0x6F12, 0x03EB);
write_cmos_sensor(0x6F12, 0x0013);
write_cmos_sensor(0x6F12, 0x0021);
write_cmos_sensor(0x6F12, 0xEB46);
write_cmos_sensor(0x6F12, 0x07EB);
write_cmos_sensor(0x6F12, 0xC30C);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0xC100);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x0110);
write_cmos_sensor(0x6F12, 0x0CEB);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x4800);
write_cmos_sensor(0x6F12, 0x851C);
write_cmos_sensor(0x6F12, 0x531C);
write_cmos_sensor(0x6F12, 0x0C30);
write_cmos_sensor(0x6F12, 0x35F9);
write_cmos_sensor(0x6F12, 0x1370);
write_cmos_sensor(0x6F12, 0x30F9);
write_cmos_sensor(0x6F12, 0x1330);
write_cmos_sensor(0x6F12, 0x35F9);
write_cmos_sensor(0x6F12, 0x1250);
write_cmos_sensor(0x6F12, 0x30F9);
write_cmos_sensor(0x6F12, 0x1200);
write_cmos_sensor(0x6F12, 0x7F1B);
write_cmos_sensor(0x6F12, 0x1B1A);
write_cmos_sensor(0x6F12, 0x7743);
write_cmos_sensor(0x6F12, 0x7343);
write_cmos_sensor(0x6F12, 0x8037);
write_cmos_sensor(0x6F12, 0x8033);
write_cmos_sensor(0x6F12, 0x05EB);
write_cmos_sensor(0x6F12, 0x2725);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0x2320);
write_cmos_sensor(0x6F12, 0x401B);
write_cmos_sensor(0x6F12, 0x00FB);
write_cmos_sensor(0x6F12, 0x09F0);
write_cmos_sensor(0x6F12, 0x8030);
write_cmos_sensor(0x6F12, 0x05EB);
write_cmos_sensor(0x6F12, 0x2020);
write_cmos_sensor(0x6F12, 0x4BF8);
write_cmos_sensor(0x6F12, 0x2100);
write_cmos_sensor(0x6F12, 0x00FA);
write_cmos_sensor(0x6F12, 0x0AF0);
write_cmos_sensor(0x6F12, 0x4BF8);
write_cmos_sensor(0x6F12, 0x2100);
write_cmos_sensor(0x6F12, 0x491C);
write_cmos_sensor(0x6F12, 0x0429);
write_cmos_sensor(0x6F12, 0xD3D3);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x0099);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0401);
write_cmos_sensor(0x6F12, 0x0199);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0401);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0801);
write_cmos_sensor(0x6F12, 0x0299);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0801);
write_cmos_sensor(0x6F12, 0xD4F8);
write_cmos_sensor(0x6F12, 0x0C01);
write_cmos_sensor(0x6F12, 0x0399);
write_cmos_sensor(0x6F12, 0x0844);
write_cmos_sensor(0x6F12, 0xC4F8);
write_cmos_sensor(0x6F12, 0x0C01);
write_cmos_sensor(0x6F12, 0x09B0);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF08F);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x42FA);
write_cmos_sensor(0x6F12, 0x050A);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x44FA);
write_cmos_sensor(0x6F12, 0x060A);
write_cmos_sensor(0x6F12, 0x91E7);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xF843);
write_cmos_sensor(0x6F12, 0x1C46);
write_cmos_sensor(0x6F12, 0x0093);
write_cmos_sensor(0x6F12, 0x0E46);
write_cmos_sensor(0x6F12, 0x1546);
write_cmos_sensor(0x6F12, 0x1346);
write_cmos_sensor(0x6F12, 0x0746);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xFD49);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x3BFA);
write_cmos_sensor(0x6F12, 0xFC49);
write_cmos_sensor(0x6F12, 0x2B46);
write_cmos_sensor(0x6F12, 0x0422);
write_cmos_sensor(0x6F12, 0x1831);
write_cmos_sensor(0x6F12, 0x3846);
write_cmos_sensor(0x6F12, 0x0094);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x33FA);
write_cmos_sensor(0x6F12, 0xF849);
write_cmos_sensor(0x6F12, 0x2B46);
write_cmos_sensor(0x6F12, 0x0822);
write_cmos_sensor(0x6F12, 0x3031);
write_cmos_sensor(0x6F12, 0x3846);
write_cmos_sensor(0x6F12, 0x0094);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x2BFA);
write_cmos_sensor(0x6F12, 0x96F8);
write_cmos_sensor(0x6F12, 0x5A02);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x69D0);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x11FA);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x65D0);
write_cmos_sensor(0x6F12, 0xF14D);
write_cmos_sensor(0x6F12, 0x4FF0);
write_cmos_sensor(0x6F12, 0x0008);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x0AFA);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x60D0);
write_cmos_sensor(0x6F12, 0x0923);
write_cmos_sensor(0x6F12, 0x1022);
write_cmos_sensor(0x6F12, 0x2968);
write_cmos_sensor(0x6F12, 0x686A);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1BFA);
write_cmos_sensor(0x6F12, 0x070A);
write_cmos_sensor(0x6F12, 0xB7FA);
write_cmos_sensor(0x6F12, 0x87F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x0929);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0921);
write_cmos_sensor(0x6F12, 0xB8FA);
write_cmos_sensor(0x6F12, 0x88F0);
write_cmos_sensor(0x6F12, 0xC0F1);
write_cmos_sensor(0x6F12, 0x2005);
write_cmos_sensor(0x6F12, 0x0939);
write_cmos_sensor(0x6F12, 0x092D);
write_cmos_sensor(0x6F12, 0x00D2);
write_cmos_sensor(0x6F12, 0x0925);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x8107);
write_cmos_sensor(0x6F12, 0x093D);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0xC001);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x06EB);
write_cmos_sensor(0x6F12, 0x4101);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4701);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4501);
write_cmos_sensor(0x6F12, 0x54F8);
write_cmos_sensor(0x6F12, 0x2020);
write_cmos_sensor(0x6F12, 0xB1F8);
write_cmos_sensor(0x6F12, 0x5C12);
write_cmos_sensor(0x6F12, 0x4A43);
write_cmos_sensor(0x6F12, 0x110B);
write_cmos_sensor(0x6F12, 0x44F8);
write_cmos_sensor(0x6F12, 0x2010);
write_cmos_sensor(0x6F12, 0x401C);
write_cmos_sensor(0x6F12, 0x0428);
write_cmos_sensor(0x6F12, 0xEAD3);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0xC001);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x06EB);
write_cmos_sensor(0x6F12, 0x4101);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4701);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4501);
write_cmos_sensor(0x6F12, 0xB1F8);
write_cmos_sensor(0x6F12, 0xEC23);
write_cmos_sensor(0x6F12, 0x04EB);
write_cmos_sensor(0x6F12, 0x8001);
write_cmos_sensor(0x6F12, 0x401C);
write_cmos_sensor(0x6F12, 0x0B69);
write_cmos_sensor(0x6F12, 0x5343);
write_cmos_sensor(0x6F12, 0x1A0B);
write_cmos_sensor(0x6F12, 0x0A61);
write_cmos_sensor(0x6F12, 0x0428);
write_cmos_sensor(0x6F12, 0xEAD3);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC1F9);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x15D1);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x00EB);
write_cmos_sensor(0x6F12, 0xC001);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x06EB);
write_cmos_sensor(0x6F12, 0x4101);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4701);
write_cmos_sensor(0x6F12, 0x01EB);
write_cmos_sensor(0x6F12, 0x4501);
write_cmos_sensor(0x6F12, 0xB1F8);
write_cmos_sensor(0x6F12, 0x2423);
write_cmos_sensor(0x6F12, 0x04EB);
write_cmos_sensor(0x6F12, 0x8001);
write_cmos_sensor(0x6F12, 0x401C);
write_cmos_sensor(0x6F12, 0x0B6A);
write_cmos_sensor(0x6F12, 0x5343);
write_cmos_sensor(0x6F12, 0x1A0B);
write_cmos_sensor(0x6F12, 0x0A62);
write_cmos_sensor(0x6F12, 0x0428);
write_cmos_sensor(0x6F12, 0xEAD3);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0xF883);
write_cmos_sensor(0x6F12, 0x0923);
write_cmos_sensor(0x6F12, 0x1022);
write_cmos_sensor(0x6F12, 0xA96C);
write_cmos_sensor(0x6F12, 0x686A);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xBAF9);
write_cmos_sensor(0x6F12, 0x070A);
write_cmos_sensor(0x6F12, 0x0923);
write_cmos_sensor(0x6F12, 0x1022);
write_cmos_sensor(0x6F12, 0x2968);
write_cmos_sensor(0x6F12, 0xA86C);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xB3F9);
write_cmos_sensor(0x6F12, 0x4FEA);
write_cmos_sensor(0x6F12, 0x1028);
write_cmos_sensor(0x6F12, 0x95E7);
write_cmos_sensor(0x6F12, 0x70B5);
write_cmos_sensor(0x6F12, 0x0646);
write_cmos_sensor(0x6F12, 0xB548);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xC168);
write_cmos_sensor(0x6F12, 0x0C0C);
write_cmos_sensor(0x6F12, 0x8DB2);
write_cmos_sensor(0x6F12, 0x2946);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xAAF9);
write_cmos_sensor(0x6F12, 0x3046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xACF9);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x2946);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xA2F9);
write_cmos_sensor(0x6F12, 0xAD49);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0x8060);
write_cmos_sensor(0x6F12, 0x0880);
write_cmos_sensor(0x6F12, 0xAB4A);
write_cmos_sensor(0x6F12, 0x4FF4);
write_cmos_sensor(0x6F12, 0xD661);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x911C);
write_cmos_sensor(0x6F12, 0x0880);
write_cmos_sensor(0x6F12, 0x121D);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0x9C51);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1080);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1080);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1180);
write_cmos_sensor(0x6F12, 0x921C);
write_cmos_sensor(0x6F12, 0x1080);
write_cmos_sensor(0x6F12, 0x901C);
write_cmos_sensor(0x6F12, 0x0180);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0xA04B);
write_cmos_sensor(0x6F12, 0x10B5);
write_cmos_sensor(0x6F12, 0xD3F8);
write_cmos_sensor(0x6F12, 0x7424);
write_cmos_sensor(0x6F12, 0x002A);
write_cmos_sensor(0x6F12, 0x0ED0);
write_cmos_sensor(0x6F12, 0xD3F8);
write_cmos_sensor(0x6F12, 0x0843);
write_cmos_sensor(0x6F12, 0x00FB);
write_cmos_sensor(0x6F12, 0x0410);
write_cmos_sensor(0x6F12, 0x9C49);
write_cmos_sensor(0x6F12, 0x8C88);
write_cmos_sensor(0x6F12, 0x4443);
write_cmos_sensor(0x6F12, 0xC888);
write_cmos_sensor(0x6F12, 0xD3F8);
write_cmos_sensor(0x6F12, 0x1413);
write_cmos_sensor(0x6F12, 0xB4FB);
write_cmos_sensor(0x6F12, 0xF0F0);
write_cmos_sensor(0x6F12, 0x401A);
write_cmos_sensor(0x6F12, 0xB0FB);
write_cmos_sensor(0x6F12, 0xF2F0);
write_cmos_sensor(0x6F12, 0x10BD);
write_cmos_sensor(0x6F12, 0x2DE9);
write_cmos_sensor(0x6F12, 0xFF4F);
write_cmos_sensor(0x6F12, 0x89B0);
write_cmos_sensor(0x6F12, 0x9146);
write_cmos_sensor(0x6F12, 0x16AA);
write_cmos_sensor(0x6F12, 0x9149);
write_cmos_sensor(0x6F12, 0x92E8);
write_cmos_sensor(0x6F12, 0x0111);
write_cmos_sensor(0x6F12, 0xDDE9);
write_cmos_sensor(0x6F12, 0x1A47);
write_cmos_sensor(0x6F12, 0xDDE9);
write_cmos_sensor(0x6F12, 0x1D6A);
write_cmos_sensor(0x6F12, 0x0978);
write_cmos_sensor(0x6F12, 0xDDF8);
write_cmos_sensor(0x6F12, 0x64B0);
write_cmos_sensor(0x6F12, 0x1D46);
write_cmos_sensor(0x6F12, 0x11B1);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0xFFF7);
write_cmos_sensor(0x6F12, 0xD6FF);
write_cmos_sensor(0x6F12, 0x8D49);
write_cmos_sensor(0x6F12, 0x0AF0);
write_cmos_sensor(0x6F12, 0xFF03);
write_cmos_sensor(0x6F12, 0x1FFA);
write_cmos_sensor(0x6F12, 0x89F2);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0x9E51);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0x8C01);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0x9081);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xC2C1);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0xB0B1);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xB441);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xE671);
write_cmos_sensor(0x6F12, 0x1C98);
write_cmos_sensor(0x6F12, 0xC1F8);
write_cmos_sensor(0x6F12, 0xD401);
write_cmos_sensor(0x6F12, 0xA1F8);
write_cmos_sensor(0x6F12, 0xD861);
write_cmos_sensor(0x6F12, 0x01F5);
write_cmos_sensor(0x6F12, 0xC271);
write_cmos_sensor(0x6F12, 0x0C46);
write_cmos_sensor(0x6F12, 0x6846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x48F9);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0x6846);
write_cmos_sensor(0x6F12, 0x1F9A);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x48F9);
write_cmos_sensor(0x6F12, 0x7D48);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x6F12, 0x38B1);
write_cmos_sensor(0x6F12, 0xDDE9);
write_cmos_sensor(0x6F12, 0x0912);
write_cmos_sensor(0x6F12, 0x6846);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x45F9);
write_cmos_sensor(0x6F12, 0x08B1);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x47F9);
write_cmos_sensor(0x6F12, 0x0021);
write_cmos_sensor(0x6F12, 0x7148);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x48F9);
write_cmos_sensor(0x6F12, 0x6F48);
write_cmos_sensor(0x6F12, 0x0121);
write_cmos_sensor(0x6F12, 0x2430);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x43F9);
write_cmos_sensor(0x6F12, 0x6D48);
write_cmos_sensor(0x6F12, 0x0221);
write_cmos_sensor(0x6F12, 0x4830);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x3EF9);
write_cmos_sensor(0x6F12, 0x0DB0);
write_cmos_sensor(0x6F12, 0xC3E6);
write_cmos_sensor(0x6F12, 0x6E48);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0xC110);
write_cmos_sensor(0x6F12, 0x19B1);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0x5700);
write_cmos_sensor(0x6F12, 0x1128);
write_cmos_sensor(0x6F12, 0x01D0);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x7047);
write_cmos_sensor(0x6F12, 0x0120);
write_cmos_sensor(0x6F12, 0x7047);
write_cmos_sensor(0x6F12, 0x10B5);
write_cmos_sensor(0x6F12, 0xE8B1);
write_cmos_sensor(0x6F12, 0x6949);
write_cmos_sensor(0x6F12, 0x2822);
write_cmos_sensor(0x6F12, 0xA1F5);
write_cmos_sensor(0x6F12, 0x5670);
write_cmos_sensor(0x6F12, 0x0446);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x2CF9);
write_cmos_sensor(0x6F12, 0x604A);
write_cmos_sensor(0x6F12, 0x6249);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x1070);
write_cmos_sensor(0x6F12, 0x0888);
write_cmos_sensor(0x6F12, 0x50B1);
write_cmos_sensor(0x6F12, 0x4888);
write_cmos_sensor(0x6F12, 0x30B9);
write_cmos_sensor(0x6F12, 0xFFF7);
write_cmos_sensor(0x6F12, 0xE1FF);
write_cmos_sensor(0x6F12, 0x28B1);
write_cmos_sensor(0x6F12, 0x5C48);
write_cmos_sensor(0x6F12, 0x90F8);
write_cmos_sensor(0x6F12, 0x7E04);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x0120);
write_cmos_sensor(0x6F12, 0x1070);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1EF9);
write_cmos_sensor(0x6F12, 0x2046);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x20F9);
write_cmos_sensor(0x6F12, 0x06E0);
write_cmos_sensor(0x6F12, 0x5A49);
write_cmos_sensor(0x6F12, 0x0E22);
write_cmos_sensor(0x6F12, 0x1831);
write_cmos_sensor(0x6F12, 0xA1F5);
write_cmos_sensor(0x6F12, 0x5670);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x0EF9);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1BF9);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1EF9);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x21F9);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0x1040);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x22B9);
write_cmos_sensor(0x6F12, 0x70B5);
write_cmos_sensor(0x6F12, 0x524D);
write_cmos_sensor(0x6F12, 0x534C);
write_cmos_sensor(0x6F12, 0x2888);
write_cmos_sensor(0x6F12, 0x2081);
write_cmos_sensor(0x6F12, 0xA081);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1FF9);
write_cmos_sensor(0x6F12, 0x2080);
write_cmos_sensor(0x6F12, 0x2888);
write_cmos_sensor(0x6F12, 0xE080);
write_cmos_sensor(0x6F12, 0x0320);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1EF9);
write_cmos_sensor(0x6F12, 0x0820);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x20F9);
write_cmos_sensor(0x6F12, 0x4C4D);
write_cmos_sensor(0x6F12, 0xA080);
write_cmos_sensor(0x6F12, 0x287D);
write_cmos_sensor(0x6F12, 0x0328);
write_cmos_sensor(0x6F12, 0x06D1);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x2875);
write_cmos_sensor(0x6F12, 0x4A48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1BF9);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x1EF9);
write_cmos_sensor(0x6F12, 0x287D);
write_cmos_sensor(0x6F12, 0x58BB);
write_cmos_sensor(0x6F12, 0x474C);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x6709);
write_cmos_sensor(0x6F12, 0x68B9);
write_cmos_sensor(0x6F12, 0x4648);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x6F12, 0x50B9);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x6C09);
write_cmos_sensor(0x6F12, 0x38B9);
write_cmos_sensor(0x6F12, 0x3A48);
write_cmos_sensor(0x6F12, 0x007A);
write_cmos_sensor(0x6F12, 0x10B1);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x5C09);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x18B1);
write_cmos_sensor(0x6F12, 0x1021);
write_cmos_sensor(0x6F12, 0x3D48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x0BF9);
write_cmos_sensor(0x6F12, 0x94F8);
write_cmos_sensor(0x6F12, 0x6909);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x18B1);
write_cmos_sensor(0x6F12, 0x4021);
write_cmos_sensor(0x6F12, 0x3848);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x02F9);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0x05F9);
write_cmos_sensor(0x6F12, 0x08B9);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x38B1);
write_cmos_sensor(0x6F12, 0x0C21);
write_cmos_sensor(0x6F12, 0x3448);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF9F8);
write_cmos_sensor(0x6F12, 0x0121);
write_cmos_sensor(0x6F12, 0x3248);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF5F8);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xFDF8);
write_cmos_sensor(0x6F12, 0x687D);
write_cmos_sensor(0x6F12, 0x0028);
write_cmos_sensor(0x6F12, 0x04D0);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0x7040);
write_cmos_sensor(0x6F12, 0x3048);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xFAB8);
write_cmos_sensor(0x6F12, 0x70BD);
write_cmos_sensor(0x6F12, 0x8908);
write_cmos_sensor(0x6F12, 0x8900);
write_cmos_sensor(0x6F12, 0x41EA);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x82B2);
write_cmos_sensor(0x6F12, 0x3E21);
write_cmos_sensor(0x6F12, 0x46F2);
write_cmos_sensor(0x6F12, 0x4420);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF4B8);
write_cmos_sensor(0x6F12, 0x10B5);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0xAB41);
write_cmos_sensor(0x6F12, 0x2848);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xF2F8);
write_cmos_sensor(0x6F12, 0x184C);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0xB531);
write_cmos_sensor(0x6F12, 0x6060);
write_cmos_sensor(0x6F12, 0x2548);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xEAF8);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x8921);
write_cmos_sensor(0x6F12, 0xA060);
write_cmos_sensor(0x6F12, 0x2248);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xE3F8);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x0F21);
write_cmos_sensor(0x6F12, 0xE060);
write_cmos_sensor(0x6F12, 0x2048);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xDCF8);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x5F11);
write_cmos_sensor(0x6F12, 0x1E48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xD6F8);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x6F12, 0x2146);
write_cmos_sensor(0x6F12, 0x0122);
write_cmos_sensor(0x6F12, 0x0870);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x1111);
write_cmos_sensor(0x6F12, 0x1A48);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xCDF8);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0xAFF2);
write_cmos_sensor(0x6F12, 0x7101);
write_cmos_sensor(0x6F12, 0xBDE8);
write_cmos_sensor(0x6F12, 0x1040);
write_cmos_sensor(0x6F12, 0x1748);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x6F12, 0xC5B8);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x9C68);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x46B4);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x5870);
write_cmos_sensor(0x6F12, 0x4000);
write_cmos_sensor(0x6F12, 0x9D62);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x68C0);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0xF000);
write_cmos_sensor(0x6F12, 0x2001);
write_cmos_sensor(0x6F12, 0x45B0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x12F0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x6D34);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xED50);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x61F0);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x2390);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x3A70);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xBA00);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0xED60);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x9420);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x7547);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x7361);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x574D);
write_cmos_sensor(0x6F12, 0x0002);
write_cmos_sensor(0x6F12, 0x432D);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0xB127);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x7E15);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x6D83);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0xD91C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xCD3C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xDD3C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xD53C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F2);
write_cmos_sensor(0x6F12, 0x3D3C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0xE95C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x391C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x45F2);
write_cmos_sensor(0x6F12, 0x4D7C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x43F6);
write_cmos_sensor(0x6F12, 0xB76C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x43F6);
write_cmos_sensor(0x6F12, 0xE96C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xA90C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xB92C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xF72C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x46F2);
write_cmos_sensor(0x6F12, 0xED1C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x45F6);
write_cmos_sensor(0x6F12, 0xE51C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4BF2);
write_cmos_sensor(0x6F12, 0x770C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0xC76C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4AF2);
write_cmos_sensor(0x6F12, 0xC17C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4AF2);
write_cmos_sensor(0x6F12, 0x797C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x691C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x41F2);
write_cmos_sensor(0x6F12, 0x331C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x44F2);
write_cmos_sensor(0x6F12, 0xE53C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x020C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x554C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0x6D7C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0x975C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F2);
write_cmos_sensor(0x6F12, 0x476C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x40F6);
write_cmos_sensor(0x6F12, 0xBF6C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x47F6);
write_cmos_sensor(0x6F12, 0xC55C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x43F6);
write_cmos_sensor(0x6F12, 0xBB1C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x010C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x48F2);
write_cmos_sensor(0x6F12, 0x551C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x4BF6);
write_cmos_sensor(0x6F12, 0x152C);
write_cmos_sensor(0x6F12, 0xC0F2);
write_cmos_sensor(0x6F12, 0x000C);
write_cmos_sensor(0x6F12, 0x6047);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0841);
write_cmos_sensor(0x6F12, 0x02BA);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x003F);
write_cmos_sensor(0x602A, 0xF000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0004);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x6028, 0x2000);
write_cmos_sensor(0x602A, 0x2500);
write_cmos_sensor(0x6F12, 0x0080);
write_cmos_sensor(0x602A, 0x10B8);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x602A, 0x1EE0);
write_cmos_sensor(0x6F12, 0x0078);
write_cmos_sensor(0x602A, 0x2870);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x250A);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x23A0);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x3022);
write_cmos_sensor(0x6F12, 0x1281);
write_cmos_sensor(0x602A, 0x32E8);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x54A2);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x120E);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x1212);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x2860);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x3220);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x1226);
write_cmos_sensor(0x6F12, 0x0301);
write_cmos_sensor(0x602A, 0x29C8);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x32EC);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x12BE);
write_cmos_sensor(0x6F12, 0x0101);
write_cmos_sensor(0x602A, 0x3034);
write_cmos_sensor(0x6F12, 0x049B);
write_cmos_sensor(0x602A, 0x1230);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x1232);
write_cmos_sensor(0x6F12, 0x00F0);
write_cmos_sensor(0x602A, 0x1236);
write_cmos_sensor(0x6F12, 0x01FF);
write_cmos_sensor(0x602A, 0x123A);
write_cmos_sensor(0x6F12, 0x0004);
write_cmos_sensor(0x602A, 0x123E);
write_cmos_sensor(0x6F12, 0xF45A);
write_cmos_sensor(0x602A, 0x1EE2);
write_cmos_sensor(0x6F12, 0x19CD);
write_cmos_sensor(0x602A, 0x115E);
write_cmos_sensor(0x6F12, 0x0048);
write_cmos_sensor(0x602A, 0x131C);
write_cmos_sensor(0x6F12, 0x2400);
write_cmos_sensor(0x602A, 0x2872);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x1314);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x20DE);
write_cmos_sensor(0x6F12, 0x0003);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x602A, 0x2108);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x6F12, 0x0022);
write_cmos_sensor(0x6F12, 0x0011);
write_cmos_sensor(0x602A, 0x1EDC);
write_cmos_sensor(0x6F12, 0x5008);
write_cmos_sensor(0x602A, 0x138E);
write_cmos_sensor(0x6F12, 0x13C0);
write_cmos_sensor(0x602A, 0x1392);
write_cmos_sensor(0x6F12, 0x0038);
write_cmos_sensor(0x602A, 0x21B6);
write_cmos_sensor(0x6F12, 0x0002);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x2550);
write_cmos_sensor(0x6F12, 0x193C);
write_cmos_sensor(0x6028, 0x4000);
write_cmos_sensor(0x0BC0, 0x0040);
write_cmos_sensor(0x0FE8, 0x49C1);
write_cmos_sensor(0x0FEA, 0x0040);
write_cmos_sensor(0x0BC8, 0x0001);
write_cmos_sensor(0x0B0A, 0x0101);
write_cmos_sensor(0x0BC6, 0x0000);
write_cmos_sensor(0x0B06, 0x0101);
write_cmos_sensor(0xF446, 0x000C);
write_cmos_sensor(0xF448, 0x0018);
write_cmos_sensor(0xF450, 0x0010);
write_cmos_sensor(0xF44E, 0x0000);
write_cmos_sensor(0xF468, 0xE000);

write_cmos_sensor(0x0100, 0x0000);
LOG_INF("Out123\n");



}				/*      sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");

	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0xF9F0);
	write_cmos_sensor(0x6218, 0xE150);
	write_cmos_sensor(0x6242, 0x0E00);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x12F2);
	write_cmos_sensor(0x6F12, 0x0D10);
	write_cmos_sensor(0x6F12, 0x0A18);
	write_cmos_sensor(0x6F12, 0x19B0);
	write_cmos_sensor(0x6F12, 0x1350);
	write_cmos_sensor(0x602A, 0x1EB6);
	write_cmos_sensor(0x6F12, 0x0206);
	write_cmos_sensor(0x602A, 0x3770);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1EB8);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x602A, 0x131E);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x3DEA);
	write_cmos_sensor(0x6F12, 0x0081);
	write_cmos_sensor(0x602A, 0x11A6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x602A, 0x11AE);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x13FC);
	write_cmos_sensor(0x6F12, 0x0044);
	write_cmos_sensor(0x6F12, 0x0064);
	write_cmos_sensor(0x6F12, 0x0044);
	write_cmos_sensor(0x602A, 0x3302);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x27D2);
	write_cmos_sensor(0x6F12, 0x0203);
	write_cmos_sensor(0x602A, 0x1EC8);
	write_cmos_sensor(0x6F12, 0x0503);
	write_cmos_sensor(0x6F12, 0x0504);
	write_cmos_sensor(0x602A, 0x1ED2);
	write_cmos_sensor(0x6F12, 0x080F);
	write_cmos_sensor(0x602A, 0x1ED6);
	write_cmos_sensor(0x6F12, 0x0307);
	write_cmos_sensor(0x602A, 0x123C);
	write_cmos_sensor(0x6F12, 0x0009);
	write_cmos_sensor(0x602A, 0x21BE);
	write_cmos_sensor(0x6F12, 0x04D2);
	write_cmos_sensor(0x6F12, 0x41A6);
	write_cmos_sensor(0x602A, 0x1EE0);
	write_cmos_sensor(0x6F12, 0x006C);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF466, 0x0E0D);
	write_cmos_sensor(0x0344, 0x0018);
	write_cmos_sensor(0x0346, 0x0010);
	write_cmos_sensor(0x0348, 0x1997);
	write_cmos_sensor(0x034A, 0x133E);
	write_cmos_sensor(0x034C, 0x0CC0);
	write_cmos_sensor(0x034E, 0x0990);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x0004);
	write_cmos_sensor(0x0900, 0x0112);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x0400, 0x2010);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0116, 0x3000);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x011C, 0x0100);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0002);
	write_cmos_sensor(0x0302, 0x0003);
	write_cmos_sensor(0x0304, 0x0004);
	write_cmos_sensor(0x0306, 0x011E);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0002);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0120);
	write_cmos_sensor(0x0312, 0x0002);
	write_cmos_sensor(0x0340, 0x0A40);
	write_cmos_sensor(0x0342, 0x38C0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x022C, 0x0100);
	write_cmos_sensor(0x0226, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x3020);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0B00, 0x0080);
	write_cmos_sensor(0x0B08, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0000);
	write_cmos_sensor(0x0D04, 0x0000);


}				/*      preview_setting  */


static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);

	preview_setting();
	#if 0
	write_cmos_sensor(0x0100, 0x0000);
	mdelay(30);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6242, 0x0E00);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x12F2);
	write_cmos_sensor(0x6F12, 0x0D10);
	write_cmos_sensor(0x6F12, 0x0A18);
	write_cmos_sensor(0x6F12, 0x19B0);
	write_cmos_sensor(0x6F12, 0x1350);
	write_cmos_sensor(0x602A, 0x1EB6);
	write_cmos_sensor(0x6F12, 0x0206);
	write_cmos_sensor(0x602A, 0x3770);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1EB8);
	write_cmos_sensor(0x6F12, 0x0300);
	write_cmos_sensor(0x602A, 0x131E);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x3DEA);
	write_cmos_sensor(0x6F12, 0x0081);
	write_cmos_sensor(0x602A, 0x11A6);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6F12, 0x0098);
	write_cmos_sensor(0x602A, 0x11AE);
	write_cmos_sensor(0x6F12, 0x0088);
	write_cmos_sensor(0x602A, 0x13FC);
	write_cmos_sensor(0x6F12, 0x0064);
	write_cmos_sensor(0x6F12, 0x0044);
	write_cmos_sensor(0x6F12, 0x0044);
	write_cmos_sensor(0x602A, 0x3302);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x27D2);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x602A, 0x1EC8);
	write_cmos_sensor(0x6F12, 0x0603);
	write_cmos_sensor(0x6F12, 0x0504);
	write_cmos_sensor(0x602A, 0x1ED2);
	write_cmos_sensor(0x6F12, 0x080F);
	write_cmos_sensor(0x602A, 0x1ED6);
	write_cmos_sensor(0x6F12, 0x0307);
	write_cmos_sensor(0x602A, 0x123C);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x602A, 0x21C0);
	write_cmos_sensor(0x6F12, 0x41AE);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF466, 0x0FFD);
	write_cmos_sensor(0x0344, 0x0008);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x19A7);
	write_cmos_sensor(0x034A, 0x134F);
	write_cmos_sensor(0x034C, 0x19A0);
	write_cmos_sensor(0x034E, 0x1338);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x000C);
	write_cmos_sensor(0x0900, 0x0111);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0116, 0x3000);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x011C, 0x0100);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0002);
	write_cmos_sensor(0x0302, 0x0003);
	write_cmos_sensor(0x0304, 0x0004);
	write_cmos_sensor(0x0306, 0x011E);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0002);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x014C);
	write_cmos_sensor(0x0312, 0x0002);
	write_cmos_sensor(0x0340, 0x13FE);
	write_cmos_sensor(0x0342, 0x4EB0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x022C, 0x0100);
	write_cmos_sensor(0x0226, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0B00, 0x0080);
	write_cmos_sensor(0x0B08, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0000);
	write_cmos_sensor(0x0D04, 0x0000);
	#endif


}



static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	/* Reset for operation */

	preview_setting();


}



static void hs_video_setting(void)
{
	LOG_INF("E! VGA 120fps\n");

	preview_setting();



}




static void slim_video_setting(void)
{
	LOG_INF("E! HD 30fps\n");

	preview_setting();


}

static kal_uint32 streaming_control(kal_bool enable)
{
	kal_uint16 val = 0;
	int index = 0;
	LOG_INF("streaming_enable123 fix (0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		for (index = 0; index < OFF_READY_DELAY; index++) {
			val = read_cmos_sensor(0x0005);
			LOG_INF("mtk_test: 0x0005 = 0x%x\n", val);
			if (val == 0xff) {
				LOG_INF("mtk_test: index = %d\n", index);
				break;
			}
			mdelay(5);
		}
		write_cmos_sensor(0x0100, 0x0100);
	}
	else
		write_cmos_sensor(0x0100, 0x0000);

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, i2c_write_id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	/* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;

	/* sensor have two i2c address 0x20,0x5a  we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, i2c write id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;	/*  */
	imgsensor.gain = 0x100;	/*  */
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*      close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	/*set_mirror_flip(imgsensor.mirror);*/
	return ERROR_NONE;
}				/*      preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&imgsensor_drv_lock);
	/* imgsensor.current_fps = 240; */
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/


	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);

	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}

/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{


	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}				/*      slim_video       */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{

	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/* LOG_INF("scenario_id = %d\n", scenario_id); */



	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;


	/*0: no support, 1: G0,R0.B0, 2: G0,R0.B1, 3: G0,R1.B0, 4: G0,R1.B1 */
	/*                    5: G1,R0.B0, 6: G1,R0.B1, 7: G1,R1.B0, 8: G1,R1.B1 */
	sensor_info->ZHDR_Mode = 5;


	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* LOG_INF("framerate = %d\n ", framerate); */
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	kal_uint32 frame_length;

	/* LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate); */

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
		    imgsensor_info.normal_video.pclk / framerate * 10 /
		    imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length > imgsensor_info.normal_video.framelength)
			    ? (frame_length - imgsensor_info.normal_video.framelength) : 0;

		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length =
			    imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength)
				? (frame_length - imgsensor_info.cap1.framelength) : 0;

			imgsensor.frame_length =
			    imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
			frame_length =
			    imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			    (frame_length > imgsensor_info.cap2.framelength)
			    ? (frame_length - imgsensor_info.cap2.framelength) : 0;

			imgsensor.frame_length =
			    imgsensor_info.cap2.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				     framerate, imgsensor_info.cap.max_framerate / 10);

			frame_length =
			    imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength)
				? (frame_length - imgsensor_info.cap.framelength) : 0;

			imgsensor.frame_length =
			    imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length =
		    imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length > imgsensor_info.hs_video.framelength)
		    ? (frame_length - imgsensor_info.hs_video.framelength) : 0;

		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length =
		    imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength) : 0;

		imgsensor.frame_length =
		    imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:		/* coding with  preview scenario by default */
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length > imgsensor_info.pre.framelength)
		    ? (frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
{

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	/* enable = false; */
	if (enable) {

		/* 0x0601[2:0]; 0=no pattern,1=solid colour,2 = 100% colour bar ,3 = Fade to gray' colour bar */
		write_cmos_sensor(0x0601, 0x02);
	} else {
		write_cmos_sensor(0x0601, 0x00);
	}
	write_cmos_sensor(0x3200, 0x00);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/* LOG_INF("feature_id = %d", feature_id); */
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		if ((sensor_reg_data->RegData >> 8) > 0)
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		else
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *(feature_data),
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (MUINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
		/* zhdr,wdrs */
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("hdr mode :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.hdr_mode = (UINT8)*feature_data_32;
		/* imgsensor.hdr_mode = 9;                                               //force set hdr_mode to zHDR */
		spin_unlock(&imgsensor_drv_lock);
		/* LOG_INF("hdr mode :%d\n", imgsensor.hdr_mode); */
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (MUINT32) *feature_data);
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data, (UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		hdr_write_shutter((UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
		streaming_control(KAL_TRUE);
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*      feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KGD1SP_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*      S5K4E6_MIPI_RAW_SensorInit      */
