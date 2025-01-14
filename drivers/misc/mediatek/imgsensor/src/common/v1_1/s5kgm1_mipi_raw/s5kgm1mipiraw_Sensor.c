/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5Kgm1mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6735
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *  20150624: the first driver from 
 *  20150706: add pip 15fps setting
 *  20150716: 更新log的打印方法
 *  20150720: use non - continue mode
 *  15072011511229: add pdaf, the pdaf old has be delete by recovery
 *  15072011511229: add 旧的log兼容，新的log在这个版本不能打印log？？
 *  15072209190629: non - continue mode bandwith limited , has <tiaowen> , modify to continue mode
 *  15072209201129: modify not enter init_setting bug
 *  15072718000000: crc addd 0x49c09f86
 *  15072718000001: MODIFY LOG SWITCH
 *  15072811330000: ADD NON-CONTIUE MODE ,PREVIEW 29FPS,CAPTURE 29FPS
 					([TODO]REG0304 0786->0780  PREVEIW INCREASE TO 30FPS)
 *  15072813000000: modify a wrong setting at pip reg030e 0x119->0xc8
 *  15080409230000: pass!
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
#include <linux/types.h>

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "s5kgm1mipiraw_Sensor.h"

/*===FEATURE SWITH===*/
#define FPTPDAFSUPPORT   //for pdaf switch
#define FANPENGTAO  1  //for debug log
//#define LOG_INF LOG_INF_LOD
//#define NONCONTINUEMODE
/*===FEATURE SWITH===*/

#define PFX "S5KGM1_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	pr_err(PFX "[%s] " format, __func__, ##args)
//static kal_uint32 streaming_control(kal_bool enable);
//#define SENSORDB LOG_INF
/****************************   Modify end    *******************************************/
//static bool bIsLongExposure = KAL_FALSE;
//extern bool read_gm1_eeprom(kal_uint16 addr, BYTE *data, kal_uint32 size);

//[agold][chenwei][20190313][start]
//extern bool s5kgm1kwCheckLensVersion(int id);

#ifndef AGOLD_S5KGM1_AWB_DISABLE
extern void s5kgm1_get_otp_data(void);
#endif
//[agold][chenwei][20190313][end]

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KGM1_SENSOR_ID,		//Sensor ID Value: 0x30C8//record sensor id defined in Kd_imgsensor.h

	.checksum_value = 0x49c09f86,		//checksum value for Camera Auto Test

	.pre = {
		.pclk = 482000000,				//record different mode's pclk
		.linelength  = 9648,				//record different mode's linelength
		.framelength = 1664,			//record different mode's framelength
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2000,		//record different mode's width of grabwindow
		.grabwindow_height = 1500,		//record different mode's height of grabwindow
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 240000000,
		.max_framerate = 300,    			
	},

	.cap = {
		.pclk = 482000000,				//record different mode's pclk
		.linelength  = 5024,				//record different mode's linelength
		.framelength = 3194,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4000,		//record different mode's width of grabwindow
		.grabwindow_height = 3000,		//record different mode's height of grabwindow
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 460800000,
		.max_framerate = 300,			
	},

	.normal_video = {
		.pclk = 482000000,				//record different mode's pclk
		.linelength  = 9648,				//record different mode's linelength
		.framelength = 1664,			//record different mode's framelength
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2000,		//record different mode's width of grabwindow
		.grabwindow_height = 1500,		//record different mode's height of grabwindow
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 240000000,
		.max_framerate = 300, 		 
	},
	.hs_video = {
		.pclk = 492000000,				//record different mode's pclk
		.linelength  = 5024,				//record different mode's linelength
		.framelength = 816,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,		//record different mode's height of grabwindow
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 196800000,
		.max_framerate = 1200,				
	},
	.slim_video = {
		/*setting for normal binning*/
		.pclk = 482000000,
		.linelength = 5024,
		.framelength = 3194,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 460800000,
		.max_framerate = 300,
	},
	.custom1 = {
			.pclk = 482000000,				//record different mode's pclk
			.linelength  = 5024,				//record different mode's linelength
			.framelength = 3194,			//record different mode's framelength
			.startx = 0,					//record different mode's startx of grabwindow
			.starty = 0,					//record different mode's starty of grabwindow
			.grabwindow_width  = 3744,		//record different mode's width of grabwindow
			.grabwindow_height = 2808,		//record different mode's height of grabwindow
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 460800000,
			.max_framerate = 300,			
		},
	.margin = 5,			                //sensor framelength & shutter margin
	.min_shutter = 5,		              //min shutter
	.min_gain = 64,
	.max_gain = 1024,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 2,
	.max_frame_length = 0xFFFF,       //REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	        //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,  //sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,      //isp gain delay frame for AE cycle
	.frame_time_delay_frame = 2,
	.ihdr_support = 0,	              //1, support; 0,not support
	.ihdr_le_firstline = 0,           //1,le first ; 0, se first
	.sensor_mode_num = 6,	            //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	.custom1_delay_frame = 2,	/* add new mode */
	.isp_driving_current = ISP_DRIVING_4MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x20,0x5A,0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
    .i2c_speed = 400, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x0200,					//current shutter
	.gain = 0x0100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x5A,//record current sensor's i2c write id
};


/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] =	 
{
 { 4000, 3000,	 0,    0,  4000, 3000, 2000, 1500,   0,	0, 2000, 1500, 	 0, 0, 2000, 1500}, // Preview 
 { 4000, 3000,	 0,    0,  4000, 3000, 4000, 3000,   0,	0, 4000, 3000, 	 0, 0, 4000, 3000}, // capture 
 { 4000, 3000,	 0,    0,  4000, 3000, 2000, 1500,   0,	0, 2000, 1500, 	 0, 0, 2000, 1500}, // normal video 
 { 4000, 3000, 720,  780,  2560, 1440, 1280,  720,   0,	0, 1280,  720, 	 0, 0, 1280,  720}, //high speed video_720p
 { 4000, 3000,	 0,    0,  4000, 3000, 4000, 3000,   0,	0, 4000, 3000, 	 0, 0, 4000, 3000},/* slim video*/
 { 4000, 3000, 128,   96,  3744, 2808, 3744, 2808,   0,	0, 3744, 2808, 	 0, 0, 3744, 2808}, // custom1 
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
//for 3m5, need to modify
{
 
    .i4OffsetX = 16,

    .i4OffsetY = 28,
 
    .i4PitchX = 32,
 
    .i4PitchY = 32,
 
    .i4PairNum =16,

    .i4SubBlkW =8,
 
    .i4SubBlkH =8, 

    .i4PosL = {{18,29},{26,29},{34,29},{42,29},{22,41},{30,41},{38,41},{46,41},{18,49},{26,49},{34,49},{42,49},{22,53},{30,53},{38,53},{46,53}},
 
    .i4PosR = {{18,33},{26,33},{34,33},{42,33},{22,37},{30,37},{38,37},{46,37},{18,45},{26,45},{34,45},{42,45},{22,57},{30,57},{38,57},{46,57}},

    .i4BlockNumX = 124,
    .i4BlockNumY = 92,
 
};

static kal_uint32 ana_gain_table_8x[] = {
	100000,
	100196,
	100392,
	100589,
	100787,
	100986,
	101186,
	101386,
	101587,
	101789,
	101992,
	102196,
	102400,
	102605,
	102811,
	103018,
	103226,
	103434,
	103644,
	103854,
	104065,
	104277,
	104490,
	104703,
	104918,
	105133,
	105350,
	105567,
	105785,
	106004,
	106224,
	106445,
	106667,
	106889,
	107113,
	107338,
	107563,
	107789,
	108017,
	108245,
	108475,
	108705,
	108936,
	109168,
	109402,
	109636,
	109871,
	110108,
	110345,
	110583,
	110823,
	111063,
	111304,
	111547,
	111790,
	112035,
	112281,
	112527,
	112775,
	113024,
	113274,
	113525,
	113778,
	114031,
	114286,
	114541,
	114798,
	115056,
	115315,
	115576,
	115837,
	116100,
	116364,
	116629,
	116895,
	117162,
	117431,
	117701,
	117972,
	118245,
	118519,
	118794,
	119070,
	119347,
	119626,
	119906,
	120188,
	120471,
	120755,
	121040,
	121327,
	121615,
	121905,
	122196,
	122488,
	122782,
	123077,
	123373,
	123671,
	123971,
	124272,
	124574,
	124878,
	125183,
	125490,
	125799,
	126108,
	126420,
	126733,
	127047,
	127363,
	127681,
	128000,
	128321,
	128643,
	128967,
	129293,
	129620,
	129949,
	130280,
	130612,
	130946,
	131282,
	131620,
	131959,
	132300,
	132642,
	132987,
	133333,
	133681,
	134031,
	134383,
	134737,
	135092,
	135450,
	135809,
	136170,
	136533,
	136898,
	137265,
	137634,
	138005,
	138378,
	138753,
	139130,
	139510,
	139891,
	140274,
	140659,
	141047,
	141436,
	141828,
	142222,
	142618,
	143017,
	143417,
	143820,
	144225,
	144633,
	145042,
	145455,
	145869,
	146286,
	146705,
	147126,
	147550,
	147977,
	148406,
	148837,
	149271,
	149708,
	150147,
	150588,
	151032,
	151479,
	151929,
	152381,
	152836,
	153293,
	153754,
	154217,
	154683,
	155152,
	155623,
	156098,
	156575,
	157055,
	157538,
	158025,
	158514,
	159006,
	159502,
	160000,
	160502,
	161006,
	161514,
	162025,
	162540,
	163057,
	163578,
	164103,
	164630,
	165161,
	165696,
	166234,
	166775,
	167320,
	167869,
	168421,
	168977,
	169536,
	170100,
	170667,
	171237,
	171812,
	172391,
	172973,
	173559,
	174150,
	174744,
	175342,
	175945,
	176552,
	177163,
	177778,
	178397,
	179021,
	179649,
	180282,
	180919,
	181560,
	182206,
	182857,
	183513,
	184173,
	184838,
	185507,
	186182,
	186861,
	187546,
	188235,
	188930,
	189630,
	190335,
	191045,
	191760,
	192481,
	193208,
	193939,
	194677,
	195420,
	196169,
	196923,
	197683,
	198450,
	199222,
	200000,
	200784,
	201575,
	202372,
	203175,
	203984,
	204800,
	205622,
	206452,
	207287,
	208130,
	208980,
	209836,
	210700,
	211570,
	212448,
	213333,
	214226,
	215126,
	216034,
	216949,
	217872,
	218803,
	219742,
	220690,
	221645,
	222609,
	223581,
	224561,
	225551,
	226549,
	227556,
	228571,
	229596,
	230631,
	231674,
	232727,
	233790,
	234862,
	235945,
	237037,
	238140,
	239252,
	240376,
	241509,
	242654,
	243810,
	244976,
	246154,
	247343,
	248544,
	249756,
	250980,
	252217,
	253465,
	254726,
	256000,
	257286,
	258586,
	259898,
	261224,
	262564,
	263918,
	265285,
	266667,
	268063,
	269474,
	270899,
	272340,
	273797,
	275269,
	276757,
	278261,
	279781,
	281319,
	282873,
	284444,
	286034,
	287640,
	289266,
	290909,
	292571,
	294253,
	295954,
	297674,
	299415,
	301176,
	302959,
	304762,
	306587,
	308434,
	310303,
	312195,
	314110,
	316049,
	318012,
	320000,
	322013,
	324051,
	326115,
	328205,
	330323,
	332468,
	334641,
	336842,
	339073,
	341333,
	343624,
	345946,
	348299,
	350685,
	353103,
	355556,
	358042,
	360563,
	363121,
	365714,
	368345,
	371014,
	373723,
	376471,
	379259,
	382090,
	384962,
	387879,
	390840,
	393846,
	396899,
	400000,
	403150,
	406349,
	409600,
	412903,
	416260,
	419672,
	423140,
	426667,
	430252,
	433898,
	437607,
	441379,
	445217,
	449123,
	453097,
	457143,
	461261,
	465455,
	469725,
	474074,
	478505,
	483019,
	487619,
	492308,
	497087,
	501961,
	506931,
	512000,
	517172,
	522449,
	527835,
	533333,
	538947,
	544681,
	550538,
	556522,
	562637,
	568889,
	575281,
	581818,
	588506,
	595349,
	602353,
	609524,
	616867,
	624390,
	632099,
	640000,
	648101,
	656410,
	664935,
	673684,
	682667,
	691892,
	701370,
	711111,
	721127,
	731429,
	742029,
	752941,
	764179,
	775758,
	787692,
	800000,
};

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable(%d) \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

#if 1
static void check_streamoff(void)
{
	unsigned int i = 0;
	int timeout = (10000 / imgsensor.current_fps) + 1;

	mdelay(3);
	for (i = 0; i < timeout; i++) {
		if (read_cmos_sensor_byte(0x0005) != 0xFF)
			mdelay(1);
		else
			break;
	}
	LOG_INF(" check_streamoff exit!\n");
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x6214, 0x7970);
		write_cmos_sensor_byte(0x0100, 0X01);
	} else {
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor_byte(0x0100, 0x00);
		check_streamoff();
	}
	return ERROR_NONE;
}
#endif

#if 0
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, (shutter) & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */
#endif

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	 spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	if (frame_length > 1)
		imgsensor.frame_length = frame_length;
/* */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
		
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter = (imgsensor_info.max_frame_length - imgsensor_info.margin);

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length*/
			write_cmos_sensor(0x0340, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length*/
		 write_cmos_sensor(0x0340, imgsensor.frame_length);
	}
	/* Update Shutter*/
	write_cmos_sensor(0x0202, shutter);

	LOG_INF("Add for N3D! shutterlzl =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}

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
	kal_uint16 realtime_fps = 0;
	//kal_uint32 frame_length = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	//write_shutter(shutter);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0X0202, shutter & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	//gain = 64 = 1x real gain.
    reg_gain = gain/2;
	//reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
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
  	//gain = 64 = 1x real gain.
	kal_uint16 reg_gain;
	
	LOG_INF("set_gain %d \n", gain);
	
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;		 
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));    
	return gain;
}	/*	set_gain  */

//ihdr_write_shutter_gain not support for S5K3m5
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
		// Extend frame length first
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3512, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3511, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3510, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}



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
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor_byte(0x0101,0x00); //GR
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor_byte(0x0101,0x01); //R
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor_byte(0x0101,0x02); //B	
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor_byte(0x0101,0x03); //GB
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
			break;
	}

}

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
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
static void sensor_init(void)
{
  LOG_INF("E\n"); 
  write_cmos_sensor(0x6028, 0x4000);      //               
  write_cmos_sensor(0x0000, 0x0009);      //version        
  write_cmos_sensor(0x0000, 0x08D1);      //sensor ID      
  write_cmos_sensor(0x6010, 0x0001);      //Open Clock                  
  mdelay(3);                              //delay 3ms     
  write_cmos_sensor(0x6214, 0x7971);
  write_cmos_sensor(0x6218, 0x7150);
  write_cmos_sensor(0x0A02, 0x0074);      //OTP page select
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x3F5C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0549);
  write_cmos_sensor(0x6F12, 0x0448);
  write_cmos_sensor(0x6F12, 0x054A);
  write_cmos_sensor(0x6F12, 0xC1F8);
  write_cmos_sensor(0x6F12, 0x5005);
  write_cmos_sensor(0x6F12, 0x101A);
  write_cmos_sensor(0x6F12, 0xA1F8);
  write_cmos_sensor(0x6F12, 0x5405);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xCFB9);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x4470);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x2E30);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6E00);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xFF5F);
  write_cmos_sensor(0x6F12, 0xF848);
  write_cmos_sensor(0x6F12, 0x8B46);
  write_cmos_sensor(0x6F12, 0x1746);
  write_cmos_sensor(0x6F12, 0x0068);
  write_cmos_sensor(0x6F12, 0x9A46);
  write_cmos_sensor(0x6F12, 0x4FEA);
  write_cmos_sensor(0x6F12, 0x1049);
  write_cmos_sensor(0x6F12, 0x80B2);
  write_cmos_sensor(0x6F12, 0x8046);
  write_cmos_sensor(0x6F12, 0x0146);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x4846);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x02FA);
  write_cmos_sensor(0x6F12, 0xF24D);
  write_cmos_sensor(0x6F12, 0x95F8);
  write_cmos_sensor(0x6F12, 0x6D00);
  write_cmos_sensor(0x6F12, 0x0228);
  write_cmos_sensor(0x6F12, 0x35D0);
  write_cmos_sensor(0x6F12, 0x0224);
  write_cmos_sensor(0x6F12, 0xF04E);
  write_cmos_sensor(0x6F12, 0x5346);
  write_cmos_sensor(0x6F12, 0xB6F8);
  write_cmos_sensor(0x6F12, 0xB802);
  write_cmos_sensor(0x6F12, 0xB0FB);
  write_cmos_sensor(0x6F12, 0xF4F0);
  write_cmos_sensor(0x6F12, 0xA6F8);
  write_cmos_sensor(0x6F12, 0xB802);
  write_cmos_sensor(0x6F12, 0xD5F8);
  write_cmos_sensor(0x6F12, 0x1411);
  write_cmos_sensor(0x6F12, 0x06F5);
  write_cmos_sensor(0x6F12, 0x2E76);
  write_cmos_sensor(0x6F12, 0x6143);
  write_cmos_sensor(0x6F12, 0xC5F8);
  write_cmos_sensor(0x6F12, 0x1411);
  write_cmos_sensor(0x6F12, 0xB5F8);
  write_cmos_sensor(0x6F12, 0x8C11);
  write_cmos_sensor(0x6F12, 0x411A);
  write_cmos_sensor(0x6F12, 0x89B2);
  write_cmos_sensor(0x6F12, 0x25F8);
  write_cmos_sensor(0x6F12, 0x981B);
  write_cmos_sensor(0x6F12, 0x35F8);
  write_cmos_sensor(0x6F12, 0x142C);
  write_cmos_sensor(0x6F12, 0x6243);
  write_cmos_sensor(0x6F12, 0x521E);
  write_cmos_sensor(0x6F12, 0x00FB);
  write_cmos_sensor(0x6F12, 0x0210);
  write_cmos_sensor(0x6F12, 0xB5F8);
  write_cmos_sensor(0x6F12, 0xF210);
  write_cmos_sensor(0x6F12, 0x07FB);
  write_cmos_sensor(0x6F12, 0x04F2);
  write_cmos_sensor(0x6F12, 0x0844);
  write_cmos_sensor(0x6F12, 0xC5F8);
  write_cmos_sensor(0x6F12, 0xF800);
  write_cmos_sensor(0x6F12, 0x5946);
  write_cmos_sensor(0x6F12, 0x0098);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xDBF9);
  write_cmos_sensor(0x6F12, 0x3088);
  write_cmos_sensor(0x6F12, 0x4146);
  write_cmos_sensor(0x6F12, 0x6043);
  write_cmos_sensor(0x6F12, 0x3080);
  write_cmos_sensor(0x6F12, 0xE86F);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0xB0FB);
  write_cmos_sensor(0x6F12, 0xF4F0);
  write_cmos_sensor(0x6F12, 0xE867);
  write_cmos_sensor(0x6F12, 0x04B0);
  write_cmos_sensor(0x6F12, 0x4846);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xF05F);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xC7B9);
  write_cmos_sensor(0x6F12, 0x0124);
  write_cmos_sensor(0x6F12, 0xC8E7);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xF041);
  write_cmos_sensor(0x6F12, 0x8046);
  write_cmos_sensor(0x6F12, 0xD148);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x4168);
  write_cmos_sensor(0x6F12, 0x0D0C);
  write_cmos_sensor(0x6F12, 0x8EB2);
  write_cmos_sensor(0x6F12, 0x3146);
  write_cmos_sensor(0x6F12, 0x2846);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xB9F9);
  write_cmos_sensor(0x6F12, 0xD04C);
  write_cmos_sensor(0x6F12, 0xCE4F);
  write_cmos_sensor(0x6F12, 0x2078);
  write_cmos_sensor(0x6F12, 0x97F8);
  write_cmos_sensor(0x6F12, 0x8B12);
  write_cmos_sensor(0x6F12, 0x10FB);
  write_cmos_sensor(0x6F12, 0x01F0);
  write_cmos_sensor(0x6F12, 0x2070);
  write_cmos_sensor(0x6F12, 0x4046);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xB8F9);
  write_cmos_sensor(0x6F12, 0x2078);
  write_cmos_sensor(0x6F12, 0x97F8);
  write_cmos_sensor(0x6F12, 0x8B12);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0xB0FB);
  write_cmos_sensor(0x6F12, 0xF1F0);
  write_cmos_sensor(0x6F12, 0x2070);
  write_cmos_sensor(0x6F12, 0x3146);
  write_cmos_sensor(0x6F12, 0x2846);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xF041);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xA1B9);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xFF47);
  write_cmos_sensor(0x6F12, 0x8146);
  write_cmos_sensor(0x6F12, 0xBF48);
  write_cmos_sensor(0x6F12, 0x1746);
  write_cmos_sensor(0x6F12, 0x8846);
  write_cmos_sensor(0x6F12, 0x8068);
  write_cmos_sensor(0x6F12, 0x1C46);
  write_cmos_sensor(0x6F12, 0x85B2);
  write_cmos_sensor(0x6F12, 0x060C);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x2946);
  write_cmos_sensor(0x6F12, 0x3046);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x92F9);
  write_cmos_sensor(0x6F12, 0x2346);
  write_cmos_sensor(0x6F12, 0x3A46);
  write_cmos_sensor(0x6F12, 0x4146);
  write_cmos_sensor(0x6F12, 0x4846);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x9BF9);
  write_cmos_sensor(0x6F12, 0xBA4A);
  write_cmos_sensor(0x6F12, 0x9088);
  write_cmos_sensor(0x6F12, 0xF0B3);
  write_cmos_sensor(0x6F12, 0xB748);
  write_cmos_sensor(0x6F12, 0x90F8);
  write_cmos_sensor(0x6F12, 0xBA10);
  write_cmos_sensor(0x6F12, 0xD1B3);
  write_cmos_sensor(0x6F12, 0xD0F8);
  write_cmos_sensor(0x6F12, 0x2801);
  write_cmos_sensor(0x6F12, 0x1168);
  write_cmos_sensor(0x6F12, 0x8842);
  write_cmos_sensor(0x6F12, 0x00D3);
  write_cmos_sensor(0x6F12, 0x0846);
  write_cmos_sensor(0x6F12, 0x010A);
  write_cmos_sensor(0x6F12, 0xB1FA);
  write_cmos_sensor(0x6F12, 0x81F0);
  write_cmos_sensor(0x6F12, 0xC0F1);
  write_cmos_sensor(0x6F12, 0x1700);
  write_cmos_sensor(0x6F12, 0xC140);
  write_cmos_sensor(0x6F12, 0x02EB);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0xC9B2);
  write_cmos_sensor(0x6F12, 0x0389);
  write_cmos_sensor(0x6F12, 0xC288);
  write_cmos_sensor(0x6F12, 0x9B1A);
  write_cmos_sensor(0x6F12, 0x4B43);
  write_cmos_sensor(0x6F12, 0x8033);
  write_cmos_sensor(0x6F12, 0x02EB);
  write_cmos_sensor(0x6F12, 0x2322);
  write_cmos_sensor(0x6F12, 0x0092);
  write_cmos_sensor(0x6F12, 0x438A);
  write_cmos_sensor(0x6F12, 0x028A);
  write_cmos_sensor(0x6F12, 0x9B1A);
  write_cmos_sensor(0x6F12, 0x4B43);
  write_cmos_sensor(0x6F12, 0x8033);
  write_cmos_sensor(0x6F12, 0x02EB);
  write_cmos_sensor(0x6F12, 0x2322);
  write_cmos_sensor(0x6F12, 0x0192);
  write_cmos_sensor(0x6F12, 0x838B);
  write_cmos_sensor(0x6F12, 0x428B);
  write_cmos_sensor(0x6F12, 0x9B1A);
  write_cmos_sensor(0x6F12, 0x4B43);
  write_cmos_sensor(0x6F12, 0x8033);
  write_cmos_sensor(0x6F12, 0x02EB);
  write_cmos_sensor(0x6F12, 0x2322);
  write_cmos_sensor(0x6F12, 0x0292);
  write_cmos_sensor(0x6F12, 0xC28C);
  write_cmos_sensor(0x6F12, 0x808C);
  write_cmos_sensor(0x6F12, 0x121A);
  write_cmos_sensor(0x6F12, 0x4A43);
  write_cmos_sensor(0x6F12, 0x8032);
  write_cmos_sensor(0x6F12, 0x00EB);
  write_cmos_sensor(0x6F12, 0x2220);
  write_cmos_sensor(0x6F12, 0x0390);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x6846);
  write_cmos_sensor(0x6F12, 0x54F8);
  write_cmos_sensor(0x6F12, 0x2210);
  write_cmos_sensor(0x6F12, 0x50F8);
  write_cmos_sensor(0x6F12, 0x2230);
  write_cmos_sensor(0x6F12, 0x5943);
  write_cmos_sensor(0x6F12, 0x090B);
  write_cmos_sensor(0x6F12, 0x44F8);
  write_cmos_sensor(0x6F12, 0x2210);
  write_cmos_sensor(0x6F12, 0x521C);
  write_cmos_sensor(0x6F12, 0x00E0);
  write_cmos_sensor(0x6F12, 0x01E0);
  write_cmos_sensor(0x6F12, 0x042A);
  write_cmos_sensor(0x6F12, 0xF2D3);
  write_cmos_sensor(0x6F12, 0x04B0);
  write_cmos_sensor(0x6F12, 0x2946);
  write_cmos_sensor(0x6F12, 0x3046);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xF047);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x3FB9);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xF041);
  write_cmos_sensor(0x6F12, 0x954C);
  write_cmos_sensor(0x6F12, 0x9349);
  write_cmos_sensor(0x6F12, 0x0646);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x6970);
  write_cmos_sensor(0x6F12, 0x8988);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x8120);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0xC1B1);
  write_cmos_sensor(0x6F12, 0x2146);
  write_cmos_sensor(0x6F12, 0xD1F8);
  write_cmos_sensor(0x6F12, 0x9410);
  write_cmos_sensor(0x6F12, 0x72B1);
  write_cmos_sensor(0x6F12, 0x8FB1);
  write_cmos_sensor(0x6F12, 0x0846);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x3FF9);
  write_cmos_sensor(0x6F12, 0x0546);
  write_cmos_sensor(0x6F12, 0xE06F);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x3BF9);
  write_cmos_sensor(0x6F12, 0x8542);
  write_cmos_sensor(0x6F12, 0x02D2);
  write_cmos_sensor(0x6F12, 0xD4F8);
  write_cmos_sensor(0x6F12, 0x9400);
  write_cmos_sensor(0x6F12, 0x26E0);
  write_cmos_sensor(0x6F12, 0xE06F);
  write_cmos_sensor(0x6F12, 0x24E0);
  write_cmos_sensor(0x6F12, 0x002F);
  write_cmos_sensor(0x6F12, 0xFBD1);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x6F12, 0x24D0);
  write_cmos_sensor(0x6F12, 0x0846);
  write_cmos_sensor(0x6F12, 0x1EE0);
  write_cmos_sensor(0x6F12, 0x8149);
  write_cmos_sensor(0x6F12, 0x0D8E);
  write_cmos_sensor(0x6F12, 0x496B);
  write_cmos_sensor(0x6F12, 0x4B42);
  write_cmos_sensor(0x6F12, 0x77B1);
  write_cmos_sensor(0x6F12, 0x8148);
  write_cmos_sensor(0x6F12, 0x806F);
  write_cmos_sensor(0x6F12, 0x10E0);
  write_cmos_sensor(0x6F12, 0x4242);
  write_cmos_sensor(0x6F12, 0x00E0);
  write_cmos_sensor(0x6F12, 0x0246);
  write_cmos_sensor(0x6F12, 0x0029);
  write_cmos_sensor(0x6F12, 0x0FDB);
  write_cmos_sensor(0x6F12, 0x8A42);
  write_cmos_sensor(0x6F12, 0x0FDD);
  write_cmos_sensor(0x6F12, 0x3046);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xF041);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x1FB9);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x6F12, 0x0CD0);
  write_cmos_sensor(0x6F12, 0x7848);
  write_cmos_sensor(0x6F12, 0xD0F8);
  write_cmos_sensor(0x6F12, 0x8C00);
  write_cmos_sensor(0x6F12, 0x25B1);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x6F12, 0xEDDA);
  write_cmos_sensor(0x6F12, 0xEAE7);
  write_cmos_sensor(0x6F12, 0x1946);
  write_cmos_sensor(0x6F12, 0xEDE7);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x17F9);
  write_cmos_sensor(0x6F12, 0xE060);
  write_cmos_sensor(0x6F12, 0x0120);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xF081);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xF35F);
  write_cmos_sensor(0x6F12, 0xDFF8);
  write_cmos_sensor(0x6F12, 0xB0A1);
  write_cmos_sensor(0x6F12, 0x0C46);
  write_cmos_sensor(0x6F12, 0xBAF8);
  write_cmos_sensor(0x6F12, 0xBE04);
  write_cmos_sensor(0x6F12, 0x08B1);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x0EF9);
  write_cmos_sensor(0x6F12, 0x6C4E);
  write_cmos_sensor(0x6F12, 0x3088);
  write_cmos_sensor(0x6F12, 0x0128);
  write_cmos_sensor(0x6F12, 0x06D1);
  write_cmos_sensor(0x6F12, 0x002C);
  write_cmos_sensor(0x6F12, 0x04D1);
  write_cmos_sensor(0x6F12, 0x684D);
  write_cmos_sensor(0x6F12, 0x2889);
  write_cmos_sensor(0x6F12, 0x18B1);
  write_cmos_sensor(0x6F12, 0x401E);
  write_cmos_sensor(0x6F12, 0x2881);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xFC9F);
  write_cmos_sensor(0x6F12, 0xDFF8);
  write_cmos_sensor(0x6F12, 0x9891);
  write_cmos_sensor(0x6F12, 0xD9F8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xB0F8);
  write_cmos_sensor(0x6F12, 0xD602);
  write_cmos_sensor(0x6F12, 0x38B1);
  write_cmos_sensor(0x6F12, 0x3089);
  write_cmos_sensor(0x6F12, 0x401C);
  write_cmos_sensor(0x6F12, 0x80B2);
  write_cmos_sensor(0x6F12, 0x3081);
  write_cmos_sensor(0x6F12, 0xFF28);
  write_cmos_sensor(0x6F12, 0x01D9);
  write_cmos_sensor(0x6F12, 0xE889);
  write_cmos_sensor(0x6F12, 0x3081);
  write_cmos_sensor(0x6F12, 0x6048);
  write_cmos_sensor(0x6F12, 0x4FF0);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x6F12, 0xC6F8);
  write_cmos_sensor(0x6F12, 0x0C80);
  write_cmos_sensor(0x6F12, 0xB0F8);
  write_cmos_sensor(0x6F12, 0x5EB0);
  write_cmos_sensor(0x6F12, 0x40F2);
  write_cmos_sensor(0x6F12, 0xFF31);
  write_cmos_sensor(0x6F12, 0x0B20);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xEBF8);
  write_cmos_sensor(0x6F12, 0xD9F8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0027);
  write_cmos_sensor(0x6F12, 0x3C46);
  write_cmos_sensor(0x6F12, 0xB0F8);
  write_cmos_sensor(0x6F12, 0xD412);
  write_cmos_sensor(0x6F12, 0x21B1);
  write_cmos_sensor(0x6F12, 0x0098);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xD2F8);
  write_cmos_sensor(0x6F12, 0x0746);
  write_cmos_sensor(0x6F12, 0x0BE0);
  write_cmos_sensor(0x6F12, 0xB0F8);
  write_cmos_sensor(0x6F12, 0xD602);
  write_cmos_sensor(0x6F12, 0x40B1);
  write_cmos_sensor(0x6F12, 0x3089);
  write_cmos_sensor(0x6F12, 0xE989);
  write_cmos_sensor(0x6F12, 0x8842);
  write_cmos_sensor(0x6F12, 0x04D3);
  write_cmos_sensor(0x6F12, 0x0098);
  write_cmos_sensor(0x6F12, 0xFFF7);
  write_cmos_sensor(0x6F12, 0x6EFF);
  write_cmos_sensor(0x6F12, 0x0746);
  write_cmos_sensor(0x6F12, 0x0124);
  write_cmos_sensor(0x6F12, 0x3846);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xD5F8);
  write_cmos_sensor(0x6F12, 0xD9F8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xB0F8);
  write_cmos_sensor(0x6F12, 0xD602);
  write_cmos_sensor(0x6F12, 0x08B9);
  write_cmos_sensor(0x6F12, 0xA6F8);
  write_cmos_sensor(0x6F12, 0x0280);
  write_cmos_sensor(0x6F12, 0xC7B3);
  write_cmos_sensor(0x6F12, 0x4746);
  write_cmos_sensor(0x6F12, 0xA6F8);
  write_cmos_sensor(0x6F12, 0x0880);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xCDF8);
  write_cmos_sensor(0x6F12, 0xF068);
  write_cmos_sensor(0x6F12, 0x3061);
  write_cmos_sensor(0x6F12, 0x688D);
  write_cmos_sensor(0x6F12, 0x50B3);
  write_cmos_sensor(0x6F12, 0xA88D);
  write_cmos_sensor(0x6F12, 0x50BB);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xCAF8);
  write_cmos_sensor(0x6F12, 0xA889);
  write_cmos_sensor(0x6F12, 0x20B3);
  write_cmos_sensor(0x6F12, 0x1CB3);
  write_cmos_sensor(0x6F12, 0x706B);
  write_cmos_sensor(0x6F12, 0xAA88);
  write_cmos_sensor(0x6F12, 0xDAF8);
  write_cmos_sensor(0x6F12, 0x0815);
  write_cmos_sensor(0x6F12, 0xCAB1);
  write_cmos_sensor(0x6F12, 0x8842);
  write_cmos_sensor(0x6F12, 0x0CDB);
  write_cmos_sensor(0x6F12, 0x90FB);
  write_cmos_sensor(0x6F12, 0xF1F3);
  write_cmos_sensor(0x6F12, 0x90FB);
  write_cmos_sensor(0x6F12, 0xF1F2);
  write_cmos_sensor(0x6F12, 0x01FB);
  write_cmos_sensor(0x6F12, 0x1303);
  write_cmos_sensor(0x6F12, 0xB3EB);
  write_cmos_sensor(0x6F12, 0x610F);
  write_cmos_sensor(0x6F12, 0x00DD);
  write_cmos_sensor(0x6F12, 0x521C);
  write_cmos_sensor(0x6F12, 0x01FB);
  write_cmos_sensor(0x6F12, 0x1200);
  write_cmos_sensor(0x6F12, 0x0BE0);
  write_cmos_sensor(0x6F12, 0x91FB);
  write_cmos_sensor(0x6F12, 0xF0F3);
  write_cmos_sensor(0x6F12, 0x91FB);
  write_cmos_sensor(0x6F12, 0xF0F2);
  write_cmos_sensor(0x6F12, 0x00FB);
  write_cmos_sensor(0x6F12, 0x1313);
  write_cmos_sensor(0x6F12, 0xB3EB);
  write_cmos_sensor(0x6F12, 0x600F);
  write_cmos_sensor(0x6F12, 0x00DD);
  write_cmos_sensor(0x6F12, 0x521C);
  write_cmos_sensor(0x6F12, 0x5043);
  write_cmos_sensor(0x6F12, 0x401A);
  write_cmos_sensor(0x6F12, 0xF168);
  write_cmos_sensor(0x6F12, 0x01EB);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0xF060);
  write_cmos_sensor(0x6F12, 0xA88D);
  write_cmos_sensor(0x6F12, 0x10B1);
  write_cmos_sensor(0x6F12, 0xF089);
  write_cmos_sensor(0x6F12, 0x3087);
  write_cmos_sensor(0x6F12, 0xAF85);
  write_cmos_sensor(0x6F12, 0x5846);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xFC5F);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x9EB8);
  write_cmos_sensor(0x6F12, 0x70B5);
  write_cmos_sensor(0x6F12, 0x2349);
  write_cmos_sensor(0x6F12, 0x0446);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0xC1F8);
  write_cmos_sensor(0x6F12, 0x3005);
  write_cmos_sensor(0x6F12, 0x1E48);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0xC168);
  write_cmos_sensor(0x6F12, 0x0D0C);
  write_cmos_sensor(0x6F12, 0x8EB2);
  write_cmos_sensor(0x6F12, 0x3146);
  write_cmos_sensor(0x6F12, 0x2846);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x53F8);
  write_cmos_sensor(0x6F12, 0x2046);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x91F8);
  write_cmos_sensor(0x6F12, 0x3146);
  write_cmos_sensor(0x6F12, 0x2846);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0x7040);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x49B8);
  write_cmos_sensor(0x6F12, 0x10B5);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0xAFF2);
  write_cmos_sensor(0x6F12, 0x9731);
  write_cmos_sensor(0x6F12, 0x1C48);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x88F8);
  write_cmos_sensor(0x6F12, 0x114C);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0xAFF2);
  write_cmos_sensor(0x6F12, 0x0D31);
  write_cmos_sensor(0x6F12, 0x2060);
  write_cmos_sensor(0x6F12, 0x1948);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x80F8);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0xAFF2);
  write_cmos_sensor(0x6F12, 0xD121);
  write_cmos_sensor(0x6F12, 0x6060);
  write_cmos_sensor(0x6F12, 0x1648);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x79F8);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0xAFF2);
  write_cmos_sensor(0x6F12, 0x1D21);
  write_cmos_sensor(0x6F12, 0xA060);
  write_cmos_sensor(0x6F12, 0x1448);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x72F8);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0xAFF2);
  write_cmos_sensor(0x6F12, 0x9511);
  write_cmos_sensor(0x6F12, 0x1248);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x6CF8);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0xAFF2);
  write_cmos_sensor(0x6F12, 0x7B01);
  write_cmos_sensor(0x6F12, 0x1048);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x66F8);
  write_cmos_sensor(0x6F12, 0xE060);
  write_cmos_sensor(0x6F12, 0x10BD);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x4460);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x2C30);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x2E30);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x2580);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6000);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x2BA0);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x3600);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x0890);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0x7000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x24A7);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x1AF3);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x09BD);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x576B);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x57ED);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xBF8D);
  write_cmos_sensor(0x6F12, 0x4AF6);
  write_cmos_sensor(0x6F12, 0x293C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x42F2);
  write_cmos_sensor(0x6F12, 0xA74C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x41F6);
  write_cmos_sensor(0x6F12, 0xF32C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x010C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x40F6);
  write_cmos_sensor(0x6F12, 0xBD1C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x010C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x4AF6);
  write_cmos_sensor(0x6F12, 0x532C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F2);
  write_cmos_sensor(0x6F12, 0x377C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F2);
  write_cmos_sensor(0x6F12, 0xD56C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F2);
  write_cmos_sensor(0x6F12, 0xC91C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x40F2);
  write_cmos_sensor(0x6F12, 0xAB2C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x44F6);
  write_cmos_sensor(0x6F12, 0x897C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F2);
  write_cmos_sensor(0x6F12, 0xA56C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F2);
  write_cmos_sensor(0x6F12, 0xEF6C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x40F2);
  write_cmos_sensor(0x6F12, 0x6D7C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x4BF6);
  write_cmos_sensor(0x6F12, 0x8D7C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x4BF2);
  write_cmos_sensor(0x6F12, 0xAB4C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x08D1);
  write_cmos_sensor(0x6F12, 0x008B);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0067);

	}	/*	sensor_init  */


static void preview_setting(void)
{
  LOG_INF("E\n");
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x6214, 0x7971);
  write_cmos_sensor(0x6218, 0x7150);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x0FA7);
  write_cmos_sensor(0x034A, 0x0BBF);
  write_cmos_sensor(0x034C, 0x07D0);
  write_cmos_sensor(0x034E, 0x05DC);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0340,0x0680);
  write_cmos_sensor(0x0342,0x25B0);
  write_cmos_sensor(0x0900, 0x0112);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0003);
  write_cmos_sensor(0x0404, 0x2000);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x0306, 0x00F1);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x030E,0x0003);
  write_cmos_sensor(0x0312,0x0002);
  write_cmos_sensor(0x0310,0x0096);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1492);
  write_cmos_sensor(0x6F12, 0x0078);
  write_cmos_sensor(0x602A, 0x0E4E);
  write_cmos_sensor(0x6F12,0x007A);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0118, 0x0004);
  write_cmos_sensor(0x021E, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x2126);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1168);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x602A, 0x2DB6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1668);
  write_cmos_sensor(0x6F12, 0xFF00);
  write_cmos_sensor(0x602A, 0x166A);
  write_cmos_sensor(0x6F12, 0xFF00);
  write_cmos_sensor(0x602A, 0x118A);
  write_cmos_sensor(0x6F12, 0x0802);
  write_cmos_sensor(0x602A, 0x151E);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x217E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1520);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x2522);
  write_cmos_sensor(0x6F12, 0x1004);
  write_cmos_sensor(0x602A, 0x2524);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x2568);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x2588);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x258C);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x25A6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x252C);
  write_cmos_sensor(0x6F12, 0x7801);
  write_cmos_sensor(0x602A, 0x252E);
  write_cmos_sensor(0x6F12, 0x7805);
  write_cmos_sensor(0x602A, 0x25A8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x25AC);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x25B0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x25B4);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x15A4);
  write_cmos_sensor(0x6F12, 0x0641);
  write_cmos_sensor(0x602A, 0x15A6);
  write_cmos_sensor(0x6F12, 0x0145);
  write_cmos_sensor(0x602A, 0x15A8);
  write_cmos_sensor(0x6F12, 0x0149);
  write_cmos_sensor(0x602A, 0x15AA);
  write_cmos_sensor(0x6F12, 0x064D);
  write_cmos_sensor(0x602A, 0x15AC);
  write_cmos_sensor(0x6F12, 0x0651);
  write_cmos_sensor(0x602A, 0x15AE);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x15B0);
  write_cmos_sensor(0x6F12, 0x0159);
  write_cmos_sensor(0x602A, 0x15B2);
  write_cmos_sensor(0x6F12, 0x065D);
  write_cmos_sensor(0x602A, 0x15B4);
  write_cmos_sensor(0x6F12, 0x0661);
  write_cmos_sensor(0x602A, 0x15B6);
  write_cmos_sensor(0x6F12, 0x0165);
  write_cmos_sensor(0x602A, 0x15B8);
  write_cmos_sensor(0x6F12, 0x0169);
  write_cmos_sensor(0x602A, 0x15BA);
  write_cmos_sensor(0x6F12, 0x066D);
  write_cmos_sensor(0x602A, 0x15BC);
  write_cmos_sensor(0x6F12, 0x0671);
  write_cmos_sensor(0x602A, 0x15BE);
  write_cmos_sensor(0x6F12, 0x0175);
  write_cmos_sensor(0x602A, 0x15C0);
  write_cmos_sensor(0x6F12, 0x0179);
  write_cmos_sensor(0x602A, 0x15C2);
  write_cmos_sensor(0x6F12, 0x067D);
  write_cmos_sensor(0x602A, 0x15C4);
  write_cmos_sensor(0x6F12, 0x0641);
  write_cmos_sensor(0x602A, 0x15C6);
  write_cmos_sensor(0x6F12, 0x0145);
  write_cmos_sensor(0x602A, 0x15C8);
  write_cmos_sensor(0x6F12, 0x0149);
  write_cmos_sensor(0x602A, 0x15CA);
  write_cmos_sensor(0x6F12, 0x064D);
  write_cmos_sensor(0x602A, 0x15CC);
  write_cmos_sensor(0x6F12, 0x0651);
  write_cmos_sensor(0x602A, 0x15CE);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x15D0);
  write_cmos_sensor(0x6F12, 0x0159);
  write_cmos_sensor(0x602A, 0x15D2);
  write_cmos_sensor(0x6F12, 0x065D);
  write_cmos_sensor(0x602A, 0x15D4);
  write_cmos_sensor(0x6F12, 0x0661);
  write_cmos_sensor(0x602A, 0x15D6);
  write_cmos_sensor(0x6F12, 0x0165);
  write_cmos_sensor(0x602A, 0x15D8);
  write_cmos_sensor(0x6F12, 0x0169);
  write_cmos_sensor(0x602A, 0x15DA);
  write_cmos_sensor(0x6F12, 0x066D);
  write_cmos_sensor(0x602A, 0x15DC);
  write_cmos_sensor(0x6F12, 0x0671);
  write_cmos_sensor(0x602A, 0x15DE);
  write_cmos_sensor(0x6F12, 0x0175);
  write_cmos_sensor(0x602A, 0x15E0);
  write_cmos_sensor(0x6F12, 0x0179);
  write_cmos_sensor(0x602A, 0x15E2);
  write_cmos_sensor(0x6F12, 0x067D);
  write_cmos_sensor(0x602A, 0x1A50);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x1A54);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0D00,0x0101);//pdaf pre
  write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0x0202, 0x0010);
  write_cmos_sensor(0x0226, 0x0010);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0B06, 0x0101);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x107A);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x1074);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x0E7C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1120);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1122);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x602A, 0x1128);
  write_cmos_sensor(0x6F12, 0x0604);
  write_cmos_sensor(0x602A, 0x1AC0);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1AC2);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x1494);
  write_cmos_sensor(0x6F12, 0x3D68);
  write_cmos_sensor(0x602A, 0x1498);
  write_cmos_sensor(0x6F12, 0xF10D);
  write_cmos_sensor(0x602A, 0x1488);
  write_cmos_sensor(0x6F12,0x0F04);
  write_cmos_sensor(0x602A, 0x148A);
  write_cmos_sensor(0x6F12,0x0F0B);
  write_cmos_sensor(0x602A, 0x150E);
  write_cmos_sensor(0x6F12, 0x00C2);
  write_cmos_sensor(0x602A, 0x1510);
  write_cmos_sensor(0x6F12, 0xC0AF);
  write_cmos_sensor(0x602A, 0x1512);
  write_cmos_sensor(0x6F12, 0x0080);
  write_cmos_sensor(0x602A, 0x1486);
  write_cmos_sensor(0x6F12, 0x1430);
  write_cmos_sensor(0x602A, 0x1490);
  write_cmos_sensor(0x6F12, 0x4D09);
  write_cmos_sensor(0x602A, 0x149E);
  write_cmos_sensor(0x6F12, 0x01C4);
  write_cmos_sensor(0x602A, 0x11CC);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x11CE);
  write_cmos_sensor(0x6F12, 0x000B);
  write_cmos_sensor(0x602A, 0x11D0);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x602A, 0x11DA);
  write_cmos_sensor(0x6F12, 0x0012);
  write_cmos_sensor(0x602A, 0x11E6);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x602A, 0x125E);
  write_cmos_sensor(0x6F12, 0x0048);
  write_cmos_sensor(0x602A, 0x11F4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x11F8);
  write_cmos_sensor(0x6F12, 0x0016);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0xF444, 0x05BF);
  write_cmos_sensor(0xF44A, 0x0008);
  write_cmos_sensor(0xF44E, 0x0012);
  write_cmos_sensor(0xF46E, 0x74C0);
  write_cmos_sensor(0xF470, 0x2809);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1CAA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6000);
  write_cmos_sensor(0x6F12, 0x000F);
  write_cmos_sensor(0x602A, 0x6002);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x602A, 0x6004);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6006);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6008);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6010);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6012);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6014);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6016);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6018);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6020);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6022);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6024);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6026);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6028);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x1144);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1146);
  write_cmos_sensor(0x6F12, 0x1B00);
    
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("start \n");  
 
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x6214, 0x7971);
  write_cmos_sensor(0x6218, 0x7150);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x0FA7);
  write_cmos_sensor(0x034A, 0x0BBF);
  write_cmos_sensor(0x034C, 0x0FA0);
  write_cmos_sensor(0x034E, 0x0BB8);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0340, 0x0C7A);
  write_cmos_sensor(0x0342, 0x13A0);
  write_cmos_sensor(0x0900, 0x0111);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0001);
  write_cmos_sensor(0x0404, 0x1000);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x0306, 0x00F1);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0312, 0x0001);
  write_cmos_sensor(0x0310, 0x0090);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1492);
  write_cmos_sensor(0x6F12, 0x0078);
  write_cmos_sensor(0x602A, 0x0E4E);
  write_cmos_sensor(0x6F12, 0x007A);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0118, 0x0004);
  write_cmos_sensor(0x021E, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x2126);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1168);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x602A, 0x2DB6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1668);
  write_cmos_sensor(0x6F12, 0xF0F0);
  write_cmos_sensor(0x602A, 0x166A);
  write_cmos_sensor(0x6F12, 0xF0F0);
  write_cmos_sensor(0x602A, 0x118A);
  write_cmos_sensor(0x6F12, 0x0802);
  write_cmos_sensor(0x602A, 0x151E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x217E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1520);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x2522);
  write_cmos_sensor(0x6F12, 0x0804);
  write_cmos_sensor(0x602A, 0x2524);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x602A, 0x2568);
  write_cmos_sensor(0x6F12, 0x5500);
  write_cmos_sensor(0x602A, 0x2588);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x258C);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x25A6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x252C);
  write_cmos_sensor(0x6F12, 0x0601);
  write_cmos_sensor(0x602A, 0x252E);
  write_cmos_sensor(0x6F12, 0x0605);
  write_cmos_sensor(0x602A, 0x25A8);
  write_cmos_sensor(0x6F12, 0x1100);
  write_cmos_sensor(0x602A, 0x25AC);
  write_cmos_sensor(0x6F12, 0x0011);
  write_cmos_sensor(0x602A, 0x25B0);
  write_cmos_sensor(0x6F12, 0x1100);
  write_cmos_sensor(0x602A, 0x25B4);
  write_cmos_sensor(0x6F12, 0x0011);
  write_cmos_sensor(0x602A, 0x15A4);
  write_cmos_sensor(0x6F12, 0x0141);
  write_cmos_sensor(0x602A, 0x15A6);
  write_cmos_sensor(0x6F12, 0x0545);
  write_cmos_sensor(0x602A, 0x15A8);
  write_cmos_sensor(0x6F12, 0x0649);
  write_cmos_sensor(0x602A, 0x15AA);
  write_cmos_sensor(0x6F12, 0x024D);
  write_cmos_sensor(0x602A, 0x15AC);
  write_cmos_sensor(0x6F12, 0x0151);
  write_cmos_sensor(0x602A, 0x15AE);
  write_cmos_sensor(0x6F12, 0x0555);
  write_cmos_sensor(0x602A, 0x15B0);
  write_cmos_sensor(0x6F12, 0x0659);
  write_cmos_sensor(0x602A, 0x15B2);
  write_cmos_sensor(0x6F12, 0x025D);
  write_cmos_sensor(0x602A, 0x15B4);
  write_cmos_sensor(0x6F12, 0x0161);
  write_cmos_sensor(0x602A, 0x15B6);
  write_cmos_sensor(0x6F12, 0x0565);
  write_cmos_sensor(0x602A, 0x15B8);
  write_cmos_sensor(0x6F12, 0x0669);
  write_cmos_sensor(0x602A, 0x15BA);
  write_cmos_sensor(0x6F12, 0x026D);
  write_cmos_sensor(0x602A, 0x15BC);
  write_cmos_sensor(0x6F12, 0x0171);
  write_cmos_sensor(0x602A, 0x15BE);
  write_cmos_sensor(0x6F12, 0x0575);
  write_cmos_sensor(0x602A, 0x15C0);
  write_cmos_sensor(0x6F12, 0x0679);
  write_cmos_sensor(0x602A, 0x15C2);
  write_cmos_sensor(0x6F12, 0x027D);
  write_cmos_sensor(0x602A, 0x15C4);
  write_cmos_sensor(0x6F12, 0x0141);
  write_cmos_sensor(0x602A, 0x15C6);
  write_cmos_sensor(0x6F12, 0x0545);
  write_cmos_sensor(0x602A, 0x15C8);
  write_cmos_sensor(0x6F12, 0x0649);
  write_cmos_sensor(0x602A, 0x15CA);
  write_cmos_sensor(0x6F12, 0x024D);
  write_cmos_sensor(0x602A, 0x15CC);
  write_cmos_sensor(0x6F12, 0x0151);
  write_cmos_sensor(0x602A, 0x15CE);
  write_cmos_sensor(0x6F12, 0x0555);
  write_cmos_sensor(0x602A, 0x15D0);
  write_cmos_sensor(0x6F12, 0x0659);
  write_cmos_sensor(0x602A, 0x15D2);
  write_cmos_sensor(0x6F12, 0x025D);
  write_cmos_sensor(0x602A, 0x15D4);
  write_cmos_sensor(0x6F12, 0x0161);
  write_cmos_sensor(0x602A, 0x15D6);
  write_cmos_sensor(0x6F12, 0x0565);
  write_cmos_sensor(0x602A, 0x15D8);
  write_cmos_sensor(0x6F12, 0x0669);
  write_cmos_sensor(0x602A, 0x15DA);
  write_cmos_sensor(0x6F12, 0x026D);
  write_cmos_sensor(0x602A, 0x15DC);
  write_cmos_sensor(0x6F12, 0x0171);
  write_cmos_sensor(0x602A, 0x15DE);
  write_cmos_sensor(0x6F12, 0x0575);
  write_cmos_sensor(0x602A, 0x15E0);
  write_cmos_sensor(0x6F12, 0x0679);
  write_cmos_sensor(0x602A, 0x15E2);
  write_cmos_sensor(0x6F12, 0x027D);
  write_cmos_sensor(0x602A, 0x1A50);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1A54);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0D00, 0x0100);//0100cap
  write_cmos_sensor(0x0D02, 0x0001);//pdaf
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0xF486, 0x0000);
  write_cmos_sensor(0xF488, 0x0000);
  write_cmos_sensor(0xF48A, 0x0000);
  write_cmos_sensor(0xF48C, 0x0000);
  write_cmos_sensor(0xF48E, 0x0000);
  write_cmos_sensor(0xF490, 0x0000);
  write_cmos_sensor(0xF492, 0x0000);
  write_cmos_sensor(0xF494, 0x0000);
  write_cmos_sensor(0xF496, 0x0000);
  write_cmos_sensor(0xF498, 0x0000);
  write_cmos_sensor(0xF49A, 0x0000);
  write_cmos_sensor(0xF49C, 0x0000);
  write_cmos_sensor(0xF49E, 0x0000);
  write_cmos_sensor(0xF4A0, 0x0000);
  write_cmos_sensor(0xF4A2, 0x0000);
  write_cmos_sensor(0xF4A4, 0x0000);
  write_cmos_sensor(0xF4A6, 0x0000);
  write_cmos_sensor(0xF4A8, 0x0000);
  write_cmos_sensor(0xF4AA, 0x0000);
  write_cmos_sensor(0xF4AC, 0x0000);
  write_cmos_sensor(0xF4AE, 0x0000);
  write_cmos_sensor(0xF4B0, 0x0000);
  write_cmos_sensor(0xF4B2, 0x0000);
  write_cmos_sensor(0xF4B4, 0x0000);
  write_cmos_sensor(0xF4B6, 0x0000);
  write_cmos_sensor(0xF4B8, 0x0000);
  write_cmos_sensor(0xF4BA, 0x0000);
  write_cmos_sensor(0xF4BC, 0x0000);
  write_cmos_sensor(0xF4BE, 0x0000);
  write_cmos_sensor(0xF4C0, 0x0000);
  write_cmos_sensor(0xF4C2, 0x0000);
  write_cmos_sensor(0xF4C4, 0x0000);
  write_cmos_sensor(0x0202, 0x0010);
  write_cmos_sensor(0x0226, 0x0010);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0B06, 0x0101);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x107A);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x1074);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x0E7C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1120);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1122);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x602A, 0x1128);
  write_cmos_sensor(0x6F12, 0x0604);
  write_cmos_sensor(0x602A, 0x1AC0);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1AC2);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x1494);
  write_cmos_sensor(0x6F12, 0x3D68);
  write_cmos_sensor(0x602A, 0x1498);
  write_cmos_sensor(0x6F12, 0xF10D);
  write_cmos_sensor(0x602A, 0x1488);
  write_cmos_sensor(0x6F12, 0x0F04);
  write_cmos_sensor(0x602A, 0x148A);
  write_cmos_sensor(0x6F12, 0x170B);
  write_cmos_sensor(0x602A, 0x150E);
  write_cmos_sensor(0x6F12, 0x00C2);
  write_cmos_sensor(0x602A, 0x1510);
  write_cmos_sensor(0x6F12, 0xC0AF);
  write_cmos_sensor(0x602A, 0x1512);
  write_cmos_sensor(0x6F12, 0x00A0);
  write_cmos_sensor(0x602A, 0x1486);
  write_cmos_sensor(0x6F12, 0x1430);
  write_cmos_sensor(0x602A, 0x1490);
  write_cmos_sensor(0x6F12, 0x4D09);
  write_cmos_sensor(0x602A, 0x149E);
  write_cmos_sensor(0x6F12, 0x01C4);
  write_cmos_sensor(0x602A, 0x11CC);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x11CE);
  write_cmos_sensor(0x6F12, 0x000B);
  write_cmos_sensor(0x602A, 0x11D0);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x602A, 0x11DA);
  write_cmos_sensor(0x6F12, 0x0012);
  write_cmos_sensor(0x602A, 0x11E6);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x602A, 0x125E);
  write_cmos_sensor(0x6F12, 0x0048);
  write_cmos_sensor(0x602A, 0x11F4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x11F8);
  write_cmos_sensor(0x6F12, 0x0016);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0xF444, 0x05BF);
  write_cmos_sensor(0xF44A, 0x0008);
  write_cmos_sensor(0xF44E, 0x0012);
  write_cmos_sensor(0xF46E, 0x40C0);
  write_cmos_sensor(0xF470, 0x7809);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1CAA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6000);
  write_cmos_sensor(0x6F12, 0x000F);
  write_cmos_sensor(0x602A, 0x6002);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x602A, 0x6004);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6006);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6008);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6010);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6012);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6014);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6016);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6018);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6020);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6022);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6024);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6026);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6028);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x1144);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1146);
  write_cmos_sensor(0x6F12, 0x1B00);
  write_cmos_sensor(0x602A, 0x1080);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1084);
  write_cmos_sensor(0x6F12, 0x00C0);
  write_cmos_sensor(0x602A, 0x108A);
  write_cmos_sensor(0x6F12, 0x00C0);
  write_cmos_sensor(0x602A, 0x1090);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1092);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1094);
  write_cmos_sensor(0x6F12, 0xA32E);

  
 } /* capture setting */
 
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("start \n"); 
  LOG_INF("E\n");
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x6214, 0x7971);
  write_cmos_sensor(0x6218, 0x7150);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x0FA7);
  write_cmos_sensor(0x034A, 0x0BBF);
  write_cmos_sensor(0x034C, 0x07D0);
  write_cmos_sensor(0x034E, 0x05DC);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0340,0x0680);
  write_cmos_sensor(0x0342,0x25B0);
  write_cmos_sensor(0x0900, 0x0112);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0003);
  write_cmos_sensor(0x0404, 0x2000);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x0306, 0x00F1);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x030E,0x0003);
  write_cmos_sensor(0x0312,0x0002);
  write_cmos_sensor(0x0310,0x0096);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1492);
  write_cmos_sensor(0x6F12, 0x0078);
  write_cmos_sensor(0x602A, 0x0E4E);
  write_cmos_sensor(0x6F12,0x007A);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0118, 0x0004);
  write_cmos_sensor(0x021E, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x2126);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1168);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x602A, 0x2DB6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1668);
  write_cmos_sensor(0x6F12, 0xFF00);
  write_cmos_sensor(0x602A, 0x166A);
  write_cmos_sensor(0x6F12, 0xFF00);
  write_cmos_sensor(0x602A, 0x118A);
  write_cmos_sensor(0x6F12, 0x0802);
  write_cmos_sensor(0x602A, 0x151E);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x217E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1520);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x2522);
  write_cmos_sensor(0x6F12, 0x1004);
  write_cmos_sensor(0x602A, 0x2524);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x2568);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x2588);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x258C);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x25A6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x252C);
  write_cmos_sensor(0x6F12, 0x7801);
  write_cmos_sensor(0x602A, 0x252E);
  write_cmos_sensor(0x6F12, 0x7805);
  write_cmos_sensor(0x602A, 0x25A8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x25AC);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x25B0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x25B4);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x15A4);
  write_cmos_sensor(0x6F12, 0x0641);
  write_cmos_sensor(0x602A, 0x15A6);
  write_cmos_sensor(0x6F12, 0x0145);
  write_cmos_sensor(0x602A, 0x15A8);
  write_cmos_sensor(0x6F12, 0x0149);
  write_cmos_sensor(0x602A, 0x15AA);
  write_cmos_sensor(0x6F12, 0x064D);
  write_cmos_sensor(0x602A, 0x15AC);
  write_cmos_sensor(0x6F12, 0x0651);
  write_cmos_sensor(0x602A, 0x15AE);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x15B0);
  write_cmos_sensor(0x6F12, 0x0159);
  write_cmos_sensor(0x602A, 0x15B2);
  write_cmos_sensor(0x6F12, 0x065D);
  write_cmos_sensor(0x602A, 0x15B4);
  write_cmos_sensor(0x6F12, 0x0661);
  write_cmos_sensor(0x602A, 0x15B6);
  write_cmos_sensor(0x6F12, 0x0165);
  write_cmos_sensor(0x602A, 0x15B8);
  write_cmos_sensor(0x6F12, 0x0169);
  write_cmos_sensor(0x602A, 0x15BA);
  write_cmos_sensor(0x6F12, 0x066D);
  write_cmos_sensor(0x602A, 0x15BC);
  write_cmos_sensor(0x6F12, 0x0671);
  write_cmos_sensor(0x602A, 0x15BE);
  write_cmos_sensor(0x6F12, 0x0175);
  write_cmos_sensor(0x602A, 0x15C0);
  write_cmos_sensor(0x6F12, 0x0179);
  write_cmos_sensor(0x602A, 0x15C2);
  write_cmos_sensor(0x6F12, 0x067D);
  write_cmos_sensor(0x602A, 0x15C4);
  write_cmos_sensor(0x6F12, 0x0641);
  write_cmos_sensor(0x602A, 0x15C6);
  write_cmos_sensor(0x6F12, 0x0145);
  write_cmos_sensor(0x602A, 0x15C8);
  write_cmos_sensor(0x6F12, 0x0149);
  write_cmos_sensor(0x602A, 0x15CA);
  write_cmos_sensor(0x6F12, 0x064D);
  write_cmos_sensor(0x602A, 0x15CC);
  write_cmos_sensor(0x6F12, 0x0651);
  write_cmos_sensor(0x602A, 0x15CE);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x15D0);
  write_cmos_sensor(0x6F12, 0x0159);
  write_cmos_sensor(0x602A, 0x15D2);
  write_cmos_sensor(0x6F12, 0x065D);
  write_cmos_sensor(0x602A, 0x15D4);
  write_cmos_sensor(0x6F12, 0x0661);
  write_cmos_sensor(0x602A, 0x15D6);
  write_cmos_sensor(0x6F12, 0x0165);
  write_cmos_sensor(0x602A, 0x15D8);
  write_cmos_sensor(0x6F12, 0x0169);
  write_cmos_sensor(0x602A, 0x15DA);
  write_cmos_sensor(0x6F12, 0x066D);
  write_cmos_sensor(0x602A, 0x15DC);
  write_cmos_sensor(0x6F12, 0x0671);
  write_cmos_sensor(0x602A, 0x15DE);
  write_cmos_sensor(0x6F12, 0x0175);
  write_cmos_sensor(0x602A, 0x15E0);
  write_cmos_sensor(0x6F12, 0x0179);
  write_cmos_sensor(0x602A, 0x15E2);
  write_cmos_sensor(0x6F12, 0x067D);
  write_cmos_sensor(0x602A, 0x1A50);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x1A54);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0D00,0x0101);//pdaf pre
  write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0x0202, 0x0010);
  write_cmos_sensor(0x0226, 0x0010);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0B06, 0x0101);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x107A);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x1074);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x0E7C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1120);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1122);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x602A, 0x1128);
  write_cmos_sensor(0x6F12, 0x0604);
  write_cmos_sensor(0x602A, 0x1AC0);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1AC2);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x1494);
  write_cmos_sensor(0x6F12, 0x3D68);
  write_cmos_sensor(0x602A, 0x1498);
  write_cmos_sensor(0x6F12, 0xF10D);
  write_cmos_sensor(0x602A, 0x1488);
  write_cmos_sensor(0x6F12,0x0F04);
  write_cmos_sensor(0x602A, 0x148A);
  write_cmos_sensor(0x6F12,0x0F0B);
  write_cmos_sensor(0x602A, 0x150E);
  write_cmos_sensor(0x6F12, 0x00C2);
  write_cmos_sensor(0x602A, 0x1510);
  write_cmos_sensor(0x6F12, 0xC0AF);
  write_cmos_sensor(0x602A, 0x1512);
  write_cmos_sensor(0x6F12, 0x0080);
  write_cmos_sensor(0x602A, 0x1486);
  write_cmos_sensor(0x6F12, 0x1430);
  write_cmos_sensor(0x602A, 0x1490);
  write_cmos_sensor(0x6F12, 0x4D09);
  write_cmos_sensor(0x602A, 0x149E);
  write_cmos_sensor(0x6F12, 0x01C4);
  write_cmos_sensor(0x602A, 0x11CC);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x11CE);
  write_cmos_sensor(0x6F12, 0x000B);
  write_cmos_sensor(0x602A, 0x11D0);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x602A, 0x11DA);
  write_cmos_sensor(0x6F12, 0x0012);
  write_cmos_sensor(0x602A, 0x11E6);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x602A, 0x125E);
  write_cmos_sensor(0x6F12, 0x0048);
  write_cmos_sensor(0x602A, 0x11F4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x11F8);
  write_cmos_sensor(0x6F12, 0x0016);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0xF444, 0x05BF);
  write_cmos_sensor(0xF44A, 0x0008);
  write_cmos_sensor(0xF44E, 0x0012);
  write_cmos_sensor(0xF46E, 0x74C0);
  write_cmos_sensor(0xF470, 0x2809);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1CAA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6000);
  write_cmos_sensor(0x6F12, 0x000F);
  write_cmos_sensor(0x602A, 0x6002);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x602A, 0x6004);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6006);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6008);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6010);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6012);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6014);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6016);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6018);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6020);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6022);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6024);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6026);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6028);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x1144);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1146);
  write_cmos_sensor(0x6F12, 0x1B00);
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x6214, 0x7971);
  write_cmos_sensor(0x6218, 0x7150);
  write_cmos_sensor(0x0344, 0x0058);
  write_cmos_sensor(0x0346, 0x01AC);
  write_cmos_sensor(0x0348, 0x0F57);
  write_cmos_sensor(0x034A, 0x0A1B);
  write_cmos_sensor(0x034C, 0x0500);
  write_cmos_sensor(0x034E, 0x02D0);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0340, 0x0330);
  write_cmos_sensor(0x0342, 0x13A0);
  write_cmos_sensor(0x0900, 0x0123);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0002);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0005);
  write_cmos_sensor(0x0404, 0x1000);
  write_cmos_sensor(0x0402, 0x1810);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x0306, 0x00F6);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0312, 0x0002);
  write_cmos_sensor(0x0310, 0x005B);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1492);
  write_cmos_sensor(0x6F12, 0x0078);
  write_cmos_sensor(0x602A, 0x0E4E);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0118, 0x0104);
  write_cmos_sensor(0x021E, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x2126);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1168);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x602A, 0x2DB6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1668);
  write_cmos_sensor(0x6F12, 0xF0F0);
  write_cmos_sensor(0x602A, 0x166A);
  write_cmos_sensor(0x6F12, 0xF0F0);
  write_cmos_sensor(0x602A, 0x118A);
  write_cmos_sensor(0x6F12, 0x0802);
  write_cmos_sensor(0x602A, 0x151E);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x217E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1520);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x2522);
  write_cmos_sensor(0x6F12, 0x0804);
  write_cmos_sensor(0x602A, 0x2524);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x602A, 0x2568);
  write_cmos_sensor(0x6F12, 0x5500);
  write_cmos_sensor(0x602A, 0x2588);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x258C);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x25A6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x252C);
  write_cmos_sensor(0x6F12, 0x0601);
  write_cmos_sensor(0x602A, 0x252E);
  write_cmos_sensor(0x6F12, 0x0605);
  write_cmos_sensor(0x602A, 0x25A8);
  write_cmos_sensor(0x6F12, 0x1100);
  write_cmos_sensor(0x602A, 0x25AC);
  write_cmos_sensor(0x6F12, 0x0011);
  write_cmos_sensor(0x602A, 0x25B0);
  write_cmos_sensor(0x6F12, 0x1100);
  write_cmos_sensor(0x602A, 0x25B4);
  write_cmos_sensor(0x6F12, 0x0011);
  write_cmos_sensor(0x602A, 0x15A4);
  write_cmos_sensor(0x6F12, 0x0141);
  write_cmos_sensor(0x602A, 0x15A6);
  write_cmos_sensor(0x6F12, 0x0545);
  write_cmos_sensor(0x602A, 0x15A8);
  write_cmos_sensor(0x6F12, 0x0649);
  write_cmos_sensor(0x602A, 0x15AA);
  write_cmos_sensor(0x6F12, 0x024D);
  write_cmos_sensor(0x602A, 0x15AC);
  write_cmos_sensor(0x6F12, 0x0151);
  write_cmos_sensor(0x602A, 0x15AE);
  write_cmos_sensor(0x6F12, 0x0555);
  write_cmos_sensor(0x602A, 0x15B0);
  write_cmos_sensor(0x6F12, 0x0659);
  write_cmos_sensor(0x602A, 0x15B2);
  write_cmos_sensor(0x6F12, 0x025D);
  write_cmos_sensor(0x602A, 0x15B4);
  write_cmos_sensor(0x6F12, 0x0161);
  write_cmos_sensor(0x602A, 0x15B6);
  write_cmos_sensor(0x6F12, 0x0565);
  write_cmos_sensor(0x602A, 0x15B8);
  write_cmos_sensor(0x6F12, 0x0669);
  write_cmos_sensor(0x602A, 0x15BA);
  write_cmos_sensor(0x6F12, 0x026D);
  write_cmos_sensor(0x602A, 0x15BC);
  write_cmos_sensor(0x6F12, 0x0171);
  write_cmos_sensor(0x602A, 0x15BE);
  write_cmos_sensor(0x6F12, 0x0575);
  write_cmos_sensor(0x602A, 0x15C0);
  write_cmos_sensor(0x6F12, 0x0679);
  write_cmos_sensor(0x602A, 0x15C2);
  write_cmos_sensor(0x6F12, 0x027D);
  write_cmos_sensor(0x602A, 0x15C4);
  write_cmos_sensor(0x6F12, 0x0141);
  write_cmos_sensor(0x602A, 0x15C6);
  write_cmos_sensor(0x6F12, 0x0545);
  write_cmos_sensor(0x602A, 0x15C8);
  write_cmos_sensor(0x6F12, 0x0649);
  write_cmos_sensor(0x602A, 0x15CA);
  write_cmos_sensor(0x6F12, 0x024D);
  write_cmos_sensor(0x602A, 0x15CC);
  write_cmos_sensor(0x6F12, 0x0151);
  write_cmos_sensor(0x602A, 0x15CE);
  write_cmos_sensor(0x6F12, 0x0555);
  write_cmos_sensor(0x602A, 0x15D0);
  write_cmos_sensor(0x6F12, 0x0659);
  write_cmos_sensor(0x602A, 0x15D2);
  write_cmos_sensor(0x6F12, 0x025D);
  write_cmos_sensor(0x602A, 0x15D4);
  write_cmos_sensor(0x6F12, 0x0161);
  write_cmos_sensor(0x602A, 0x15D6);
  write_cmos_sensor(0x6F12, 0x0565);
  write_cmos_sensor(0x602A, 0x15D8);
  write_cmos_sensor(0x6F12, 0x0669);
  write_cmos_sensor(0x602A, 0x15DA);
  write_cmos_sensor(0x6F12, 0x026D);
  write_cmos_sensor(0x602A, 0x15DC);
  write_cmos_sensor(0x6F12, 0x0171);
  write_cmos_sensor(0x602A, 0x15DE);
  write_cmos_sensor(0x6F12, 0x0575);
  write_cmos_sensor(0x602A, 0x15E0);
  write_cmos_sensor(0x6F12, 0x0679);
  write_cmos_sensor(0x602A, 0x15E2);
  write_cmos_sensor(0x6F12, 0x027D);
  write_cmos_sensor(0x602A, 0x1A50);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1A54);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0D00, 0x0101);//pdaf  hs
  write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0xF486, 0x0000);
  write_cmos_sensor(0xF488, 0x0000);
  write_cmos_sensor(0xF48A, 0x0000);
  write_cmos_sensor(0xF48C, 0x0000);
  write_cmos_sensor(0xF48E, 0x0000);
  write_cmos_sensor(0xF490, 0x0000);
  write_cmos_sensor(0xF492, 0x0000);
  write_cmos_sensor(0xF494, 0x0000);
  write_cmos_sensor(0xF496, 0x0000);
  write_cmos_sensor(0xF498, 0x0000);
  write_cmos_sensor(0xF49A, 0x0000);
  write_cmos_sensor(0xF49C, 0x0000);
  write_cmos_sensor(0xF49E, 0x0000);
  write_cmos_sensor(0xF4A0, 0x0000);
  write_cmos_sensor(0xF4A2, 0x0000);
  write_cmos_sensor(0xF4A4, 0x0000);
  write_cmos_sensor(0xF4A6, 0x0000);
  write_cmos_sensor(0xF4A8, 0x0000);
  write_cmos_sensor(0xF4AA, 0x0000);
  write_cmos_sensor(0xF4AC, 0x0000);
  write_cmos_sensor(0xF4AE, 0x0000);
  write_cmos_sensor(0xF4B0, 0x0000);
  write_cmos_sensor(0xF4B2, 0x0000);
  write_cmos_sensor(0xF4B4, 0x0000);
  write_cmos_sensor(0xF4B6, 0x0000);
  write_cmos_sensor(0xF4B8, 0x0000);
  write_cmos_sensor(0xF4BA, 0x0000);
  write_cmos_sensor(0xF4BC, 0x0000);
  write_cmos_sensor(0xF4BE, 0x0000);
  write_cmos_sensor(0xF4C0, 0x0000);
  write_cmos_sensor(0xF4C2, 0x0000);
  write_cmos_sensor(0xF4C4, 0x0000);
  write_cmos_sensor(0x0202, 0x0010);
  write_cmos_sensor(0x0226, 0x0010);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0B06, 0x0101);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x107A);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x1074);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x0E7C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1120);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1122);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x602A, 0x1128);
  write_cmos_sensor(0x6F12, 0x0604);
  write_cmos_sensor(0x602A, 0x1AC0);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1AC2);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x1494);
  write_cmos_sensor(0x6F12, 0x3D68);
  write_cmos_sensor(0x602A, 0x1498);
  write_cmos_sensor(0x6F12, 0xF10D);
  write_cmos_sensor(0x602A, 0x1488);
  write_cmos_sensor(0x6F12, 0x0F0F);
  write_cmos_sensor(0x602A, 0x148A);
  write_cmos_sensor(0x6F12, 0x170F);
  write_cmos_sensor(0x602A, 0x150E);
  write_cmos_sensor(0x6F12, 0x00C2);
  write_cmos_sensor(0x602A, 0x1510);
  write_cmos_sensor(0x6F12, 0xC0AF);
  write_cmos_sensor(0x602A, 0x1512);
  write_cmos_sensor(0x6F12, 0x0080);
  write_cmos_sensor(0x602A, 0x1486);
  write_cmos_sensor(0x6F12, 0x1430);
  write_cmos_sensor(0x602A, 0x1490);
  write_cmos_sensor(0x6F12, 0x4D09);
  write_cmos_sensor(0x602A, 0x149E);
  write_cmos_sensor(0x6F12, 0x01C4);
  write_cmos_sensor(0x602A, 0x11CC);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x11CE);
  write_cmos_sensor(0x6F12, 0x000B);
  write_cmos_sensor(0x602A, 0x11D0);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x602A, 0x11DA);
  write_cmos_sensor(0x6F12, 0x0012);
  write_cmos_sensor(0x602A, 0x11E6);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x602A, 0x125E);
  write_cmos_sensor(0x6F12, 0x0048);
  write_cmos_sensor(0x602A, 0x11F4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x11F8);
  write_cmos_sensor(0x6F12, 0x0016);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0xF444, 0x05BF);
  write_cmos_sensor(0xF44A, 0x0008);
  write_cmos_sensor(0xF44E, 0x0012);
  write_cmos_sensor(0xF46E, 0x6CC0);
  write_cmos_sensor(0xF470, 0x7809);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1CAA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6000);
  write_cmos_sensor(0x6F12, 0x000F);
  write_cmos_sensor(0x602A, 0x6002);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x602A, 0x6004);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6006);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6008);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6010);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6012);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6014);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6016);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6018);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6020);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6022);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6024);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6026);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6028);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x1144);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1146);
  write_cmos_sensor(0x6F12, 0x1B00);
  write_cmos_sensor(0x602A, 0x1080);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1084);
  write_cmos_sensor(0x6F12, 0x00C0);
  write_cmos_sensor(0x602A, 0x108A);
  write_cmos_sensor(0x6F12, 0x00C0);
  write_cmos_sensor(0x602A, 0x1090);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1092);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1094);
  write_cmos_sensor(0x6F12, 0xA32E);
   
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor(0x0100, 0x0000); 
  mdelay(30);                        
  write_cmos_sensor(0x0344,	0x0340);
  write_cmos_sensor(0x0346,	0x0260);
  write_cmos_sensor(0x0348,	0x0D3F);
  write_cmos_sensor(0x034A,	0x09DF);
  write_cmos_sensor(0x034C,	0x0280);
  write_cmos_sensor(0x034E,	0x01E0);
  write_cmos_sensor(0x0900,	0x0144);
  write_cmos_sensor(0x0380,	0x0001);
  write_cmos_sensor(0x0382,	0x0001);
  write_cmos_sensor(0x0384,	0x0001);
  write_cmos_sensor(0x0386,	0x0007);
  write_cmos_sensor(0x0114,	0x0330);
  write_cmos_sensor(0x0110,	0x0002);
  write_cmos_sensor(0x0136,	0x1800);
  write_cmos_sensor(0x0304,	0x0004);
  write_cmos_sensor(0x0306,	0x0078);
  write_cmos_sensor(0x3C1E,	0x0000);
  write_cmos_sensor(0x030C,	0x0003);
  write_cmos_sensor(0x030E,	0x005D);
  write_cmos_sensor(0x3C16,	0x0003);
  write_cmos_sensor(0x0300,	0x0006);
  write_cmos_sensor(0x0342,	0x1320);
  write_cmos_sensor(0x0340,	0x0330);
  write_cmos_sensor(0x38C4,	0x0006);
  write_cmos_sensor(0x38D8,	0x0003);
  write_cmos_sensor(0x38DA,	0x0003);
  write_cmos_sensor(0x38DC,	0x0017);
  write_cmos_sensor(0x38C2,	0x0008);
  write_cmos_sensor(0x38C0,	0x0000);
  write_cmos_sensor(0x38D6,	0x0013);
  write_cmos_sensor(0x38D4,	0x0005);
  write_cmos_sensor(0x38B0,	0x0002);
  write_cmos_sensor(0x3932,	0x1800);
  write_cmos_sensor(0x3938,	0x200C);
  write_cmos_sensor(0x0820,	0x00BA);
  write_cmos_sensor(0x380C,	0x0023);
  write_cmos_sensor(0x3064,	0xEBCF);
  write_cmos_sensor(0x309C,	0x0600);
  write_cmos_sensor(0x3090,	0x8000);
  write_cmos_sensor(0x3238,	0x000A);
  write_cmos_sensor(0x314A,	0x5F00);
  write_cmos_sensor(0x3300,	0x0000);
  write_cmos_sensor(0x3400,	0x0000);
  write_cmos_sensor(0x3402,	0x4E46);
  write_cmos_sensor(0x32B2,	0x000A);
  write_cmos_sensor(0x32B4,	0x000A);
  write_cmos_sensor(0x32B6,	0x000A);
  write_cmos_sensor(0x32B8,	0x000A);
  write_cmos_sensor(0x3C34,	0x0048);
  write_cmos_sensor(0x3C36,	0x4000);
  write_cmos_sensor(0x3C38,	0x0020);
  write_cmos_sensor(0x393E,	0x4000);
  write_cmos_sensor(0x3C1E, 0x0000); 
}

static void custom1_setting(void)
{
	LOG_INF("start \n");  
 
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x6214, 0x7971);
  write_cmos_sensor(0x6218, 0x7150);
  write_cmos_sensor(0x0344, 0x0088);
  write_cmos_sensor(0x0346, 0x0068);
  write_cmos_sensor(0x0348, 0x0F27);
  write_cmos_sensor(0x034A, 0x0B5F);
  write_cmos_sensor(0x034C, 0x0EA0);
  write_cmos_sensor(0x034E, 0x0AF8);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0340, 0x0C7A);
  write_cmos_sensor(0x0342, 0x13A0);
  write_cmos_sensor(0x0900, 0x0111);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0001);
  write_cmos_sensor(0x0404, 0x1000);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x0306, 0x00F1);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0312, 0x0001);
  write_cmos_sensor(0x0310, 0x0090);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1492);
  write_cmos_sensor(0x6F12, 0x0078);
  write_cmos_sensor(0x602A, 0x0E4E);
  write_cmos_sensor(0x6F12, 0x007A);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0118, 0x0004);
  write_cmos_sensor(0x021E, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x2126);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1168);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x602A, 0x2DB6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1668);
  write_cmos_sensor(0x6F12, 0xF0F0);
  write_cmos_sensor(0x602A, 0x166A);
  write_cmos_sensor(0x6F12, 0xF0F0);
  write_cmos_sensor(0x602A, 0x118A);
  write_cmos_sensor(0x6F12, 0x0802);
  write_cmos_sensor(0x602A, 0x151E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x217E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1520);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x2522);
  write_cmos_sensor(0x6F12, 0x0804);
  write_cmos_sensor(0x602A, 0x2524);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x602A, 0x2568);
  write_cmos_sensor(0x6F12, 0x5500);
  write_cmos_sensor(0x602A, 0x2588);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x258C);
  write_cmos_sensor(0x6F12, 0x1111);
  write_cmos_sensor(0x602A, 0x25A6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x252C);
  write_cmos_sensor(0x6F12, 0x0601);
  write_cmos_sensor(0x602A, 0x252E);
  write_cmos_sensor(0x6F12, 0x0605);
  write_cmos_sensor(0x602A, 0x25A8);
  write_cmos_sensor(0x6F12, 0x1100);
  write_cmos_sensor(0x602A, 0x25AC);
  write_cmos_sensor(0x6F12, 0x0011);
  write_cmos_sensor(0x602A, 0x25B0);
  write_cmos_sensor(0x6F12, 0x1100);
  write_cmos_sensor(0x602A, 0x25B4);
  write_cmos_sensor(0x6F12, 0x0011);
  write_cmos_sensor(0x602A, 0x15A4);
  write_cmos_sensor(0x6F12, 0x0141);
  write_cmos_sensor(0x602A, 0x15A6);
  write_cmos_sensor(0x6F12, 0x0545);
  write_cmos_sensor(0x602A, 0x15A8);
  write_cmos_sensor(0x6F12, 0x0649);
  write_cmos_sensor(0x602A, 0x15AA);
  write_cmos_sensor(0x6F12, 0x024D);
  write_cmos_sensor(0x602A, 0x15AC);
  write_cmos_sensor(0x6F12, 0x0151);
  write_cmos_sensor(0x602A, 0x15AE);
  write_cmos_sensor(0x6F12, 0x0555);
  write_cmos_sensor(0x602A, 0x15B0);
  write_cmos_sensor(0x6F12, 0x0659);
  write_cmos_sensor(0x602A, 0x15B2);
  write_cmos_sensor(0x6F12, 0x025D);
  write_cmos_sensor(0x602A, 0x15B4);
  write_cmos_sensor(0x6F12, 0x0161);
  write_cmos_sensor(0x602A, 0x15B6);
  write_cmos_sensor(0x6F12, 0x0565);
  write_cmos_sensor(0x602A, 0x15B8);
  write_cmos_sensor(0x6F12, 0x0669);
  write_cmos_sensor(0x602A, 0x15BA);
  write_cmos_sensor(0x6F12, 0x026D);
  write_cmos_sensor(0x602A, 0x15BC);
  write_cmos_sensor(0x6F12, 0x0171);
  write_cmos_sensor(0x602A, 0x15BE);
  write_cmos_sensor(0x6F12, 0x0575);
  write_cmos_sensor(0x602A, 0x15C0);
  write_cmos_sensor(0x6F12, 0x0679);
  write_cmos_sensor(0x602A, 0x15C2);
  write_cmos_sensor(0x6F12, 0x027D);
  write_cmos_sensor(0x602A, 0x15C4);
  write_cmos_sensor(0x6F12, 0x0141);
  write_cmos_sensor(0x602A, 0x15C6);
  write_cmos_sensor(0x6F12, 0x0545);
  write_cmos_sensor(0x602A, 0x15C8);
  write_cmos_sensor(0x6F12, 0x0649);
  write_cmos_sensor(0x602A, 0x15CA);
  write_cmos_sensor(0x6F12, 0x024D);
  write_cmos_sensor(0x602A, 0x15CC);
  write_cmos_sensor(0x6F12, 0x0151);
  write_cmos_sensor(0x602A, 0x15CE);
  write_cmos_sensor(0x6F12, 0x0555);
  write_cmos_sensor(0x602A, 0x15D0);
  write_cmos_sensor(0x6F12, 0x0659);
  write_cmos_sensor(0x602A, 0x15D2);
  write_cmos_sensor(0x6F12, 0x025D);
  write_cmos_sensor(0x602A, 0x15D4);
  write_cmos_sensor(0x6F12, 0x0161);
  write_cmos_sensor(0x602A, 0x15D6);
  write_cmos_sensor(0x6F12, 0x0565);
  write_cmos_sensor(0x602A, 0x15D8);
  write_cmos_sensor(0x6F12, 0x0669);
  write_cmos_sensor(0x602A, 0x15DA);
  write_cmos_sensor(0x6F12, 0x026D);
  write_cmos_sensor(0x602A, 0x15DC);
  write_cmos_sensor(0x6F12, 0x0171);
  write_cmos_sensor(0x602A, 0x15DE);
  write_cmos_sensor(0x6F12, 0x0575);
  write_cmos_sensor(0x602A, 0x15E0);
  write_cmos_sensor(0x6F12, 0x0679);
  write_cmos_sensor(0x602A, 0x15E2);
  write_cmos_sensor(0x6F12, 0x027D);
  write_cmos_sensor(0x602A, 0x1A50);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1A54);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0D00, 0x0101);//0100  custom
  write_cmos_sensor(0x0D02, 0x0001);//pdaf
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0xF486, 0x0000);
  write_cmos_sensor(0xF488, 0x0000);
  write_cmos_sensor(0xF48A, 0x0000);
  write_cmos_sensor(0xF48C, 0x0000);
  write_cmos_sensor(0xF48E, 0x0000);
  write_cmos_sensor(0xF490, 0x0000);
  write_cmos_sensor(0xF492, 0x0000);
  write_cmos_sensor(0xF494, 0x0000);
  write_cmos_sensor(0xF496, 0x0000);
  write_cmos_sensor(0xF498, 0x0000);
  write_cmos_sensor(0xF49A, 0x0000);
  write_cmos_sensor(0xF49C, 0x0000);
  write_cmos_sensor(0xF49E, 0x0000);
  write_cmos_sensor(0xF4A0, 0x0000);
  write_cmos_sensor(0xF4A2, 0x0000);
  write_cmos_sensor(0xF4A4, 0x0000);
  write_cmos_sensor(0xF4A6, 0x0000);
  write_cmos_sensor(0xF4A8, 0x0000);
  write_cmos_sensor(0xF4AA, 0x0000);
  write_cmos_sensor(0xF4AC, 0x0000);
  write_cmos_sensor(0xF4AE, 0x0000);
  write_cmos_sensor(0xF4B0, 0x0000);
  write_cmos_sensor(0xF4B2, 0x0000);
  write_cmos_sensor(0xF4B4, 0x0000);
  write_cmos_sensor(0xF4B6, 0x0000);
  write_cmos_sensor(0xF4B8, 0x0000);
  write_cmos_sensor(0xF4BA, 0x0000);
  write_cmos_sensor(0xF4BC, 0x0000);
  write_cmos_sensor(0xF4BE, 0x0000);
  write_cmos_sensor(0xF4C0, 0x0000);
  write_cmos_sensor(0xF4C2, 0x0000);
  write_cmos_sensor(0xF4C4, 0x0000);
  write_cmos_sensor(0x0202, 0x0010);
  write_cmos_sensor(0x0226, 0x0010);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0B06, 0x0101);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x107A);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x1074);
  write_cmos_sensor(0x6F12, 0x1D00);
  write_cmos_sensor(0x602A, 0x0E7C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1120);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1122);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x602A, 0x1128);
  write_cmos_sensor(0x6F12, 0x0604);
  write_cmos_sensor(0x602A, 0x1AC0);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x1AC2);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x1494);
  write_cmos_sensor(0x6F12, 0x3D68);
  write_cmos_sensor(0x602A, 0x1498);
  write_cmos_sensor(0x6F12, 0xF10D);
  write_cmos_sensor(0x602A, 0x1488);
  write_cmos_sensor(0x6F12, 0x0F04);
  write_cmos_sensor(0x602A, 0x148A);
  write_cmos_sensor(0x6F12, 0x170B);
  write_cmos_sensor(0x602A, 0x150E);
  write_cmos_sensor(0x6F12, 0x00C2);
  write_cmos_sensor(0x602A, 0x1510);
  write_cmos_sensor(0x6F12, 0xC0AF);
  write_cmos_sensor(0x602A, 0x1512);
  write_cmos_sensor(0x6F12, 0x00A0);
  write_cmos_sensor(0x602A, 0x1486);
  write_cmos_sensor(0x6F12, 0x1430);
  write_cmos_sensor(0x602A, 0x1490);
  write_cmos_sensor(0x6F12, 0x4D09);
  write_cmos_sensor(0x602A, 0x149E);
  write_cmos_sensor(0x6F12, 0x01C4);
  write_cmos_sensor(0x602A, 0x11CC);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x11CE);
  write_cmos_sensor(0x6F12, 0x000B);
  write_cmos_sensor(0x602A, 0x11D0);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x602A, 0x11DA);
  write_cmos_sensor(0x6F12, 0x0012);
  write_cmos_sensor(0x602A, 0x11E6);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x602A, 0x125E);
  write_cmos_sensor(0x6F12, 0x0048);
  write_cmos_sensor(0x602A, 0x11F4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x11F8);
  write_cmos_sensor(0x6F12, 0x0016);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0xF444, 0x05BF);
  write_cmos_sensor(0xF44A, 0x0008);
  write_cmos_sensor(0xF44E, 0x0012);
  write_cmos_sensor(0xF46E, 0x40C0);
  write_cmos_sensor(0xF470, 0x7809);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1CAA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CAE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CB8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CBE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC6);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1CC8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6000);
  write_cmos_sensor(0x6F12, 0x000F);
  write_cmos_sensor(0x602A, 0x6002);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x602A, 0x6004);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x6006);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6008);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x600E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6010);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6012);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6014);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6016);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6018);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x601E);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6020);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6022);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6024);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6026);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x6028);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602A);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x602C);
  write_cmos_sensor(0x6F12, 0x1000);
  write_cmos_sensor(0x602A, 0x1144);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1146);
  write_cmos_sensor(0x6F12, 0x1B00);
  write_cmos_sensor(0x602A, 0x1080);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x1084);
  write_cmos_sensor(0x6F12, 0x00C0);
  write_cmos_sensor(0x602A, 0x108A);
  write_cmos_sensor(0x6F12, 0x00C0);
  write_cmos_sensor(0x602A, 0x1090);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1092);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1094);
  write_cmos_sensor(0x6F12, 0xA32E);

  
 } /* custom1 setting */

static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
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
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    /*if(hbb_flag == 0)
    {
        while(1){
            while (imgsensor_info.i2c_addr_table[i] != 0xff) {
                spin_lock(&imgsensor_drv_lock);
                imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
                spin_unlock(&imgsensor_drv_lock);
                do {
                    *sensor_id = return_sensor_id();
                    if (*sensor_id == imgsensor_info.sensor_id) {
        #ifdef CONFIG_MTK_CAM_CAL
	            //read_imx135_otp_mtk_fmt();
        #endif
			            LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
                        //return ERROR_NONE;
                    }
		            LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
                    retry--;
                } while(retry > 0);
                i++;
                retry = 1;
            }
        }    
    }else*/
        while (imgsensor_info.i2c_addr_table[i] != 0xff) {
            spin_lock(&imgsensor_drv_lock);
            imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
            spin_unlock(&imgsensor_drv_lock);
            do {
                *sensor_id = return_sensor_id();
                if (*sensor_id == imgsensor_info.sensor_id) {
//    #ifdef CONFIG_MTK_CAM_CAL
		    s5kgm1_get_otp_data();
//    #endif
				    LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
//				s5kgm1kwCheckLensVersion(imgsensor.i2c_write_id);
                    return ERROR_NONE;
                }
			    LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
                retry--;
            } while(retry > 0);
            i++;
            retry = 1;
        }    
    
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
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
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();

    //[agold][chenwei][20190313][start]
    #ifndef AGOLD_S5KGM1_AWB_DISABLE
//    s5kgm1_get_otp_data();
    #endif
    //[agold][chenwei][20190313][end]
	
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    //flag = read_cmos_sensor_16_16(0x05);  
    
   // LOG_INF("Read sensor id 0x05, 0x%x\n", flag);    


    return ERROR_NONE;
}   /*  open  */



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
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	/*PIP24fps_capture_setting(imgsensor.current_fps,imgsensor.pdaf_mode); */
	custom1_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}


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
#ifdef FANPENGTAO
	int i=0;
#endif

	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);	
	mdelay(10);
	#ifdef FANPENGTAO
	//int i=0;
	for(i=0; i<10; i++){
		LOG_INF("delay time = %d, the frame no = %d\n", i*10, read_cmos_sensor(0x0005));
		mdelay(10);
	}
	#endif
	return ERROR_NONE;
}	/*	preview   */

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
    int i;
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 
	else  
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("cap115fps: use cap1's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else  { //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("Warning:=== current_fps %d fps is not support, so use cap1's setting\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps); 
	set_mirror_flip(IMAGE_NORMAL);	
	mdelay(10);


	for(i=0; i<10; i++){
		LOG_INF("delay time = %d, the frame no = %d\n", i*10, read_cmos_sensor(0x0005));
		mdelay(10);
	}

#if 0
	if(imgsensor.test_pattern == KAL_TRUE)
	{
		//write_cmos_sensor(0x5002,0x00);
  }
#endif

	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);	
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}
	
/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;
	
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame; 

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame; /* The delay frame of setting frame length  */
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;
	#ifdef FPTPDAFSUPPORT
	sensor_info->PDAF_Support = 1;
	#else 
	sensor_info->PDAF_Support = 0;
	#endif

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
        case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
            break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

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
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		 write_cmos_sensor(0x6028, 0x2000);
		 write_cmos_sensor(0x602A, 0x1082);
		 write_cmos_sensor(0x6F12, 0x0000);
		 write_cmos_sensor(0x3734, 0x0001);
		 write_cmos_sensor(0x0600, 0x0308);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		 write_cmos_sensor(0x6028, 0x2000);
		 write_cmos_sensor(0x602A, 0x1082);
		 write_cmos_sensor(0x6F12, 0x8010);
		 write_cmos_sensor(0x3734, 0x0010);
		 write_cmos_sensor(0x0600, 0x0300);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;    
    kal_uint32 rate;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(ana_gain_table_8x)/sizeof(char);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)ana_gain_table_8x,
			sizeof(ana_gain_table_8x)/sizeof(char));
		}
		break;
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CUSTOM1:
                    memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
			break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;

	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
		streaming_control(KAL_TRUE);
		break;
        /******************** PDAF START >>> *********/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; // video & capture use same setting
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:	
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
//			read_gm1_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;	
        /******************** PDAF END   <<< *********/
		case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:/*lzl*/
				set_shutter_frame_length((UINT16)*feature_data, (UINT16)*(feature_data+1));
				break;
		case SENSOR_FEATURE_GET_PIXEL_RATE:

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.cap.pclk /
					(imgsensor_info.cap.linelength - 80))*
					imgsensor_info.cap.grabwindow_width;

					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.normal_video.pclk /
					(imgsensor_info.normal_video.linelength - 80))*
					imgsensor_info.normal_video.grabwindow_width;

					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.hs_video.pclk /
					(imgsensor_info.hs_video.linelength - 80))*
					imgsensor_info.hs_video.grabwindow_width;

					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.slim_video.pclk /
					(imgsensor_info.slim_video.linelength - 80))*
					imgsensor_info.slim_video.grabwindow_width;

					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.pre.pclk /
					(imgsensor_info.pre.linelength - 80))*
					imgsensor_info.pre.grabwindow_width;
					break;
			}
			break;			
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					rate = imgsensor_info.cap.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					rate = imgsensor_info.normal_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					rate = imgsensor_info.hs_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					rate = imgsensor_info.slim_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					rate = imgsensor_info.pre.mipi_pixel_rate;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
			break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.cap.pclk;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.normal_video.pclk;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.hs_video.pclk;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.custom1.pclk;
				break;
			//case MSDK_SCENARIO_ID_CUSTOM2:
			//	*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			//	    = imgsensor_info.custom2.pclk;
			//	break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.slim_video.pclk;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.pre.pclk;
		    break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.cap.framelength << 16)
				    + imgsensor_info.cap.linelength;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.normal_video.framelength << 16)
				    + imgsensor_info.normal_video.linelength;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.hs_video.framelength << 16)
				    + imgsensor_info.hs_video.linelength;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.slim_video.framelength << 16)
				    + imgsensor_info.slim_video.linelength;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.custom1.framelength << 16)
				    + imgsensor_info.custom1.linelength;
				break;
			//case MSDK_SCENARIO_ID_CUSTOM2:
			//	*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			//	= (imgsensor_info.custom2.framelength << 16)
			//	    + imgsensor_info.custom2.linelength;
			//	break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.pre.framelength << 16)
				    + imgsensor_info.pre.linelength;
				break;
		}
		break;
		default:
	            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KGM1_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL) {
		*pfFunc=&sensor_func;
	}
	return ERROR_NONE;
}	/*	S5Kgm1_MIPI_RAW_SensorInit	*/



