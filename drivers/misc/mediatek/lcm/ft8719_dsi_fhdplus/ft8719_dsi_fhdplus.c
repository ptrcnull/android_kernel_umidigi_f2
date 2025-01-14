/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;


#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))
#define MDELAY(n)               (lcm_util.mdelay(n))
#define UDELAY(n)               (lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
        lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
                lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
          lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
                lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "lcm_i2c.h"

// ---------------------------------------------------------------------------
// Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH (1080)
#define FRAME_HEIGHT (2340)

#define LCM_PHYSICAL_WIDTH									(69500)
#define LCM_PHYSICAL_HEIGHT									(150580)

#define REGFLAG_DELAY          	0XFE
#define REGFLAG_END_OF_TABLE  	0xFC   // END OF REGISTERS MARKER

#define LCM_ID_FT8719          0x8719
#define LCM_DSI_CMD_MODE	0
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// ---------------------------------------------------------------------------
// Local Variables
// ---------------------------------------------------------------------------
#define LOG_TAG "LCM"
#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif


static struct LCM_UTIL_FUNCS lcm_util;


#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
// Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif

#ifdef BUILD_LK
#define GPIO_LDO_1V8 	   (GPIO160 | 0x80000000)
#endif

// static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
// {
// #ifdef BUILD_LK
	// mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	// mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	// mt_set_gpio_out(GPIO, output);
// #else
	// gpio_set_value(GPIO, output);
// #endif
// }


struct LCM_setting_table {
unsigned char cmd;
unsigned char count;
unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x01,0,{}},
	//{REGFLAG_DELAY,200, {}},
	{0x00,1,{0x00}},
	{0xFF,3,{0x87,0x19,0x01}},
	{0x00,1,{0x80}},
	{0xFF,2,{0x87,0x19}},
	{0x00,1,{0x82}},
	{0xA4,2,{0x29,0x23}},
	{0x00,1,{0x80}},
	{0xA7,1,{0x03}},
	{0x00,1,{0x82}},
	{0xA7,2,{0x11,0x01}},
	{0x00,1,{0x90}},
	{0xC3,4,{0x0C,0x00,0x00,0x01}},
	{0x00,1,{0xA0}},
	{0xC3,7,{0x31,0x21,0x02,0x10,0x01,0x20,0x12}},
	{0x00,1,{0xB3}},
	{0xC5,1,{0x08}},
	{0x00,1,{0x80}},
	{0xC2,4,{0x82,0x01,0x20,0x56}},
	{0x00,1,{0xA0}},
	{0xC2,15,{0x00,0x00,0x00,0x24,0x98,0x01,0x00,0x00,0x24,0x98,0x02,0x00,0x00,0x24,0x98}},
	{0x00,1,{0xB0}},
	{0xC2,10,{0x03,0x00,0x00,0x24,0x98,0x00,0x02,0x03,0x00,0x80}},
	{0x00,1,{0xE0}},
	{0xC2,8,{0x33,0x33,0x73,0x33,0x33,0x33,0x00,0x00}},
	{0x00,1,{0xFA}},
	{0xC2,3,{0x23,0xFF,0x23}},
	{0x00,1,{0x80}},
	{0xCB,16,{0x00,0x01,0x00,0x00,0xFD,0x01,0x00,0x00,0x00,0x00,0xFD,0x01,0x00,0x01,0x00,0x03}},
	{0x00,1,{0x90}},
	{0xCB,16,{0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xA0}},
	{0xCB,4,{0x00,0x00,0x00,0x00}},
	{0x00,1,{0xB0}},
	{0xCB,4,{0x55,0x55,0x55,0x57}},
	{0x00,1,{0x80}},
	{0xCC,16,{0x00,0x29,0x00,0x23,0x00,0x0A,0x00,0x00,0x09,0x08,0x07,0x06,0x00,0x00,0x00,0x00}},
	{0x00,1,{0x90}},
	{0xCC,8,{0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00,1,{0x80}},
	{0xCD,16,{0x00,0x00,0x00,0x02,0x00,0x0A,0x00,0x00,0x09,0x08,0x07,0x06,0x00,0x00,0x00,0x00}},
	{0x00,1,{0x90}},
	{0xCD,8,{0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00,1,{0xA0}},
	{0xCC,16,{0x00,0x29,0x00,0x23,0x00,0x0A,0x00,0x00,0x06,0x07,0x08,0x09,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xB0}},
	{0xCC,8,{0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00,1,{0xA0}},
	{0xCD,16,{0x00,0x00,0x00,0x02,0x00,0x0A,0x00,0x00,0x06,0x07,0x08,0x09,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xB0}},
	{0xCD,8,{0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00,1,{0x80}},
	{0xC0,6,{0x00,0x7A,0x00,0x6C,0x00,0x10}},
	{0x00,1,{0x89}},
	{0xC0,3,{0x01,0x1D,0x04}},
	{0x00,1,{0xA0}},
	{0xC0,6,{0x01,0x09,0x00,0x3A,0x00,0x10}},
	{0x00,1,{0xB0}},
	{0xC0,5,{0x00,0x7A,0x02,0x10,0x10}},
	{0x00,1,{0xC1}},
	{0xC0,8,{0x00,0xB1,0x00,0x8B,0x00,0x76,0x00,0xD0}},
	{0x00,1,{0xCA}},
	{0xC0,1,{0x80}},
	{0x00,1,{0xD7}},
	{0xC0,6,{0x00,0x76,0x00,0x6F,0x00,0x10}},
	{0x00,1,{0xA5}},
	{0xC1,4,{0x00,0x36,0x00,0x02}},
	{0x00,1,{0x82}},
	{0xCE,13,{0x01,0x09,0x00,0xD8,0x00,0xD8,0x00,0x90,0x00,0x90,0x0D,0x0E,0x09}},
	{0x00,1,{0x90}},
	{0xCE,8,{0x00,0x82,0x0D,0x5C,0x00,0x82,0x80,0x09}},
	{0x00,1,{0xA0}},
	{0xCE,3,{0x00,0x00,0x00}},
	{0x00,1,{0xB0}},
	{0xCE,3,{0x11,0x00,0x00}},
	{0x00,1,{0xD1}},
	{0xCE,7,{0x00,0x0A,0x01,0x01,0x00,0x5D,0x01}},
	{0x00,1,{0xE1}},
	{0xCE,11,{0x08,0x02,0x15,0x02,0x15,0x02,0x15,0x00,0x2B,0x00,0x60}},
	{0x00,1,{0xF1}},
	{0xCE,9,{0x16,0x0B,0x0F,0x01,0x12,0x01,0x11,0x01,0x23}},
	{0x00,1,{0xB0}},
	{0xCF,4,{0x00,0x00,0x6C,0x70}},
	{0x00,1,{0xB5}},
	{0xCF,4,{0x04,0x04,0xA4,0xA8}},
	{0x00,1,{0xC0}},
	{0xCF,4,{0x08,0x08,0xCA,0xCE}},
	{0x00,1,{0xC5}},
	{0xCF,4,{0x00,0x00,0x08,0x0C}},
	{0x00,1,{0x90}},
	{0xC0,6,{0x00,0x7A,0x00,0x6C,0x00,0x10}},
	{0x00,1,{0xA1}},
	{0xB3,6,{0x04,0x38,0x09,0x24,0xC0,0xF8}},
	{0x00,1,{0x82}},
	{0xC5,7,{0x4B,0x4B,0x3C,0x3C,0x00,0x60,0x0C}},
	{0x00,1,{0x00}},
	{0xD8,2,{0x2B,0x2B}},
	{0x00,1,{0x00}},
	{0xD9,3,{0x00,0x95,0x95}},
	{0x00,1,{0xA3}},
	{0xC5,1,{0x1B}},
	{0x00,1,{0xA9}},
	{0xC5,1,{0x21}},
	{0x00,1,{0x86}},
	{0xC3,3,{0x00,0x00,0x00}},
	{0x00,1,{0x89}},
	{0xF5,1,{0x5F}},
	{0x00,1,{0x96}},
	{0xF5,1,{0x5F}},
	{0x00,1,{0xA6}},
	{0xF5,1,{0x5F}},
	{0x00,1,{0xB1}},
	{0xF5,1,{0x1E}},
	{0x00,1,{0x81}},
	{0xF5,2,{0x5F,0x5F}},
	{0x00,1,{0x86}},
	{0xF5,2,{0x5F,0x5F}},
	{0x00,1,{0xAA}},
	{0xF5,1,{0x8E}},
	{0x00,1,{0x85}},
	{0xC4,1,{0x1E}},
	{0x00,1,{0xB7}},
	{0xCE,2,{0x2B,0x05}},
	{0x00,1,{0x90}},
	{0xC5,1,{0x83}},
	{0x00,1,{0x92}},
	{0xC5,1,{0x63}},
	{0x00,1,{0xE8}},
	{0xC0,1,{0x40}},
	{0x00,1,{0x87}},
	{0xC4,1,{0x40}},
	{0x00,1,{0x9B}},
	{0xF5,4,{0x8D,0x8C,0x8D,0x8A}},
	{0x00,1,{0x91}},
	{0xF5,2,{0xED,0x8C}},
	{0x00,1,{0x95}},
	{0xF5,1,{0x8A}},
	{0x00,1,{0x98}},
	{0xF5,1,{0xEB}},
	{0x00,1,{0x85}},
	{0xA7,1,{0x0F}},
	{0x00,1,{0x00}},
	{0xE1,40,{0x00,0x04,0x04,0x09,0x0C,0x15,0x1D,0x24,0x2E,0x72,0x37,0x3E,0x44,0x4A,0x84,0x4F,0x58,0x60,0x67,0x57,0x6F,0x76,0x7E,0x87,0x18,0x90,0x96,0x9D,0xA4,0xD0,0xAC,0xB7,0xC6,0xCE,0x97,0xD9,0xE9,0xF4,0xFB,0xFF}},
	{0x00,1,{0x00}},
	{0xE2,40,{0x00,0x04,0x04,0x09,0x0C,0x16,0x1E,0x25,0x2F,0x72,0x38,0x40,0x47,0x4D,0xA4,0x52,0x5B,0x63,0x6A,0x57,0x72,0x77,0x7E,0x87,0x38,0x90,0x96,0x9D,0xA4,0xD0,0xAC,0xB7,0xC4,0xCD,0x9F,0xD9,0xE9,0xF4,0xFB,0xFF}},
	{0x00,1,{0x00}},
	{0xE3,40,{0x00,0x04,0x04,0x09,0x0C,0x15,0x1D,0x24,0x2E,0x72,0x37,0x3E,0x44,0x4A,0x84,0x4F,0x58,0x60,0x67,0x57,0x6F,0x76,0x7E,0x87,0x18,0x90,0x96,0x9D,0xA4,0xD0,0xAC,0xB7,0xC6,0xCE,0x97,0xD9,0xE9,0xF4,0xFB,0xFF}},
	{0x00,1,{0x00}},
	{0xE4,40,{0x00,0x04,0x04,0x09,0x0C,0x16,0x1E,0x25,0x2F,0x72,0x38,0x40,0x47,0x4D,0xA4,0x52,0x5B,0x63,0x6A,0x57,0x72,0x77,0x7E,0x87,0x38,0x90,0x96,0x9B,0xA3,0xDE,0xAC,0xB7,0xC4,0xCD,0x9F,0xD9,0xE9,0xF4,0xFB,0xFF}},
	{0x00,1,{0x00}},
	{0xE5,40,{0x00,0x04,0x04,0x09,0x0C,0x15,0x1D,0x24,0x2E,0x72,0x37,0x3E,0x44,0x4A,0x84,0x4F,0x58,0x60,0x67,0x57,0x6F,0x76,0x7E,0x87,0x18,0x90,0x96,0x9D,0xA4,0xD0,0xAC,0xB7,0xC6,0xCE,0x97,0xD9,0xE9,0xF4,0xFB,0xFF}},
	{0x00,1,{0x00}},
	{0xE6,40,{0x00,0x04,0x04,0x09,0x0C,0x16,0x1E,0x25,0x2F,0x72,0x38,0x40,0x47,0x4D,0xA4,0x52,0x5B,0x63,0x6A,0x57,0x72,0x77,0x7E,0x87,0x38,0x90,0x96,0x9D,0xA4,0xD0,0xAC,0xB7,0xC4,0xCD,0x9F,0xD9,0xE9,0xF4,0xFB,0xFF}},
	{0x00,1,{0xB0}},
	{0xF5,1,{0x00}},
	{0x00,1,{0xC1}},
	{0xB6,3,{0x09,0x89,0x68}},
	{0x00,1,{0x80}},
	{0xB4,1,{0x0A}},
	{0x00,1,{0x8C}},
	{0xC3,1,{0x01}},
	{0x00,1,{0x8E}},
	{0xC3,1,{0x10}},
	{0x00,1,{0x8A}},
	{0xC0,2,{0x1C,0x05}},
	{0x00,1,{0xB0}},
	{0xF3,2,{0x02,0xFD}},
	{0x11,0,{}},
	{REGFLAG_DELAY,120, {}},
	{0x29,0,{}},
	{0x35,1,{0x01}},
	{REGFLAG_DELAY,20, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {

		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;

			case REGFLAG_END_OF_TABLE :
			break;

			default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
// LCM Driver Implementations
// ---------------------------------------------------------------------------


static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)

{
    memset(params, 0, sizeof(struct LCM_PARAMS));


    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
#ifndef BUILD_LK
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
#endif

    // enable tearing-free
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

    #if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
    #else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
    #endif

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;
    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=FRAME_WIDTH*3;
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 26;
	params->dsi.vertical_frontporch					= 112;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 6;
	params->dsi.horizontal_backporch				= 16;
	params->dsi.horizontal_frontporch				= 16;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 555;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void ft8716_lcm_init_power(void)
{
#ifdef BUILD_LK
	int ret=0;
	ret = PMU_REG_MASK(0xB2, (0x3 << 6), (0x3 << 6));

	/* set AVDD 5.4v, (4v+28*0.05v) */
	ret = PMU_REG_MASK(0xB3, 0x24, (0x3F << 0));//5.8v

	if (ret < 0)
		LCM_LOGI("ft8613----cmd=%0x--i2c write error----\n", 0xB3);
	else
		LCM_LOGI("ft8613----cmd=%0x--i2c write success----\n", 0xB3);

	/* set AVEE */
	ret = PMU_REG_MASK(0xB4, 0x24, (0x3F << 0));//5.8V

	if (ret < 0)
		LCM_LOGI("ft8613----cmd=%0x--i2c write error----\n", 0xB4);
	else
		LCM_LOGI("ft8613----cmd=%0x--i2c write success----\n", 0xB4);

	/* enable AVDD & AVEE */
	/* 0x12--default value; bit3--Vneg; bit6--Vpos; */
	ret = PMU_REG_MASK(0xB1,  (1<<6), (1<<6));
	if (ret < 0)
		LCM_LOGI("ft8613----Vpos cmd=%0x--i2c write error----\n", 0xB1);
	else
		LCM_LOGI("ft8613----Vpos cmd=%0x--i2c write success----\n", 0xB1);
	MDELAY(10);
	ret = PMU_REG_MASK(0xB1, (1<<3), (1<<3));
	if (ret < 0)
		LCM_LOGI("ft8613----Vneg cmd=%0x--i2c write error----\n", 0xB1);
	else
		LCM_LOGI("ft8613----Vneg cmd=%0x--i2c write success----\n", 0xB1);
	MDELAY(10);
#endif
}

#if 1
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static int lcm_read_ADC_value(void)
{
	int val;           //lvl = LCM_V_LEVEL;

	int dwChannel = 2; //LCM_ADC_CHAN;

	int data[4] = {0,0,0,0};
	int data0;
	//char* buf_temp;
	int res =0;
	//unsigned int ret;
	int num1_10;
	int num1_1;
	int num2_10;
	int num2_1;

    //read and calculate ADC value
	res = IMM_GetOneChannelValue(dwChannel,data,NULL);
	num1_10=data[0]/10;
	num1_1=data[0]%10;
	if(data[1]<10)
	{
	    num2_10=(data[1] * 10) / 10;
	    num2_1=(data[1] * 10) % 10;
	    data0=num1_10*1000+num1_1*100+num2_10*10+num2_1;
	    val=data0*100;
	}
	else
	{
	    num2_10=data[1] / 10;
	    num2_1=data[1] % 10;
	    data0=num1_10*1000+num1_1*100+num2_10*10+num2_1;
	    val=data0*10;
	}
    return val;
} 
#endif

static void lcm_init(void)
{
	int val;
	ft8716_lcm_init_power();

	SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
	
	val=lcm_read_ADC_value();
	
#ifdef BUILD_LK
	    printf("[tyd] lk lcm_ft8719------------------- val:%d\n",val);
#else
	    printk("[tyd] kernel lcm_ft8719------------------- val:%d\n",val);
#endif

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static struct LCM_setting_table lcm_sleep_in_setting[] =
{
    {0x28,0,{0x00}},
    {REGFLAG_DELAY, 10, {}},

    {0x10,0,{0x00}},
    {REGFLAG_DELAY, 120, {}},
	{0x00,1,{0x00}},
	{0xFF,3,{0x87,0x19,0x01}},
	{0x00,1,{0x80}},
	{0xFF,2,{0x87,0x19}},
	{0x00,1,{0x00}},
	{0xF7,4,{0x5A,0xA5,0x95,0x27}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void lcm_suspend(void)
{
    push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(15);
#ifdef BUILD_LK
	SET_RESET_PIN(0);
#else
	//lcm_ldo_gpio_set(LCM_RST, 0);
#endif
}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_init_power(void)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LDO_1V8, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LDO_1V8, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LDO_1V8, GPIO_OUT_ONE);
	MDELAY(100);
#else
    display_bias_enable();
#endif
}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
	MDELAY(10);
	mt_set_gpio_mode(GPIO_LDO_1V8, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LDO_1V8, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LDO_1V8, GPIO_OUT_ZERO);
	MDELAY(10);
#else
	display_bias_disable();
#endif
}

static void lcm_resume_power(void)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LDO_1V8, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LDO_1V8, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LDO_1V8, GPIO_OUT_ONE);
	MDELAY(100);
#else
	display_bias_enable();
#endif
}



static unsigned int lcm_compare_id(void)
{
	unsigned int id=0, id_high = 0, id_low = 0;
	unsigned char buffer[5];
	unsigned int array[16];
	unsigned int val = 0, lcm_id = 0;

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LDO_1V8, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LDO_1V8, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LDO_1V8, GPIO_OUT_ONE);
	MDELAY(100);
#else
#endif
	ft8716_lcm_init_power();
	
	SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

#if 0
	val = lcm_read_ADC_value();
	if((val > 200) && (val < 600))
		lcm_id = 1;
#endif

	array[0] = 0x00053700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xA1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];

	id = (id_high<<8) | id_low;
    #ifdef BUILD_LK
		printf("%s, lk FT8716 debug: FT8716 id = 0x%04x, val = %d, lcm_id = %d\n", __func__, id, val, lcm_id);
    #else
		printk("%s, kernel FT8716 horse debug: FT8716 id = 0x%04x, val = %d, lcm_id = %d\n", __func__, id, val, lcm_id);
    #endif

#if 0
	return ((LCM_ID_FT8719 == id) && (lcm_id == 1))?1:0;
#else
	 return (LCM_ID_FT8719 == id)?1:0;
#endif
}

// ---------------------------------------------------------------------------
// Get LCM Driver Hooks
// ---------------------------------------------------------------------------
struct LCM_DRIVER ft8719_dsi_fhdplus_drv =
{
    .name		    = "FT8719_DSI_FHD+",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};

