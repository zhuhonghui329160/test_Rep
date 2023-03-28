/*
 * Copyright (c) 2018-2022, Qiancheng Technology Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-01     Hohn      the first version
 *
 */

#ifndef __SENSOR_GEOMAGNETISM_MMC5603_H__
#define __SENSOR_GEOMAGNETISM_MMC5603_H__

#include "MMC5603NJ.h"

#define MMC5603_REG_DATA			GMREG00
#define MMC5603_REG_XL				GMREG00
#define MMC5603_REG_XH				GMREG01
#define MMC5603_REG_YL				GMREG02
#define MMC5603_REG_YH				GMREG03
#define MMC5603_REG_ZL				GMREG04
#define MMC5603_REG_ZH				GMREG05
 
#define MMC5603_REG_STATUS1			GMREG18
//#define MMC5603_REG_STATUS0			GMREG19
 
#define MMC5603_REG_ODR				GMREG1A
#define MMC5603_REG_CTRL0			GMREG1B
#define MMC5603_REG_CTRL1			GMREG1C
#define MMC5603_REG_CTRL2			GMREG1D
 
#define MMC5603_REG_X_THD			GMREG1E
#define MMC5603_REG_Y_THD			GMREG1F
#define MMC5603_REG_Z_THD			GMREG20
 
#define MMC5603_REG_ST_X_VAL		GMREG27
#define MMC5603_REG_ST_Y_VAL		GMREG28
#define MMC5603_REG_ST_Z_VAL		GMREG29
 
#define MMC5603_REG_PRODUCTID1		GMREG39

/*Bit definition for status register 0x18*/
#define MMC5603_STAT_M_T_DONE       0x01
#define MMC5603_STAT_M_M_DONE       0x02

/* Bit definition for control register ODR 0x1A */
#define MMC5603_CMD_ODR_1HZ			0x01
#define MMC5603_CMD_ODR_5HZ			0x05
#define MMC5603_CMD_ODR_10HZ		0x0A
#define MMC5603_CMD_ODR_50HZ		0x32
#define MMC5603_CMD_ODR_100HZ		0x64
#define MMC5603_CMD_ODR_200HZ		0xC8
#define MMC5603_CMD_ODR_255HZ		0xFF
 
/* Bit definition for control register 0 0x1B */
#define MMC5603_CMD_TMM				0x01
#define MMC5603_CMD_TMT         	0x02
#define MMC5603_CMD_START_MDT		0x04
#define MMC5603_CMD_SET				0x08
#define MMC5603_CMD_RESET			0x10
#define MMC5603_CMD_AUTO_SR_EN		0x20
#define MMC5603_CMD_AUTO_ST_EN		0x40
#define MMC5603_CMD_CMM_FREQ_EN		0x80
 
/* Bit definition for control register 1 0x1C */
#define MMC5603_CMD_BW00			0x00
#define MMC5603_CMD_BW01			0x01
#define MMC5603_CMD_BW10			0x02
#define MMC5603_CMD_BW11			0x03
#define MMC5603_CMD_ST_ENP			0x20
#define MMC5603_CMD_ST_ENM			0x40
#define MMC5603_CMD_SW_RST			0x80
 
/* Bit definition for control register 2 0x1D */
#define MMC5603_CMD_PART_SET1		0x00
#define MMC5603_CMD_PART_SET25		0x01
#define MMC5603_CMD_PART_SET75		0x02
#define MMC5603_CMD_PART_SET100		0x03
#define MMC5603_CMD_PART_SET250		0x04
#define MMC5603_CMD_PART_SET500		0x05
#define MMC5603_CMD_PART_SET1000	0x06
#define MMC5603_CMD_PART_SET2000	0x07
#define MMC5603_CMD_EN_PART_SET		0x08
#define MMC5603_CMD_CMM_EN			0x10
#define MMC5603_CMD_INT_MDT_EN		0x20
#define MMC5603_CMD_INT_MD_EN		0x40
#define MMC5603_CMD_HPOWER			0x80

#define MMC5603_PRODUCT_ID			0x10
#define MMC5603_MM_DONE_INT			0x01
#define MMC5603_MT_DONE_INT			0x02
#define MMC5603_MDT_FLAG_INT		0x04
#define MMC5603_ST_FAIL_INT			0x08
#define MMC5603_OTP_READ_DONE		0x10
#define MMC5603_SAT_SENSOR			0x20
#define MMC5603_MM_DONE				0x40
#define MMC5603_MT_DONE				0x80
 
// 16-bit mode, null field output (32768)
#define	MMC5603_16BIT_OFFSET		32768
#define	MMC5603_16BIT_SENSITIVITY	1024
 
struct magnetic_field
{
    float x;
    float y;
    float z;
};

enum magnetic_sampling_mode
{
	CONTINUOUS_SAMPLING = 1,
	SINGLE_SAMPLING,
};
 
int geomagnetism_sensorhub_init(void);



#endif

