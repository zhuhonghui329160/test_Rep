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

#ifndef __MMC5603NJ_H__
#define __MMC5603NJ_H__


#include <stdint.h>
#include <stdbool.h>

#define GEOMAGNETISM_DEVICE_ADDR 0x30

//geomagnetism sensor register Addr
//3-aixs Data
//option mode: 16bit out[19:4];18bit out[19:2];20bit out[19:0];
#define GMREG00  0x00  //Xout0   //Xout[19:12]
#define GMREG01  0x01  //Xout1   //Xout[11:4]
#define GMREG02  0x02  //Yout0   
#define GMREG03  0x03  //Yout1
#define GMREG04  0x04  //Zout0
#define GMREG05  0x05  //Zout1
#define GMREG06  0x06  //Xout2   //Xout[3:0] 
#define GMREG07  0x07  //Yout2
#define GMREG08  0x08  //Zout2

//tempreture Data
#define GMREG09  0x09  //Tempreture output //0x00 = -75℃

//
//bit5: selftest PASS signal 0:PASS
//bit6: measure magnetic done and data ready stat 1:data was ready
//bit7: measure tempreture done and data ready stat 1:data was ready 
#define GMREG18  0x18  //Dev status1

//
//use of continuous-mode measurement
//0~255 HZ; if(hpower(control reg2 bit7) = 1 && this reg value = 255) ODR=1000Hz; 
#define GMREG1A  0x1A  //Output data rate

//Control Reg
#define GMREG1B  0x1B  //Control Reg0
//bit0:measure Magnetic; set 1:measure ;selfclear;
//bit1:measure tempreture;
//bit3:  1:Set option;selfclear;
//bit4:  1:Reset option;selfclear;
//bit5: Auto Set/Reset;
//bit6: Auto SelfTest;selfclear;
//bit7: 1:calc measurement period;set before continuou-mode measure are started;
#define GMREG1C  0x1C  //Control Reg1
//bit0:BW0;bit1:BW1;  Bandwitch Selection
//	  BW1  BW0  MeasurementTime
//    0    0    6.6ms
//    0    1    3.5ms
//    1    0    2.0ms
//    1    1    1.2ms          X/Y/Z single channel delaytime = 1/3
//bit2/3/4:X/Y/Z channel inhibit; Y/Z needs to be inhibited the same time;
//bit5/6:St_enp/St_enm; 
//bit7:Software Reset;
#define GMREG1D  0x1D  //Control Reg2
//bit[2:0]: Number of measurements before Set option;
//bit3: 1:Enable periodical set;
//bit4: 1:Enable Continuous-mode measurement;
//bit7: 1:achieve 1000Hz ODR;

//selftest config
#define GMREG1E  0x1E  //X-aixs selftest threshold//阈值
#define GMREG1F  0x1F  //Y-aixs selftest threshold
#define GMREG20  0x20  //Z-aixs selftest threshold
#define GMREG27  0x27  //X-aixs selftest set value
#define GMREG28  0x28  //Y-aixs selftest set value
#define GMREG29  0x29  //Z-aixs selftest set value

//
#define GMREG39  0x39  //Product ID

#define GMREGSIZE (1)




#endif
