/*
 * Copyright (c) 2018-2022, Qiancheng Technology Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-1     Hohn      the first version
 *
 */
#include "sensor_geomagnetism_mmc5603.h"
#include "MMC5603NJ.h"

#include "cmsis_os.h"
#include "cmsis.h"
#include "hal_i2c.h"
#include "hal_trace.h"
#include "hal_timer.h"
#include <stdlib.h>
#include <string.h>

#include "hal_sysfreq.h"
#include "qc_sensorhub_app_func.h"
#include "cwm_lib_dml.h"
#include "algo_cywee_process.h"

#define GEOMAGN_DEBUG

struct i2c_dev {
	const char *name;
	enum HAL_I2C_ID_T bus;
	uint32_t addr;
	int speed;
};

static struct i2c_dev geomagnetism_dev = {
	.name = "mmc5603",
	.bus = (enum HAL_I2C_ID_T)HAL_I2C_ID_2,
	.addr = GEOMAGNETISM_DEVICE_ADDR,
	.speed = 400*1000,
};

static struct magnetic_field magnetic_field_now = {
		.x = 0,
		.y = 0,
		.z = 0,
};
static struct magnetic_field magnetic_field_OffSet = {
		.x = 0,
		.y = 0,
		.z = 0,
};



static osThreadId geomagnetism_thread_id = NULL;


osSemaphoreDef(geomagnetism_sem);
osSemaphoreId geomagnetism_sem_id = NULL;

/*************************Init Func*****************************************/
static int geomagnetism_sem_init(void)
{
    if (geomagnetism_sem_id == NULL) {
        geomagnetism_sem_id = osSemaphoreCreate(osSemaphore(geomagnetism_sem), 0);
    }
    ASSERT(geomagnetism_sem_id, "create geomagnetism_sem_id fail!");
    return 0;
}


static int geomagnetism_i2c_init()
{
    int ret = 0;
    struct HAL_I2C_CONFIG_T cfg = {0};
    cfg.mode      = HAL_I2C_API_MODE_SIMPLE;
    cfg.use_dma   = 1;
    cfg.use_sync  = 1;
    cfg.as_master = 1;
    cfg.speed = geomagnetism_dev.speed;
    ret = hal_i2c_open(geomagnetism_dev.bus, &cfg);
    if(ret != 0) {
        TRACE(0, "%s bus_id %d init err", __func__, geomagnetism_dev.bus);
        hal_i2c_close(geomagnetism_dev.bus);
        return -1;
    }
    TRACE(0, "%s bus_id %d init success", __func__, geomagnetism_dev.bus);
    return 0;
}

/*******************************R&T Func******************************************/
static int geomagnetism_i2c_write(uint8_t reg_addr, const uint8_t *pd, uint32_t dlen)
{
	uint32_t ret;
	uint8_t *tx_buf = malloc(dlen +1);
	*(tx_buf + 0) = reg_addr;
	memcpy(tx_buf + 1, pd, dlen);
	ret  = hal_i2c_simple_send(geomagnetism_dev.bus,geomagnetism_dev.addr,tx_buf,dlen + 1);
	free(tx_buf);
	if(ret){
		TRACE(0, "%s, send failed,ret=%d", __func__,ret);
		return -1;
	}
	return 0;
}

static int geomagnetism_i2c_read(uint8_t *reg_addr, uint32_t reg_len, uint8_t *pd, uint32_t dlen)
{
    uint32_t ret;
    ret = hal_i2c_simple_recv(geomagnetism_dev.bus, geomagnetism_dev.addr, reg_addr, reg_len, pd, dlen);
    if (ret) {
        /* pending! why failed */
         TRACE(0, "%s, read failed, ret=0x%x, dlen=%d",__func__, ret, dlen);
        return -1;
    }
    return 0;
}

int geomagnetism_write_regs(uint8_t reg_addr, const uint8_t *buf, uint8_t buf_len)
{
    int ret;
    ret = geomagnetism_i2c_write(reg_addr, buf, buf_len);
    return ret;
}

int geomagnetism_read_regs(uint8_t reg_addr, uint8_t *buf, uint8_t buf_len)
{
    int ret;
    ret = geomagnetism_i2c_read(&reg_addr, 1, buf, buf_len);
    return ret;
}

/*****************************Option Func*****************************************/
static uint8_t sampling_mode = CONTINUOUS_SAMPLING;//Auto or single

void geomagnetism_SET(void)
{
	uint8_t sendbuf = MMC5603_CMD_SET;
	geomagnetism_write_regs(MMC5603_REG_CTRL0,&sendbuf,1);

	osDelay(1);
}
void geomagnetism_RESET(void)
{
	uint8_t sendbuf = MMC5603_CMD_RESET;
	geomagnetism_write_regs(MMC5603_REG_CTRL0,&sendbuf,1);

	osDelay(1);
}
//Check product ID
int geomagnetism_CheckID(void)
{
	uint8_t pro_id = 0;

	geomagnetism_read_regs(MMC5603_REG_PRODUCTID1,&pro_id,1);
	TRACE(0,"Geomagnetism device product ID 0x%2x",pro_id);
	if(pro_id != MMC5603_PRODUCT_ID)
		return -1;
	
	return 1;
}
//Selftest configure
void geomagnetism_Auto_SelfTest_Configuration(void)
{
	int i;
	uint8_t reg_value[3];
	int16_t st_thr_data[3]={0};
	int16_t st_thr_new[3]={0};
	
	int16_t st_thd[3]={0};
	uint8_t st_thd_reg[3];		
 
	/* Read trim data from reg 0x27-0x29 */
	geomagnetism_read_regs(MMC5603_REG_ST_X_VAL, reg_value, 3);	
	for (i = 0; i < 3; i++)
	{
		st_thr_data[i] = (int16_t)(reg_value[i]-128)*32; 
		if (st_thr_data[i]<0)
			st_thr_data[i] = -st_thr_data[i];
		st_thr_new[i] = st_thr_data[i]-st_thr_data[i]/5;
		
		st_thd[i] = st_thr_new[i]/8;
		if (st_thd[i] > 255)
			st_thd_reg[i] = 0xFF;
		else
			st_thd_reg[i] = (uint8_t)st_thd[i];
	}
	/* Write threshold into the reg 0x1E-0x20 */
	geomagnetism_write_regs(MMC5603_REG_X_THD, st_thd_reg,3);
}

//Self test
int geomagnetism_Auto_SelfTest(void)
{
	uint8_t reg_status = 0;
	uint8_t sendbuf = MMC5603_CMD_AUTO_ST_EN | MMC5603_CMD_TMM;
	/* Write 0x40 to register 0x1B, set Auto_st_en bit high */
	geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf,1);
	
	/* Delay 15ms to finish the selftest process */
	osDelay(15);
	
	/* Read register 0x18, check Sat_sensor bit */
	geomagnetism_read_regs(MMC5603_REG_STATUS1, &reg_status,1);	
	if ((reg_status&MMC5603_SAT_SENSOR))
		return -1;
	
	return 1;
}

//Continuous mode configuration with auto set and reset
void geomagnetism_Continuous_Mode_With_Auto_SR(uint8_t bandwith, uint8_t sampling_rate)
{
	uint8_t sendbuf = bandwith;
	/* Write reg 0x1C, Set BW<1:0> = bandwith */
	geomagnetism_write_regs(MMC5603_REG_CTRL1, &sendbuf, 1);
	
	/* Write reg 0x1A, set ODR<7:0> = sampling_rate */
	sendbuf = sampling_rate;
	geomagnetism_write_regs(MMC5603_REG_ODR, &sendbuf, 1); 
	
	/* Write reg 0x1B */
	/* Set Auto_SR_en bit '1', Enable the function of automatic set/reset */
	/* Set Cmm_freq_en bit '1', Start the calculation of the measurement period according to the ODR*/	
	sendbuf = MMC5603_CMD_CMM_FREQ_EN|MMC5603_CMD_AUTO_SR_EN;
	geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);
 
	/* Write reg 0x1D */
	/* Set Cmm_en bit '1', Enter continuous mode */	
	sendbuf = MMC5603_CMD_CMM_EN;
	geomagnetism_write_regs(MMC5603_REG_CTRL2, &sendbuf, 1);
}

//Do selftest operation periodically
int geomagnetism_Saturation_Checking(void)
{	
	int ret = 0; //1 pass, -1 fail, 0 elapsed time is less 5 seconds
 
	/* If sampling rate is 50Hz, then do saturation checking every 250 loops, i.e. 5 seconds */
	static int NumOfSamples = 5;
	static int cnt = 0;
	uint8_t sendbuf = MMC5603_CMD_TMM;
 
	if ((cnt++) >= NumOfSamples) {
		cnt = 0;
		ret = geomagnetism_Auto_SelfTest();
		if (ret == -1) {
			/* Sensor is saturated, need to do SET operation */
			geomagnetism_SET();
			TRACE(0,"geomagnetism self test Fail !!! ");
		}
		else
		{
			TRACE(0,"geomagnetism self test success !!! ");
		}
		
		/* Do TM_M after selftest operation */
		geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);	
		osDelay(8);
	}
	return 1;
}

//Auto switch the working mode between Auto_SR and SETonly
void geomagnetism_Auto_Switch(uint16_t *mag)
{
	float mag_out[3];	
	uint8_t sendbuf = 0;
	
	mag_out[0] = ((float)mag[0] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;
	mag_out[1] = ((float)mag[1] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;
	mag_out[2] = ((float)mag[2] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY;
	
	if (sampling_mode == CONTINUOUS_SAMPLING) {
		/* If X or Y axis output exceed 10 Gauss, then switch to single mode */
		if ((ABS(mag_out[0])>10.0f) || (ABS(mag_out[1])>10.0f)) {	
			sampling_mode = SINGLE_SAMPLING;
			
			/* Disable continuous mode */	
			sendbuf = 0x00;
			geomagnetism_write_regs(MMC5603_REG_CTRL2, &sendbuf, 1);
			osDelay(15);//Delay 15ms to finish the last sampling
			
			/* Do SET operation */
			sendbuf = MMC5603_CMD_SET;
			geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);	
			osDelay(1);//Delay 1ms to finish the SET operation
			
			/* Do TM_M before next data reading */
			sendbuf = MMC5603_CMD_TMM;
			geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);	
			osDelay(8);//Delay 8ms to finish the TM_M operation			
		}	
	} else if (sampling_mode == SINGLE_SAMPLING) {
		/* If both of X and Y axis output less than 8 Gauss, then switch to continuous mode with Auto_SR */
		if ((ABS(mag_out[0])<8.0f) && (ABS(mag_out[1])<8.0f))	{	
			sampling_mode = CONTINUOUS_SAMPLING;
			
			/* Enable continuous mode with Auto_SR */
			sendbuf = MMC5603_CMD_CMM_FREQ_EN|MMC5603_CMD_AUTO_SR_EN;
			geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);
			sendbuf = MMC5603_CMD_CMM_EN;
			geomagnetism_write_regs(MMC5603_REG_CTRL2, &sendbuf, 1);
		} else {	
			/* Sensor checking */
			if (geomagnetism_Saturation_Checking()==0) {			
				/* Do TM_M before next data reading */
				sendbuf = MMC5603_CMD_TMM;
				geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);	
			}
		}		
	}
}


//Get magnetic field vector(磁场矢量) from device Reg;
uint8_t geomagnetism_GetData(struct magnetic_field *mf)
{
	uint8_t data_reg[6] = {0};
	uint16_t data_temp[3] = {0};
			
	/* Read register data */
	geomagnetism_read_regs(MMC5603_REG_DATA, data_reg, 6);
	
	/* Get high 16bits data */
	data_temp[0] = (uint16_t)(data_reg[0] << 8 | data_reg[1]);	
	data_temp[1] = (uint16_t)(data_reg[2] << 8 | data_reg[3]);			
	data_temp[2] = (uint16_t)(data_reg[4] << 8 | data_reg[5]); 
 
	/* Transform to unit Gauss */
	mf->x = ((float)data_temp[0] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY - magnetic_field_OffSet.x;
	mf->y = ((float)data_temp[1] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY - magnetic_field_OffSet.y;
	mf->z = ((float)data_temp[2] - MMC5603_16BIT_OFFSET)/MMC5603_16BIT_SENSITIVITY - magnetic_field_OffSet.z;
 
	geomagnetism_Auto_Switch(data_temp);	
	return 0;
}

//calculate magnetic field Offset
void geomagnetic_field_OffSet_get(struct magnetic_field *mf)
{
	uint8_t sendbuf = 0,readbuf = 0;
	uint8_t readcnt = 0;
	struct magnetic_field magnetic_field_backup = {
		.x = 0,
		.y = 0,
		.z = 0,
	};
	//Init
	mf->x = 0;
	mf->y = 0;
	mf->z = 0;
	
	//SET + measure = H + Offset
	geomagnetism_SET();
	sendbuf = MMC5603_CMD_TMM;
	geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);
	while(!(readbuf & MMC5603_STAT_M_M_DONE) && readcnt < 5)
	{
		geomagnetism_read_regs(MMC5603_REG_STATUS1, &readbuf, 1);
		readcnt++;
		osDelay(3);
	}
	geomagnetism_GetData(&magnetic_field_backup);
	TRACE(0,"Geomagnetic field backup: x=%.4f y=%.4f z=%.4f",magnetic_field_backup.x,magnetic_field_backup.y,magnetic_field_backup.z);
	//RESET + measure = -H + Offset
	readcnt = 0;
	geomagnetism_RESET();
	geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);
	while(!(readbuf & MMC5603_STAT_M_M_DONE) && readcnt < 5)
	{
		geomagnetism_read_regs(MMC5603_REG_STATUS1, &readbuf, 1);
		readcnt++;
		osDelay(3);
	}
	geomagnetism_GetData(&magnetic_field_now);
	TRACE(0,"Geomagnetic field now: x=%.4f y=%.4f z=%.4f",magnetic_field_now.x,magnetic_field_now.y,magnetic_field_now.z);
	//Offset= (H + offset + (-H) + offset)/2 ;
	mf->x = (magnetic_field_backup.x + magnetic_field_now.x) / 2;
	mf->y = (magnetic_field_backup.y + magnetic_field_now.y) / 2;
	mf->z = (magnetic_field_backup.z + magnetic_field_now.z) / 2;
	TRACE(0,"Geomagnetic field Offset: x_Offset=%.4f y_Offset=%.4f z_Offset=%.4f",mf->x,mf->y,mf->z);
}

//Disable Continuous mode measure
void geomagnetism_Disable(void)
{
	uint8_t sendbuf = 0;
	/* Write reg 0x1D */
	/* Set Cmm_en bit '0', Disable continuous mode */	
	geomagnetism_write_regs(MMC5603_REG_CTRL2, &sendbuf,1);
	
	osDelay(20);
}

//SW_Reset
void geomagnetism_SoftwareReset()
{
	uint8_t sendbuf = MMC5603_CMD_RESET;

	geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf,1);

	osDelay(20);
}

//Enable continuous mode measure
void geomagnetism_Enable(void)
{	
	int ret = 0;
	
	/* Inite the sensor state */
	sampling_mode = CONTINUOUS_SAMPLING;
	
	/* Check product ID */
	ret = geomagnetism_CheckID();
	if (ret<0)
		return;	

	/* Calclate Offset*/
	geomagnetic_field_OffSet_get(&magnetic_field_OffSet);
	
	/* Auto self-test registers configuration */
	geomagnetism_Auto_SelfTest_Configuration();
	
	/* Do SET operation */
	geomagnetism_SET();		

	ret = geomagnetism_Auto_SelfTest();
	if (ret == -1) {
		TRACE(0,"geomagnetism self test Fail !!! ");
		/* Do SET operation again*/
		//geomagnetism_SET();	
	}
	else
	{
		TRACE(0,"geomagnetism self test success !!! ");
	}
 
	/* Work mode setting */
	geomagnetism_Continuous_Mode_With_Auto_SR(MMC5603_CMD_BW00, 50);
	
	osDelay(20);
}

//get measure result
struct magnetic_field* geomagnetic_field_Get(void)
{
	return &magnetic_field_now;
}
/***************************Algo test fun********************************************/
int CWM_OS_i2cRead(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int busIndex) {
	
	uint32_t ret;
    ret = hal_i2c_simple_recv(geomagnetism_dev.bus, slaveAddr, (uint8_t *)reg, regLength, readData, readDataSize);
	TRACE(0, "slave addr : 0x%x", slaveAddr);
	if (ret) {
        /* pending! why failed */
         TRACE(0, "%s, read failed, ret=0x%x, dlen=%d",__func__, ret, readDataSize);
        return -1;
    }
    return 0;
}

int CWM_OS_i2cWrite(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int busIndex) {
	uint32_t ret;
	uint8_t *tx_buf = malloc(writeDataSize +1);
	*(tx_buf + 0) = (uint8_t)reg;
	memcpy(tx_buf + 1, writeData, writeDataSize);
	ret  = hal_i2c_simple_send(geomagnetism_dev.bus,slaveAddr,tx_buf,writeDataSize + 1);
	free(tx_buf);
	if(ret){
		TRACE(0, "%s, send failed,ret=%d", __func__,ret);
		return -1;
	}
	return 0;
}
int CWM_OS_spiRead (uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int busIndex) 
{return 0;}
int CWM_OS_spiWrite (uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int busIndex) 
{return 0;}

void CWM_AP_SensorListen_test(pSensorEVT_t sensorEVT) {
	TRACE(0,"test thread entry Ponit8");
	switch (sensorEVT->sensorType) {
        case 0:
			TRACE(0,"test thread entry Ponit9");
            TRACE(0,"acc: x=%.3f, y=%.3f, z=%.3f\n",
                                                       sensorEVT->fData[0],
                                                       sensorEVT->fData[1],
                                                       sensorEVT->fData[2]
                                                        );
            break;

        case 1:
            TRACE(0,"gyro: x=%.3f, y=%.3f, z=%.3f\n",
                                                       sensorEVT->fData[0],
                                                       sensorEVT->fData[1],
                                                       sensorEVT->fData[2]
                                                        );
            break;

        default:
            TRACE(0,"IDX[%d]: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", sensorEVT->sensorType,
                                                       sensorEVT->fData[0],
                                                       sensorEVT->fData[1],
                                                       sensorEVT->fData[2],
                                                       sensorEVT->fData[3],
                                                       sensorEVT->fData[4],
                                                       sensorEVT->fData[5],
                                                       sensorEVT->fData[6],
                                                       sensorEVT->fData[7]
                                                        );
    }
}
void CWM_OS_uSleep_test(uint32_t time)
{
    osDelay(time);
}
int CWM_OS_dbgOutput_test(const char *format)
{
    TRACE(0, "%s", format);
    /* rpc to app core to dump log */
    return 0;
}
uint64_t CWM_OS_GetTimeNs_test(void)
{
    uint64_t ret;
    static uint64_t pre_time = 0;
    static uint64_t overflow_cnt = 0;

    uint32_t lock = int_lock();
    uint64_t cur_time = (uint64_t)TICKS_TO_US(hal_sys_timer_get()) * 1000ULL;
	TRACE(0,"current time %d ",cur_time);
    // rt_kprintf("=========> GetTimeNs %lld \r\n", cur_time);
    if (cur_time < pre_time) {
        overflow_cnt++;
    }
    pre_time = cur_time;
    ret = cur_time + overflow_cnt * TICKS_TO_US(4294967296)*1000ULL;
    // rt_kprintf("=========> GetTimeNs %lld \r\n", ret);
    int_unlock(lock);
    return ret;
}


/***************************Thread Func**********************************************/
static void geomagnetism_thread_entry(void const *param);

osThreadDef(geomagnetism_thread_entry, osPriorityNormal, 1, (4 * 1024), "geomagnetism_task");

static void geomagnetism_thread_entry(void const *param)
{
	uint8_t sendbuf = 0,readbuf = 0;
	uint8_t samplingstat = 0;// 0 done ,1 sampling
	uint8_t readcnt = 0;
	
	geomagnetism_Enable();
	//osDelay(10);
//	SettingControl_t scl;
//	os_api api = {
//		.GetTimeNs    = CWM_OS_GetTimeNs_test,
//        .dbgOutput    = CWM_OS_dbgOutput_test,
//        .i2cRead      = CWM_OS_i2cRead,
//        .i2cWrite     = CWM_OS_i2cWrite,
//        .spiRead      = CWM_OS_spiRead,
//        .spiWrite     = CWM_OS_spiWrite,
//        .uSleep       = CWM_OS_uSleep_test,
//	};
//	char chipInfo[64];
//	memset(&scl, 0, sizeof(scl));
//
//	TRACE(0,"test thread entry");
//	scl.iData[0] = 1;
//	CWM_SettingControl(SCL_GET_LIB_INFO, &scl);
//	TRACE(0,"version:%d.%d.%d.%d product:%d model:%d\n", scl.iData[1], scl.iData[2], scl.iData[3], scl.iData[4], scl.iData[5], scl.iData[6]);
//
//	CWM_LibPreInit(&api);
//	//set mcu_chip information, must call this before CWM_LibPostInit()
//    memset(&scl, 0, sizeof(scl));
//    scl.iData[0] = 1;
//    scl.iData[1] = 11010100; // 0: mcu_auto_detect 2: skip_mcu_auto_detect
//	scl.iData[2] = 105713522;
//	scl.iData[3] = 0;
//	CWM_SettingControl(SCL_CHIP_VENDOR_CONFIG, &scl);
//	TRACE(0,"test thread entry Ponit1");
//
//	CWM_LibPostInit(CWM_AP_SensorListen_test);
//
//	TRACE(0,"test thread entry Ponit2");
//	memset(&scl, 0, sizeof(scl));
//	scl.iData[0] = 1;
//	scl.iData[1] = 1;
//	scl.iData[2] = (int)chipInfo;
//	scl.iData[3] = sizeof(chipInfo);
//	scl.iData[4] = 0;
//	scl.iData[5] = 0;
//	scl.iData[6] = 0;
//	CWM_SettingControl(SCL_GET_CHIP_INFO, &scl);
//	TRACE(0,"have_security = %d.%d ret_buff_size = %d	chipInfo = %s\n", scl.iData[5], scl.iData[6], scl.iData[4], chipInfo);
//	TRACE(0,"chip_settings = %d, %d, %d\n", scl.iData[9], scl.iData[10], scl.iData[11]);
//
//	CWM_Dml_LibInit();
//	
//	memset(&scl, 0, sizeof(scl));
//	scl.iData[0] = 1;
//    scl.iData[3] = 9;
//    scl.iData[4] = 5;
//    scl.iData[7] = -1;
//	CWM_SettingControl(SCL_LOG, &scl);
//	
//	memset(&scl, 0, sizeof(scl));
//    scl.iData[0] = 1;
//    scl.iData[3] = 2;
//    scl.iData[4] = 2;
//    scl.iData[8] = 52; // for st sensor(52Hz)
//    CWM_SettingControl(SCL_DML_DRV_AG_CONFIG, &scl);
//
//	memset(&scl, 0, sizeof(scl));
//    scl.iData[0] = 1;
//    scl.iData[1] = 1;
//    CWM_SettingControl(SCL_DML_DRV_INIT, &scl);
//
//	memset(&scl, 0, sizeof(scl));
//    scl.iData[0] = 1;
//    CWM_SettingControl(SCL_DML_GET_INITED_LIST, &scl);
//	
//	memset(&scl, 0, sizeof(scl));
//    scl.iData[0] = 1;
//    scl.iData[1] = 1;
//    scl.iData[2] = 3;
//    CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);
//
//	CWM_Sensor_Enable(CUSTOM_ACC);
//    CWM_Sensor_Enable(CUSTOM_GYRO);
	
    while(1) {
#ifndef GEOMAGN_DEBUG
        osSemaphoreWait(geomagnetism_sem_id, osWaitForever);
#endif
		//geomagnetism_CheckID();
		//geomagnetism_Saturation_Checking();
		switch(sampling_mode){
			case CONTINUOUS_SAMPLING:
				geomagnetism_GetData(&magnetic_field_now);
				break;
			case SINGLE_SAMPLING:
				samplingstat = 0;// 0 sampling ,1 done
				readcnt = 0;
				while(readcnt < 5 && samplingstat == 0)
				{
					geomagnetism_read_regs(MMC5603_REG_STATUS1, &readbuf, 1);
					if(readbuf & MMC5603_STAT_M_M_DONE)
						samplingstat = 1;
					readcnt++;
					osDelay(5);
				}
				geomagnetism_GetData(&magnetic_field_now);
				sendbuf = MMC5603_CMD_TMM;
				geomagnetism_write_regs(MMC5603_REG_CTRL0, &sendbuf, 1);	
				break;
		}

		//lib test part
		
//		CWM_Dml_process();
#ifdef GEOMAGN_DEBUG
		TRACE(0, " magnetic field now: x=%.4f y=%.4f z=%.4f  (mG)", magnetic_field_now.x*1000,magnetic_field_now.y*1000,magnetic_field_now.z*1000);
		osDelay(10);
#endif
    }
}



int geomagnetism_sensorhub_init(void)
{
	//geomagnetisim_sem_init();
	
	if (geomagnetism_thread_id == NULL) {
        geomagnetism_thread_id = osThreadCreate(osThread(geomagnetism_thread_entry), NULL);
        ASSERT(geomagnetism_thread_id, "create stk8328 thread fail!");
    }
}



