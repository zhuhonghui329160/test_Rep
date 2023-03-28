/*
 * Copyright (c) 2018-2022, Qiancheng Technology Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-27     hohn      the first version
 *
 */

#include "charger_aw32012.h"

#include "hal_i2c.h"
#include "app_utils.h"
#include "hal_iomux.h"

#define DBG_TAG              "charger.aw32012"
#define DBG_LVL               DBG_LOG
#include <rtdbg.h>
#define AW32012_I2C_ADDR             0x49

#define AW32012_INPUT_CTRL_REG			0x00
#define AW32012_POWERON_CONF_REG		0x01
#define AW32012_CHARGE_I_CTRL_REG		0x02
#define AW32012_DISCHARGE_I_CTRL_REG	0x03
#define AW32012_CHARGE_V_CTRL_REG		0x04
#define AW32012_CHARGE_TIM_CTRL_REG		0x05
#define AW32012_MAIN_CTRL_REG			0x06
#define AW32012_SYS_V_CTRL_REG			0x07
#define AW32012_SYS_STAT_REG			0x08
#define AW32012_FAULT_REG				0x09
#define AW32012_CHIPID_REG				0x0A
#define AW32012_INDIV_CHARGE_REG		0x0B
#define AW32012_ADD_FUNC_1_REG			0x0C
#define AW32012_ADD_FUNC_2_REG			0x0D
#define AW32012_ADD_FUNC_3_REG			0x22

struct i2c_dev {
    const char *name;
    enum HAL_I2C_ID_T bus;
    uint32_t addr;
    int speed;
};

static struct i2c_dev aw32012_dev = {
    .name     = "aw32012",
    .bus      = (enum HAL_I2C_ID_T )CHARGER_I2C_CHAN,//HAL_I2C_ID_0,//test//
    .addr     = AW32012_I2C_ADDR,
    .speed    = 400000,
};

static int32_t aw32012_i2c_init()
{
    int32_t ret = 0;

    struct HAL_I2C_CONFIG_T cfg = {0};
    cfg.mode      = HAL_I2C_API_MODE_SIMPLE;
    cfg.use_dma   = 1;
    cfg.use_sync  = 1;
    cfg.as_master = 1;

    cfg.speed = aw32012_dev.speed;
    ret = hal_i2c_open(aw32012_dev.bus, &cfg);
    if(ret != 0) {
        LOG_E("%s bus_id %d init err", __func__, aw32012_dev.bus);
        hal_i2c_close(aw32012_dev.bus);
    }
    LOG_I("%s bus_id %d init success", __func__, aw32012_dev.bus);
    return ret;
}

static uint32_t aw32012_i2c_write(uint32_t i2c_bus, uint32_t slave, uint8_t reg_addr, uint8_t *pd, uint32_t dlen)
{
    uint32_t ret;

    uint8_t *tx_buf = rt_malloc(dlen + 1);
    *(tx_buf + 0) = reg_addr;
    rt_memcpy(tx_buf + 1, pd, dlen);
    ret = hal_i2c_simple_send(i2c_bus, slave, tx_buf, dlen + 1);
    rt_free(tx_buf);

    if (ret) {
        LOG_E("%s, send failed, ret=%d", __func__, ret);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t aw32012_i2c_read(uint32_t i2c_bus, uint32_t slave, uint8_t *reg_addr, uint32_t reg_len, uint8_t *pd, uint32_t dlen)
{
    uint32_t ret;
    //hal_sysfreq_req(APP_SYSFREQ_USER_CHARGER, HAL_CMU_FREQ_26M);
    ret = hal_i2c_simple_recv(i2c_bus, slave, reg_addr, reg_len, pd, dlen);
    //hal_sysfreq_req(APP_SYSFREQ_USER_CHARGER, HAL_CMU_FREQ_32K);
    if (ret) {
        LOG_E("%s, read failed, ret=0x%x, dlen=%d",__func__, ret, dlen);
        return -RT_ERROR;
    }
    return RT_EOK;
}

rt_err_t charger_aw32012_init()
{
    rt_err_t ret;
    uint8_t data = 0;
    uint8_t regAddr;
////test 
//	static const struct HAL_IOMUX_PIN_FUNCTION_MAP pinmux[] = {
//		{HAL_IOMUX_PIN_P9_3,HAL_IOMUX_FUNC_GPIO,HAL_IOMUX_PIN_VOLTAGE_VIO,HAL_IOMUX_PIN_PULLUP_ENABLE},
//};
//		hal_iomux_init(pinmux, ARRAY_SIZE(pinmux));
//		hal_gpio_pin_set(HAL_IOMUX_PIN_P9_3);
//
//	hal_iomux_set_i2c0();
////test end
	aw32012_i2c_init();

	/* === tRST_DGL = 16s, tRST_DUR = 2s ,Q1 open,Q2 open,VBAT_UVLO = 3.03V === */
	regAddr = AW32012_POWERON_CONF_REG;
    data = 0x87;
    ret = aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_POWERON_CONF_REG write Err", __func__);
        return -RT_ERROR;
    }

	/* === REG_RST = 0, WD_TMR_RST = 0, CC = 304mA === */
	regAddr = AW32012_CHARGE_I_CTRL_REG;
    data = 0x25;
    ret = aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_CHARGE_I_CTRL_REG write Err", __func__);
        return -RT_ERROR;
    }

	/* === IDSCHG = 2000mA, ITERM = 15mA === */
	regAddr = AW32012_DISCHARGE_I_CTRL_REG;
    data = 0x97;
    ret = aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_DISCHARGE_I_CTRL_REG write Err", __func__);
        return -RT_ERROR;
    }

	/* === VBAT_REG(CV) = 4.455V, VBAT_PRE = 3.0V, VRECH = 200mV === */
	regAddr = AW32012_CHARGE_V_CTRL_REG;
    data = 0xE7;
    ret = aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_CHARGE_V_CTRL_REG write Err", __func__);
        return -RT_ERROR;
    }

	/* === EN_WD_DISCHG = 0, WATCHDOG = 0,EN_TERM = 1,EN_TIMER = 1,CHG_TMR = 5hours,TERM_TMR = 0=== */
	regAddr = AW32012_CHARGE_TIM_CTRL_REG;
    data = 0x1A;
    ret = aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_CHARGE_TIM_CTRL_REG write Err", __func__);
        return -RT_ERROR;
    }

	ulog_tag_lvl_filter_set(DBG_TAG, DBG_INFO);
    return RT_EOK;
}
MSH_CMD_EXPORT(charger_aw32012_init,charger_aw32012_init);


#define CHG_MODE_STR(param) #param
const char charge_mode_str[AW32012_WORKMODE_NUM][32] = {
    CHG_MODE_STR(AW32012_WORKMODE_DISCHARGING),
    CHG_MODE_STR(AW32012_WORKMODE_CHARGING),
    CHG_MODE_STR(AW32012_WORKMODE_ONLY_VUBUS),
};

#define CHG_STAT_STR(param) #param
const char charge_stat_str[AW32012_STAT_NUM][32] = {
    CHG_STAT_STR(AW32012_STAT_NOTCHARGING),
    CHG_STAT_STR(AW32012_STAT_PRECHARGING),
    CHG_STAT_STR(AW32012_STAT_FASTCHARGING),
    CHG_STAT_STR(AW32012_STAT_CHARGED_DONE),
};

rt_err_t aw32012_ChipID_check()
{
	rt_err_t ret;
	uint8_t regAddr;
	uint8_t data;

	regAddr = AW32012_CHIPID_REG;
	ret = aw32012_i2c_read(aw32012_dev.bus,AW32012_I2C_ADDR,&regAddr,1,&data, 1);
	if (ret != RT_EOK) {
        LOG_E("%s AW32012_CHIPID_REG read Err", __func__);
        return -RT_ERROR;
    }
	if(data != AW32012_I2C_ADDR){
		LOG_E("%s AW32012_CHIPID_REG read Err, data:0x%02x", __func__,data);
        return -RT_ERROR;
	}
	LOG_I("%s AW32012_CHIPID_REG read success, data:0x%02x", __func__,data);
	return ret;
}
MSH_CMD_EXPORT(aw32012_ChipID_check,aw32012_ChipID_check);

int8_t charger_aw32012_work_mode_get()
{
	uint8_t data = 0;
    uint8_t regAddr = AW32012_POWERON_CONF_REG;
    uint8_t mode;
    int8_t stat;
    
    aw32012_i2c_read(aw32012_dev.bus, AW32012_I2C_ADDR, &regAddr, 1, &data, 1);
    
    /* bit 2:1, === 00: discharging, 01: charging, 10: only, vbuspower 11: xx === */
    mode = (data & 0x18) >> 3;
    switch (mode) {
        case 0x00:
            stat = AW32012_WORKMODE_CHARGING;
            break;
        case 0x01:
            stat = AW32012_WORKMODE_ONLY_VUBUS;
            break;
        case 0x02:
            stat = AW32012_WORKMODE_DISCHARGING;
            break;
        default:
            stat = AW32012_STAT_ERR;
            break;
    }
    LOG_I("==== %s ====\n", charge_mode_str[stat]);
    return stat;
}
MSH_CMD_EXPORT(charger_aw32012_work_mode_get,charger_aw32012_work_mode_get);

int8_t charger_aw32012_chg_stat_get()
{
	uint8_t regAddr = AW32012_SYS_STAT_REG;
	uint8_t mode;
	uint8_t data;
	int8_t stat;

	aw32012_i2c_read(aw32012_dev.bus, AW32012_I2C_ADDR, &regAddr, 1, &data, 1);

    LOG_I("==== REG 0x%02X is 0x%02X ====\n", AW32012_SYS_STAT_REG, data);
    /* bit 2:1, === 00: discharging, 01: charging, 10: only, vbuspower 11: xx === */
    mode = (data & 0x18) >> 3;
    switch (mode) {
        case 0x00:
            stat = AW32012_STAT_NOTCHARGING;
            break;
        case 0x01:
            stat = AW32012_STAT_PRECHARGING;
            break;
        case 0x02:
            stat = AW32012_STAT_FASTCHARGING;
            break;
        case 0x03:
            stat = AW32012_STAT_CHARGED_DONE;
            break;
        default:
            stat = AW32012_STAT_ERR;
            break;
    }
    LOG_I("==== %s ====\n", charge_stat_str[stat]);
    return stat;
}
MSH_CMD_EXPORT(charger_aw32012_chg_stat_get,charger_aw32012_chg_stat_get);

rt_err_t charger_aw32012_enter_ship_mode()
{
	uint8_t data = 0;
    uint8_t regAddr = AW32012_MAIN_CTRL_REG;
    rt_err_t ret;
    ret = aw32012_i2c_read(aw32012_dev.bus, AW32012_I2C_ADDR, &regAddr, 1, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_MAIN_CTRL_REG read Err", __func__);
        return -RT_ERROR;
    }
    data |= 0x20;
    ret = aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);    
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_MAIN_CTRL_REG write Err", __func__);
        return -RT_ERROR;
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(charger_aw32012_enter_ship_mode,charger_aw32012_enter_ship_mode);


#include "hal_key.h"
void charger_aw32012_read_pwrkey()
{
    int stat = 0;
    stat = hal_key_read_status(HAL_KEY_CODE_PWR);
    LOG_I("====== pwrkey stat %s =====", (stat == HAL_KEY_EVENT_UP) ? "Press" : "Release");
}
MSH_CMD_EXPORT(charger_aw32012_read_pwrkey,charger_aw32012_read_pwrkey);

void charger_aw32012_wdt_test()
{
    uint8_t regAddr = AW32012_CHARGE_I_CTRL_REG;
    uint8_t data;
    int ret = aw32012_i2c_read(aw32012_dev.bus, AW32012_I2C_ADDR, &regAddr, 1, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_CHARGE_I_CTRL_REG read Err", __func__);
        return;
    }
    data |= 0x40;
    ret = aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
    if (ret != RT_EOK) {
        LOG_E("%s AW32012_CHARGE_I_CTRL_REG write Err", __func__);
        return;
    }
    LOG_I("=== %s ===", __func__);
}
MSH_CMD_EXPORT(charger_aw32012_wdt_test,charger_aw32012_wdt_test);

#include "stdlib.h"
void charger_aw32012_const_current_test(int argc, char **argv)
{
    uint8_t regAddr = AW32012_CHARGE_I_CTRL_REG;
    uint8_t data = atoi(argv[1]);
    LOG_I("==== set cc val %d =====\n", data);
	data /= 8;
	data &= 0x3F;//
    aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1); 
}
MSH_CMD_EXPORT(charger_aw32012_const_current_test,charger_aw32012_const_current_test);

void charger_aw32012_discharge_current_test(int argc, char **argv)
{
    uint8_t regAddr = AW32012_DISCHARGE_I_CTRL_REG;
    uint8_t val = atoi(argv[1]);
    uint8_t data = 0x97;
    if (val >= 0) {
        data = (val << 4) | 0x07;
        LOG_I("==== set discharge REG 0x%02x ====\n", data);
    }
    aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
}
MSH_CMD_EXPORT(charger_aw32012_discharge_current_test,charger_aw32012_discharge_current_test);

#ifdef BSP_USING_BATTERY
extern void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status);
#endif
void charger_aw32012_enable(void)
{
	uint8_t data;
    uint8_t regAddr = AW32012_POWERON_CONF_REG;
    aw32012_i2c_read(aw32012_dev.bus, AW32012_I2C_ADDR, &regAddr, 1, &data, 1);
    data &= ~(1 << 3);
    aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
#ifdef BSP_USING_BATTERY
	pmu_charger_set_irq_handler(app_battery_charger_handler);
#endif
    LOG_I("==== enable charge ====\n");
}
MSH_CMD_EXPORT(charger_aw32012_enable,charger_aw32012_enable);

void charger_aw32012_disable(void)
{
	uint8_t data;
    uint8_t regAddr = AW32012_POWERON_CONF_REG;
    aw32012_i2c_read(aw32012_dev.bus, AW32012_I2C_ADDR, &regAddr, 1, &data, 1);
    data |= (1 << 3);
    aw32012_i2c_write(aw32012_dev.bus, AW32012_I2C_ADDR, regAddr, &data, 1);
#ifdef BSP_USING_BATTERY
	pmu_charger_set_irq_handler(NULL);
#endif
    LOG_I("==== disable charge ====\n");
}
MSH_CMD_EXPORT(charger_aw32012_disable,charger_aw32012_disable);


