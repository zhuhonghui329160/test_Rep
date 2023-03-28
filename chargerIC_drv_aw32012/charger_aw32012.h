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

#ifndef __CHARGER_AW32012_H__
#define __CHARGER_AW32012_H__

#include <stdint.h>
#include "rtthread.h"
#include "rtdef.h"

#ifdef __cplusplus
extern "C"
{
#endif


#if defined(BSP_USING_QC_RH304)
#define CHARGER_I2C_CHAN       HAL_I2C_ID_1
#endif

#if defined(BSP_USING_QC_EVB_V11)
#define CHARGER_I2C_CHAN       HAL_I2C_ID_1
#endif

#if defined(BSP_USING_QC_BOLT_EVB_V10)
#define CHARGER_I2C_CHAN       HAL_I2C_ID_1
#endif

#ifndef CHARGER_I2C_CHAN
#define CHARGER_I2C_CHAN       HAL_I2C_ID_3
#endif

typedef enum {
    AW32012_WORKMODE_ERR = -1,
    AW32012_WORKMODE_DISCHARGING,
    AW32012_WORKMODE_CHARGING,
    AW32012_WORKMODE_ONLY_VUBUS,
    AW32012_WORKMODE_NUM,
} AW32012_WORKMODE_T;

typedef enum {
    AW32012_STAT_ERR = -1,
    AW32012_STAT_NOTCHARGING,
    AW32012_STAT_PRECHARGING,
    AW32012_STAT_FASTCHARGING,
    AW32012_STAT_CHARGED_DONE,
    AW32012_STAT_NUM,
} AW32012_STAT_T;

rt_err_t charger_aw32012_init();
int8_t charger_aw32012_work_mode_get();
int8_t charger_aw32012_chg_stat_get();
rt_err_t charger_aw32012_enter_ship_mode();
rt_err_t charger_aw32012_const_current();
void charger_aw32012_enable(void);
void charger_aw32012_disable(void);

#ifdef __cplusplus
}
#endif

//#endif //BSP_USING_CHARGER_AW32012

#endif // __CHARGER_AW32012_H__

