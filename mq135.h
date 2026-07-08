/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MQ135 Air Quality Sensor Driver for PSoC Edge E84
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-06     user         first version
 */

#ifndef __MQ135_H__
#define __MQ135_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * MQ135 引脚配置 (根据实际接线修改)
 * ============================================================
 * MQ135 模拟输出连接到 PSoC Edge E84 的 ADC 引脚
 * 底板扩展接口参考:
 *   - ADC1 Channel 0: P8.4 (J12 排针)
 *   - 也可使用其他 ADC 通道, 在 rtconfig.h 中开启对应 ADC
 */
#ifndef MQ135_ADC_DEV_NAME
#define MQ135_ADC_DEV_NAME          "adc1"      /* ADC 设备名称 */
#endif

#ifndef MQ135_ADC_CHANNEL
#define MQ135_ADC_CHANNEL           0           /* ADC 通道号 */
#endif

#ifndef MQ135_VREF
#define MQ135_VREF                  3300        /* 参考电压 (mV), PSoC Edge 使用 3.3V */
#endif

#ifndef MQ135_ADC_RESOLUTION
#define MQ135_ADC_RESOLUTION        4096        /* ADC 分辨率 (12位 = 4096) */
#endif

/* ============================================================
 * MQ135 预热与采样配置
 * ============================================================ */
#define MQ135_PREHEAT_TIME_MS       60000       /* 预热时间 60 秒 (首次使用建议 >24h) */
#define MQ135_SAMPLE_INTERVAL_MS    1000        /* 采样间隔 1 秒 */
#define MQ135_SAMPLE_WINDOW_SIZE    10          /* 滑动窗口大小, 用于滤波 */

/* ============================================================
 * MQ135 传感器标定参数 (需根据实际环境标定)
 * ============================================================
 * R0: 传感器在洁净空气中的电阻值
 * 通过测量洁净空气中的 ADC 值计算得出:
 *   R0 = Rs / 3.6  (MQ135 datasheet: Rs/R0 ≈ 3.6 @ clean air)
 *
 * 以下为默认参考值, 使用时请根据实际环境重新标定
 */
#ifndef MQ135_R0_DEFAULT
#define MQ135_R0_DEFAULT            760         /* 默认洁净空气 R0 值 (需标定) */
#endif

/* ============================================================
 * 负载电阻 RL (硬件分压电阻)
 * ============================================================
 * MQ135 模块通常自带 1KΩ 负载电阻
 * 计算公式: Rs = RL * (Vref - Vout) / Vout
 */
#ifndef MQ135_RL_VALUE
#define MQ135_RL_VALUE              1000        /* 负载电阻 1KΩ, 单位为欧姆 */
#endif

/* ============================================================
 * 气体类型枚举
 * ============================================================ */
typedef enum
{
    MQ135_GAS_CO2       = 0,    /* 二氧化碳 */
    MQ135_GAS_CO        = 1,    /* 一氧化碳 */
    MQ135_GAS_NH4       = 2,    /* 氨气 */
    MQ135_GAS_ALCOHOL   = 3,    /* 酒精 */
    MQ135_GAS_BENZENE   = 4,    /* 苯 */
    MQ135_GAS_SMOKE     = 5,    /* 烟雾 */
    MQ135_GAS_COUNT
} mq135_gas_type_t;

/* ============================================================
 * 空气质量等级
 * ============================================================ */
typedef enum
{
    MQ135_AIR_QUALITY_GOOD          = 0,    /* 优:   0  ~ 50   PPM */
    MQ135_AIR_QUALITY_MODERATE      = 1,    /* 良:   50 ~ 100  PPM */
    MQ135_AIR_QUALITY_LIGHT_POLLUTE = 2,    /* 轻度: 100 ~ 200 PPM */
    MQ135_AIR_QUALITY_MID_POLLUTE   = 3,    /* 中度: 200 ~ 400 PPM */
    MQ135_AIR_QUALITY_HEAVY_POLLUTE = 4,    /* 重度: > 400    PPM */
} mq135_air_quality_t;

/* ============================================================
 * MQ135 传感器数据结构
 * ============================================================ */
typedef struct
{
    rt_adc_device_t adc_dev;                  /* ADC 设备句柄 */
    rt_uint32_t     adc_channel;              /* ADC 通道 */
    rt_uint32_t     vref;                     /* 参考电压 (mV) */
    rt_uint32_t     adc_resolution;           /* ADC 分辨率 */
    rt_uint32_t     rl_value;                 /* 负载电阻值 (Ω) */
    float           r0;                       /* 洁净空气 R0 值 */

    /* 运行时数据 */
    rt_uint32_t     raw_adc;                  /* 当前原始 ADC 值 */
    float           voltage;                  /* 当前电压 (V) */
    float           rs;                       /* 传感器电阻 Rs (Ω) */
    float           rs_ratio;                 /* Rs/R0 比值 */
    float           co2_ppm;                  /* CO2 浓度 (PPM, 估算) */
    mq135_air_quality_t quality;              /* 当前空气质量等级 */

    /* 滑动窗口滤波 */
    rt_uint32_t     adc_window[MQ135_SAMPLE_WINDOW_SIZE];
    rt_uint8_t      window_index;
    rt_uint8_t      window_count;

    /* 状态 */
    rt_bool_t       is_preheated;             /* 是否已完成预热 */
    rt_uint32_t     preheat_start_tick;       /* 预热开始时间 */
} mq135_dev_t;

/* ============================================================
 * 公共 API
 * ============================================================ */

/**
 * @brief   初始化 MQ135 传感器
 * @param   dev         传感器设备结构体指针
 * @param   adc_name    ADC 设备名称 (如 "adc1")
 * @param   channel     ADC 通道号
 * @return  RT_EOK 成功, 其他值表示失败
 */
rt_err_t mq135_init(mq135_dev_t *dev, const char *adc_name, rt_uint32_t channel);

/**
 * @brief   标定 MQ135 传感器 (在洁净空气中执行)
 * @param   dev         传感器设备结构体指针
 * @return  RT_EOK 成功
 * @note    调用此函数前请确保传感器已在洁净空气中预热至少 60 秒
 */
rt_err_t mq135_calibrate(mq135_dev_t *dev);

/**
 * @brief   读取原始 ADC 值并更新所有计算值
 * @param   dev         传感器设备结构体指针
 * @return  RT_EOK 成功
 * @note    此函数会更新 dev 结构体中的所有运行时数据
 */
rt_err_t mq135_read(mq135_dev_t *dev);

/**
 * @brief   获取传感器电阻 Rs
 * @param   dev         传感器设备结构体指针
 * @return  传感器电阻值 (Ω)
 */
float mq135_get_resistance(mq135_dev_t *dev);

/**
 * @brief   获取 Rs/R0 比值
 * @param   dev         传感器设备结构体指针
 * @return  Rs/R0 比值
 */
float mq135_get_ratio(mq135_dev_t *dev);

/**
 * @brief   获取指定气体的估算浓度
 * @param   dev         传感器设备结构体指针
 * @param   gas_type    气体类型
 * @return  气体浓度 (PPM, 估算值)
 * @note    MQ135 对多种气体敏感, 此值为基于 Rs/R0 的经验曲线估算
 *          实际应用中建议针对特定气体进行标定
 */
float mq135_get_gas_ppm(mq135_dev_t *dev, mq135_gas_type_t gas_type);

/**
 * @brief   获取 CO2 等效浓度
 * @param   dev         传感器设备结构体指针
 * @return  CO2 浓度 (PPM, 估算)
 */
float mq135_get_co2_ppm(mq135_dev_t *dev);

/**
 * @brief   获取当前空气质量等级
 * @param   dev         传感器设备结构体指针
 * @return  空气质量等级
 */
mq135_air_quality_t mq135_get_air_quality(mq135_dev_t *dev);

/**
 * @brief   获取空气质量等级的中文描述
 * @param   quality     空气质量等级
 * @return  中文描述字符串
 */
const char *mq135_quality_to_string(mq135_air_quality_t quality);

/**
 * @brief   检查传感器是否预热完成
 * @param   dev         传感器设备结构体指针
 * @return  RT_TRUE 已预热, RT_FALSE 未完成
 */
rt_bool_t mq135_is_preheated(mq135_dev_t *dev);

/**
 * @brief   判断当前空气质量是否异常
 * @param   dev         传感器设备结构体指针
 * @return  RT_TRUE 异常 (轻度及以上污染), RT_FALSE 正常
 */
rt_bool_t mq135_is_air_abnormal(mq135_dev_t *dev);

/**
 * @brief   设置 R0 值 (用于已标定的传感器)
 * @param   dev         传感器设备结构体指针
 * @param   r0          洁净空气中的 R0 值
 */
void mq135_set_r0(mq135_dev_t *dev, float r0);

#ifdef __cplusplus
}
#endif

#endif /* __MQ135_H__ */
