/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MQ135 Air Quality Sensor Driver for PSoC Edge E84
 *
 * MQ135 工作原理:
 *   MQ135 是一种金属氧化物半导体 (MOS) 气体传感器,
 *   传感器电导率随空气中目标气体浓度增加而增大。
 *   通过测量 Vout, 计算出 Rs (传感器电阻), 进而估算气体浓度。
 *
 * 计算公式:
 *   Vout = raw_adc * Vref / ADC_Resolution
 *   Rs   = RL * (Vref - Vout) / Vout
 *   ratio = Rs / R0
 *
 * 气体浓度 PPM 估算: PPM = a * (Rs/R0)^b
 *   (参数 a, b 根据 MQ135 数据手册拟合)
 *
 * 硬件连接:
 *   MQ135 VCC  -> 5V
 *   MQ135 GND  -> GND
 *   MQ135 AOUT -> PSoC Edge ADC 输入 (如 P8.4, ADC1_CH0)
 *   MQ135 DOUT -> (可选) GPIO 数字输出 (阈值触发)
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-06     user         first version
 */

#include "mq135.h"
#include <math.h>

/* ============================================================
 * 气体浓度计算曲线参数
 * 基于 MQ135 数据手册中的灵敏度特性曲线拟合
 * 公式: PPM = a * (Rs/R0)^b
 *
 * 注意: 这些参数需要根据实际标定环境调整
 * ============================================================ */
typedef struct
{
    float a;    /* 比例系数 */
    float b;    /* 指数系数 */
} mq135_gas_curve_t;

static const mq135_gas_curve_t gas_curves[MQ135_GAS_COUNT] =
{
    [MQ135_GAS_CO2]     = { .a = 110.0f,  .b = -2.70f  },    /* CO2 */
    [MQ135_GAS_CO]      = { .a = 650.0f,  .b = -3.10f  },    /* CO */
    [MQ135_GAS_NH4]     = { .a = 100.0f,  .b = -2.50f  },    /* NH4 */
    [MQ135_GAS_ALCOHOL] = { .a = 75.0f,   .b = -2.90f  },    /* Alcohol */
    [MQ135_GAS_BENZENE] = { .a = 60.0f,   .b = -3.30f  },    /* Benzene */
    [MQ135_GAS_SMOKE]   = { .a = 55.0f,   .b = -3.40f  },    /* Smoke */
};

/* ============================================================
 * 获取当前系统 tick (ms)
 * ============================================================ */
static rt_uint32_t mq135_get_tick_ms(void)
{
    return rt_tick_get() * (1000 / RT_TICK_PER_SECOND);
}

/* ============================================================
 * 滑动窗口滤波 - 添加新值
 * ============================================================ */
static void mq135_window_push(mq135_dev_t *dev, rt_uint32_t value)
{
    dev->adc_window[dev->window_index] = value;
    dev->window_index = (dev->window_index + 1) % MQ135_SAMPLE_WINDOW_SIZE;

    if (dev->window_count < MQ135_SAMPLE_WINDOW_SIZE)
    {
        dev->window_count++;
    }
}

/* ============================================================
 * 滑动窗口滤波 - 获取平均值
 * ============================================================ */
static rt_uint32_t mq135_window_average(mq135_dev_t *dev)
{
    if (dev->window_count == 0)
    {
        return 0;
    }

    rt_uint32_t sum = 0;
    for (rt_uint8_t i = 0; i < dev->window_count; i++)
    {
        sum += dev->adc_window[i];
    }

    return sum / dev->window_count;
}

/* ============================================================
 * 初始化 MQ135 传感器
 * ============================================================ */
rt_err_t mq135_init(mq135_dev_t *dev, const char *adc_name, rt_uint32_t channel)
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(adc_name != RT_NULL);

    /* 清零设备结构体 */
    rt_memset(dev, 0, sizeof(mq135_dev_t));

    /* 配置参数 */
    dev->adc_channel    = channel;
    dev->vref           = MQ135_VREF;
    dev->adc_resolution = MQ135_ADC_RESOLUTION;
    dev->rl_value       = MQ135_RL_VALUE;
    dev->r0             = (float)MQ135_R0_DEFAULT;

    /* 查找并打开 ADC 设备 */
    dev->adc_dev = (rt_adc_device_t)rt_device_find(adc_name);
    if (dev->adc_dev == RT_NULL)
    {
        rt_kprintf("[MQ135] Error: cannot find ADC device '%s'\n", adc_name);
        return -RT_ERROR;
    }

    /* 使能 ADC 通道 */
    rt_err_t ret = rt_adc_enable(dev->adc_dev, dev->adc_channel);
    if (ret != RT_EOK)
    {
        rt_kprintf("[MQ135] Error: cannot enable ADC channel %d\n", channel);
        return ret;
    }

    /* 记录预热开始时间 */
    dev->preheat_start_tick = mq135_get_tick_ms();
    dev->is_preheated = RT_FALSE;

    rt_kprintf("[MQ135] Init OK. ADC=%s CH=%d, preheating...\n", adc_name, channel);

    return RT_EOK;
}

/* ============================================================
 * 检查预热状态
 * ============================================================ */
rt_bool_t mq135_is_preheated(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);

    if (dev->is_preheated)
    {
        return RT_TRUE;
    }

    rt_uint32_t elapsed = mq135_get_tick_ms() - dev->preheat_start_tick;
    if (elapsed >= MQ135_PREHEAT_TIME_MS)
    {
        dev->is_preheated = RT_TRUE;
        rt_kprintf("[MQ135] Preheating complete (%d ms)\n", elapsed);
        return RT_TRUE;
    }

    return RT_FALSE;
}

/* ============================================================
 * 读取原始 ADC 值并更新所有计算值
 * ============================================================ */
rt_err_t mq135_read(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);

    if (dev->adc_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* 读取 ADC 原始值 */
    rt_uint32_t raw = rt_adc_read(dev->adc_dev, dev->adc_channel);

    /* 滑动窗口滤波 */
    mq135_window_push(dev, raw);
    dev->raw_adc = mq135_window_average(dev);

    /* 计算电压 (mV -> V) */
    dev->voltage = (float)(dev->raw_adc) * (float)(dev->vref)
                   / (float)(dev->adc_resolution) / 1000.0f;

    /* 计算传感器电阻 Rs
     * Rs = RL * (Vref_mV - Vout_mV) / Vout_mV
     * 注意: 当 Vout 很小时需要特殊处理, 防止除以零
     */
    float vout_mv = dev->voltage * 1000.0f;   /* 转换为 mV */
    if (vout_mv < 1.0f)
    {
        vout_mv = 1.0f;   /* 最小电压限制 */
    }
    if (vout_mv >= (float)(dev->vref))
    {
        vout_mv = (float)(dev->vref) - 1.0f;  /* 防止分母为零或负数 */
    }

    dev->rs = (float)(dev->rl_value) * ((float)(dev->vref) - vout_mv) / vout_mv;

    /* 计算 Rs/R0 比值 */
    if (dev->r0 > 0.0f)
    {
        dev->rs_ratio = dev->rs / dev->r0;
    }
    else
    {
        dev->rs_ratio = 1.0f;
    }

    /* 计算 CO2 等效 PPM */
    dev->co2_ppm = mq135_get_gas_ppm(dev, MQ135_GAS_CO2);

    /* 更新空气质量等级 */
    dev->quality = mq135_get_air_quality(dev);

    return RT_EOK;
}

/* ============================================================
 * 获取传感器电阻 Rs
 * ============================================================ */
float mq135_get_resistance(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);
    return dev->rs;
}

/* ============================================================
 * 获取 Rs/R0 比值
 * ============================================================ */
float mq135_get_ratio(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);
    return dev->rs_ratio;
}

/* ============================================================
 * 获取指定气体浓度 (PPM)
 * PPM = a * (Rs/R0)^b
 * ============================================================ */
float mq135_get_gas_ppm(mq135_dev_t *dev, mq135_gas_type_t gas_type)
{
    RT_ASSERT(dev != RT_NULL);

    if (gas_type >= MQ135_GAS_COUNT)
    {
        return 0.0f;
    }

    const mq135_gas_curve_t *curve = &gas_curves[gas_type];
    float ppm = curve->a * powf(dev->rs_ratio, curve->b);

    return ppm;
}

/* ============================================================
 * 获取 CO2 等效浓度
 * ============================================================ */
float mq135_get_co2_ppm(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);
    return dev->co2_ppm;
}

/* ============================================================
 * 获取当前空气质量等级
 * 基于 CO2 等效浓度判定:
 *   优:    0 ~ 50   PPM
 *   良:   50 ~ 100  PPM
 *   轻度: 100 ~ 200 PPM
 *   中度: 200 ~ 400 PPM
 *   重度: > 400    PPM
 *
 * 同时考虑 Rs/R0 比值:
 *   MQ135 在洁净空气中 Rs/R0 ≈ 3.6
 *   当 Rs/R0 显著下降时, 说明检测到较多气体
 * ============================================================ */
mq135_air_quality_t mq135_get_air_quality(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);

    float ppm = dev->co2_ppm;

    /* 优先使用 CO2 PPM 判定 */
    if (ppm <= 50.0f)
    {
        return MQ135_AIR_QUALITY_GOOD;
    }
    else if (ppm <= 100.0f)
    {
        return MQ135_AIR_QUALITY_MODERATE;
    }
    else if (ppm <= 200.0f)
    {
        return MQ135_AIR_QUALITY_LIGHT_POLLUTE;
    }
    else if (ppm <= 400.0f)
    {
        return MQ135_AIR_QUALITY_MID_POLLUTE;
    }
    else
    {
        return MQ135_AIR_QUALITY_HEAVY_POLLUTE;
    }
}

/* ============================================================
 * 获取空气质量等级的中文描述
 * ============================================================ */
const char *mq135_quality_to_string(mq135_air_quality_t quality)
{
    switch (quality)
    {
    case MQ135_AIR_QUALITY_GOOD:
        return "优";
    case MQ135_AIR_QUALITY_MODERATE:
        return "良";
    case MQ135_AIR_QUALITY_LIGHT_POLLUTE:
        return "轻度污染";
    case MQ135_AIR_QUALITY_MID_POLLUTE:
        return "中度污染";
    case MQ135_AIR_QUALITY_HEAVY_POLLUTE:
        return "重度污染";
    default:
        return "未知";
    }
}

/* ============================================================
 * 判断当前空气质量是否异常
 * 轻度及以上污染视为异常
 * ============================================================ */
rt_bool_t mq135_is_air_abnormal(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);

    mq135_air_quality_t q = mq135_get_air_quality(dev);
    return (q >= MQ135_AIR_QUALITY_LIGHT_POLLUTE) ? RT_TRUE : RT_FALSE;
}

/* ============================================================
 * 标定 R0 值
 * 在洁净空气中调用此函数, 自动计算 R0
 * MQ135 datasheet: 洁净空气中 Rs/R0 ≈ 3.6
 *
 * 标定步骤:
 *   1. 将传感器放置在洁净空气中 (室外或通风良好处)
 *   2. 预热至少 60 秒
 *   3. 调用此函数
 *   4. R0 = Rs / 3.6
 * ============================================================ */
rt_err_t mq135_calibrate(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);

    /* 多次采样取平均 */
    float rs_sum = 0.0f;
    rt_uint32_t sample_count = 20;

    rt_kprintf("[MQ135] Calibrating R0... (sampling %d times)\n", sample_count);

    for (rt_uint32_t i = 0; i < sample_count; i++)
    {
        mq135_read(dev);
        rs_sum += dev->rs;
        rt_thread_mdelay(100);
    }

    float rs_avg = rs_sum / (float)sample_count;

    /* 洁净空气中 Rs/R0 ≈ 3.6 */
    dev->r0 = rs_avg / 3.6f;

    rt_kprintf("[MQ135] Calibration done.\n");
    rt_kprintf("[MQ135]   Rs_avg = %.2f Ω\n", rs_avg);
    rt_kprintf("[MQ135]   R0     = %.2f Ω\n", dev->r0);
    rt_kprintf("[MQ135]   (R0 stored, use mq135_get_ratio() to verify)\n");

    return RT_EOK;
}

/* ============================================================
 * 设置 R0 值
 * ============================================================ */
void mq135_set_r0(mq135_dev_t *dev, float r0)
{
    RT_ASSERT(dev != RT_NULL);
    if (r0 > 0.0f)
    {
        dev->r0 = r0;
    }
}

/* ============================================================
 * 打印传感器调试信息
 * ============================================================ */
void mq135_dump(mq135_dev_t *dev)
{
    RT_ASSERT(dev != RT_NULL);

    rt_kprintf("========================================\n");
    rt_kprintf("  MQ135 Air Quality Sensor Status\n");
    rt_kprintf("========================================\n");
    rt_kprintf("  Raw ADC    : %d\n", dev->raw_adc);
    rt_kprintf("  Voltage    : %.3f V\n", dev->voltage);
    rt_kprintf("  Rs         : %.2f Ω\n", dev->rs);
    rt_kprintf("  R0         : %.2f Ω\n", dev->r0);
    rt_kprintf("  Rs/R0      : %.4f\n", dev->rs_ratio);
    rt_kprintf("  CO2 PPM    : %.2f\n", dev->co2_ppm);
    rt_kprintf("  Quality    : %s\n", mq135_quality_to_string(dev->quality));
    rt_kprintf("  Preheated  : %s\n", dev->is_preheated ? "YES" : "NO");
    rt_kprintf("  Abnormal   : %s\n", mq135_is_air_abnormal(dev) ? "YES!" : "Normal");
    rt_kprintf("========================================\n");
}

#ifdef RT_USING_FINSH
#include <finsh.h>

/* 供 MSH Shell 调用的测试函数 */
static void mq135_test(int argc, char **argv)
{
    static mq135_dev_t mq135;
    static rt_bool_t initialized = RT_FALSE;

    if (!initialized)
    {
        mq135_init(&mq135, MQ135_ADC_DEV_NAME, MQ135_ADC_CHANNEL);
        initialized = RT_TRUE;
    }

    if (argc >= 2 && rt_strcmp(argv[1], "cal") == 0)
    {
        mq135_calibrate(&mq135);
    }
    else if (argc >= 2 && rt_strcmp(argv[1], "dump") == 0)
    {
        mq135_read(&mq135);
        mq135_dump(&mq135);
    }
    else
    {
        mq135_read(&mq135);
        rt_kprintf("[MQ135] CO2: %.2f PPM | Quality: %s | %s\n",
                   mq135.co2_ppm,
                   mq135_quality_to_string(mq135.quality),
                   mq135_is_air_abnormal(&mq135) ? "ABNORMAL!" : "Normal");
    }
}
MSH_CMD_EXPORT(mq135_test, MQ135 sensor test: mq135_test [cal|dump]);
#endif /* RT_USING_FINSH */
