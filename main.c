/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MQ135 Gas Sensor Detection - Main Application
 * for PSoC Edge E84 (Edgi-Talk) + RT-Thread
 *
 * 功能:
 *   1. MQ135 传感器初始化与预热
 *   2. 定时采样空气质量 (CO2, 各类气体浓度)
 *   3. 空气质量异常 LED 告警
 *   4. MSH Shell 命令进行交互式查询和标定
 *   5. JSON 格式输出, 供 Web/App 端解析
 *
 * 硬件连接:
 *   MQ135 AOUT -> P8.4 (ADC1 Channel 0)
 *   LED        -> P16.5 (板载蓝色 LED, 告警指示)
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-08     user         created for Edgi_Talk_MQ135 project
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "mq135.h"

/* ============================================================
 * 硬件引脚配置 (PSoC Edge E84 底板 J12/J13 排针)
 * ============================================================ */
#define LED_STATUS_PIN          GET_PIN(16, 5)    /* 板载蓝色 LED */

/* ============================================================
 * 全局变量
 * ============================================================ */
static mq135_dev_t g_mq135;                             /* MQ135 传感器实例 */
static rt_bool_t   g_air_abnormal = RT_FALSE;           /* 空气质量异常标志 */
static rt_bool_t   g_monitor_running = RT_TRUE;         /* 监测线程运行标志 */
static rt_thread_t g_monitor_thread = RT_NULL;

/* ============================================================
 * LED 控制
 * ============================================================ */

/* LED 心跳闪烁 (正常运行) */
static void led_heartbeat(void)
{
    static rt_uint8_t tick = 0;
    tick++;

    if (tick % 2 == 0)
        rt_pin_write(LED_STATUS_PIN, PIN_HIGH);
    else
        rt_pin_write(LED_STATUS_PIN, PIN_LOW);
}

/* LED 快速闪烁 (异常告警) */
static void led_alert(void)
{
    for (int i = 0; i < 6; i++)
    {
        rt_pin_write(LED_STATUS_PIN, PIN_HIGH);
        rt_thread_mdelay(100);
        rt_pin_write(LED_STATUS_PIN, PIN_LOW);
        rt_thread_mdelay(100);
    }
}

/* ============================================================
 * 空气质量监测线程
 * ============================================================ */
static void air_monitor_thread_entry(void *parameter)
{
    rt_uint32_t loop_count = 0;
    rt_bool_t last_abnormal = RT_FALSE;

    /* 等待预热完成 */
    rt_kprintf("[AirMonitor] Waiting for MQ135 preheat (60s)...\n");
    while (!mq135_is_preheated(&g_mq135))
    {
        rt_thread_mdelay(1000);
    }

    rt_kprintf("[AirMonitor] Preheated. Start monitoring...\n");

    while (g_monitor_running)
    {
        /* 读取传感器 */
        mq135_read(&g_mq135);

        /* LED 心跳 */
        led_heartbeat();

        /* 每 5 次循环打印一次详细状态 */
        if (++loop_count % 5 == 0)
        {
            rt_kprintf("\n[AirMonitor] ==================================\n");
            rt_kprintf("[AirMonitor] ADC=%d, V=%.3fV, Rs=%.0fΩ\n",
                       g_mq135.raw_adc, g_mq135.voltage, g_mq135.rs);
            rt_kprintf("[AirMonitor] CO2=%.1f PPM, Quality=%s\n",
                       g_mq135.co2_ppm,
                       mq135_quality_to_string(g_mq135.quality));
            rt_kprintf("[AirMonitor] ==================================\n\n");
        }

        /* 检测空气质量变化 */
        rt_bool_t is_abnormal = mq135_is_air_abnormal(&g_mq135);

        if (is_abnormal && !last_abnormal)
        {
            /* 从正常变为异常 */
            rt_kprintf("[AirMonitor] *** AIR QUALITY ABNORMAL! ***\n");
            g_air_abnormal = RT_TRUE;
            led_alert();
        }
        else if (!is_abnormal && last_abnormal)
        {
            /* 从异常恢复为正常 */
            rt_kprintf("[AirMonitor] Air quality back to normal.\n");
            g_air_abnormal = RT_FALSE;
        }
        else if (is_abnormal && last_abnormal)
        {
            /* 持续异常, LED 闪烁告警 */
            led_alert();
        }

        last_abnormal = is_abnormal;

        /* 采样间隔 */
        rt_thread_mdelay(MQ135_SAMPLE_INTERVAL_MS);
    }
}

/* ============================================================
 * 初始化所有 GPIO 外设
 * ============================================================ */
static void gpio_init(void)
{
    /* 状态 LED */
    rt_pin_mode(LED_STATUS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED_STATUS_PIN, PIN_LOW);

    rt_kprintf("[GPIO] LED pin initialized (P16.5).\n");
}

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void)
{
    rt_kprintf("\n");
    rt_kprintf("========================================\n");
    rt_kprintf("  PSoC Edge E84 - MQ135 Gas Sensor\n");
    rt_kprintf("  Air Quality Detection System\n");
    rt_kprintf("========================================\n");
    rt_kprintf("  Core: Cortex-M33\n");
    rt_kprintf("  RT-Thread Version: %s\n", RT_VERSION);
    rt_kprintf("========================================\n\n");

    /* ---- 1. 初始化 GPIO ---- */
    gpio_init();

    /* ---- 2. 初始化 MQ135 传感器 ---- */
    rt_kprintf("[Main] Initializing MQ135 sensor...\n");
    if (mq135_init(&g_mq135, MQ135_ADC_DEV_NAME, MQ135_ADC_CHANNEL) != RT_EOK)
    {
        rt_kprintf("[Main] ERROR: MQ135 init failed!\n");
        rt_kprintf("[Main] Check: ADC device '%s', channel %d\n",
                   MQ135_ADC_DEV_NAME, MQ135_ADC_CHANNEL);
    }
    else
    {
        rt_kprintf("[Main] MQ135 sensor initialized. ADC=%s CH=%d\n",
                   MQ135_ADC_DEV_NAME, MQ135_ADC_CHANNEL);
    }

    /* ---- 3. 创建空气质量监测线程 ---- */
    g_monitor_thread = rt_thread_create(
        "air_mon",
        air_monitor_thread_entry,
        RT_NULL,
        2048,               /* 栈大小 */
        15,                 /* 优先级 */
        20                  /* 时间片 */
    );

    if (g_monitor_thread != RT_NULL)
    {
        rt_thread_startup(g_monitor_thread);
        rt_kprintf("[Main] Air monitor thread created.\n");
    }
    else
    {
        rt_kprintf("[Main] ERROR: Cannot create air monitor thread!\n");
    }

    rt_kprintf("\n[Main] System initialization complete.\n");
    rt_kprintf("[Main] Commands: air_status, air_calibrate, mq135_test\n\n");

    return 0;
}

/* ============================================================
 * MSH Shell 扩展命令
 * ============================================================ */

#ifdef RT_USING_FINSH
#include <finsh.h>

/**
 * @brief   手动读取 MQ135 并打印状态
 */
static void air_status(int argc, char **argv)
{
    mq135_read(&g_mq135);
    mq135_dump(&g_mq135);
}
MSH_CMD_EXPORT(air_status, Show MQ135 air quality status);

/**
 * @brief   获取空气质量 (JSON 格式输出, 供 Web/App 解析)
 */
static void air_json(int argc, char **argv)
{
    mq135_read(&g_mq135);
    rt_kprintf("{");
    rt_kprintf("\"sensor\":\"MQ135\",");
    rt_kprintf("\"adc\":%d,", g_mq135.raw_adc);
    rt_kprintf("\"voltage\":%.3f,", g_mq135.voltage);
    rt_kprintf("\"co2_ppm\":%.1f,", g_mq135.co2_ppm);
    rt_kprintf("\"ratio\":%.4f,", g_mq135.rs_ratio);
    rt_kprintf("\"quality\":\"%s\",", mq135_quality_to_string(g_mq135.quality));
    rt_kprintf("\"abnormal\":%s", g_air_abnormal ? "true" : "false");
    rt_kprintf("}\n");
}
MSH_CMD_EXPORT(air_json, Get air quality data in JSON format);

/**
 * @brief   标定 MQ135 传感器
 * @note    请在洁净空气中运行此命令, 传感器需预热 >60s
 */
static void air_calibrate(int argc, char **argv)
{
    rt_kprintf("Calibrating MQ135 in clean air...\n");
    rt_kprintf("Make sure sensor is in CLEAN AIR and PREHEATED (>60s)!\n");
    rt_thread_mdelay(2000);
    mq135_calibrate(&g_mq135);
}
MSH_CMD_EXPORT(air_calibrate, Calibrate MQ135 R0 in clean air);

#endif /* RT_USING_FINSH */
