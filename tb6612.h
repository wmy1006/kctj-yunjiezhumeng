/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TB6612FNG 双路电机驱动模块 for PSoC Edge E84
 *
 * TB6612FNG 是东芝半导体推出的双路 H 桥电机驱动芯片,
 * 可同时驱动两个直流电机或一个双极性步进电机。
 *
 * 特性:
 *   - 电源电压: VM = 2.5V ~ 13.5V (推荐 5V~12V)
 *   - 控制电压: VCC = 2.7V ~ 5.5V
 *   - 每通道持续电流: 1.2A (峰值 3.2A)
 *   - 支持: 正转 / 反转 / 刹车 / 停止
 *
 * TB6612 控制逻辑 (单通道):
 *   IN1=0, IN2=0  → 停止 (Coast)
 *   IN1=1, IN2=0  → 正转 (CW/Forward)
 *   IN1=0, IN2=1  → 反转 (CCW/Backward)
 *   IN1=1, IN2=1  → 刹车 (Short Brake)
 *   PWM 控制: 通过 PWM 引脚输入 PWM 信号调节转速 (0%~100%)
 *   STBY=1       → 正常工作
 *   STBY=0       → 待机 (电机停转)
 *
 * 硬件连接参考 (PSoC Edge E84 底板扩展接口):
 *   使用 J12/J13 排针上的 GPIO 和 PWM 引脚
 *   电源接线:
 *     TB6612 VM   -> 电池 7.4V~12V (电机电源)
 *     TB6612 VCC  -> 3.3V (逻辑电源, PSoC Edge)
 *     TB6612 GND  -> 共地
 *   控制接线 (请根据实际接线修改以下宏定义):
 *     STBY -> P16.4
 *     AIN1 -> P16.6, AIN2 -> P16.7, PWMA -> PWM 输出
 *     BIN1 -> P17.0, BIN2 -> P17.1, PWMB -> PWM 输出
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-06     user         first version
 */

#ifndef __TB6612_H__
#define __TB6612_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * TB6612 引脚配置 (根据实际接线修改)
 * ============================================================
 * PSoC Edge E84 底板扩展排针:
 *   J12 排针: P8.0 ~ P8.7, P16.0 ~ P16.7
 *   J13 排针: P17.0 ~ P17.7, P18.0 ~ P18.7
 *
 * PWM 说明:
 *   - 如果使用硬件 PWM, 需要先在 rtconfig.h 中启用对应 PWM 通道
 *     (如 BSP_USING_PWM18), 并在 board.c 中配置
 *   - 如果使用软件模拟 PWM (通过 GPIO + 定时器线程),
 *     则不需要硬件 PWM 支持
 */
/* ---- 待机引脚 ---- */
#ifndef TB6612_STBY_PIN
#define TB6612_STBY_PIN             GET_PIN(16, 4)  /* STBY: P16.4 */
#endif

/* ---- 电机 A (左轮) ---- */
#ifndef TB6612_AIN1_PIN
#define TB6612_AIN1_PIN             GET_PIN(16, 6)  /* AIN1: P16.6 */
#endif

#ifndef TB6612_AIN2_PIN
#define TB6612_AIN2_PIN             GET_PIN(16, 7)  /* AIN2: P16.7 */
#endif

#ifndef TB6612_PWMA_DEV_NAME
#define TB6612_PWMA_DEV_NAME        "pwm18"         /* PWMA PWM 设备名 */
#endif

#ifndef TB6612_PWMA_CHANNEL
#define TB6612_PWMA_CHANNEL         0               /* PWMA PWM 通道 */
#endif

/* ---- 电机 B (右轮) ---- */
#ifndef TB6612_BIN1_PIN
#define TB6612_BIN1_PIN             GET_PIN(17, 0)  /* BIN1: P17.0 */
#endif

#ifndef TB6612_BIN2_PIN
#define TB6612_BIN2_PIN             GET_PIN(17, 1)  /* BIN2: P17.1 */
#endif

#ifndef TB6612_PWMB_DEV_NAME
#define TB6612_PWMB_DEV_NAME        "pwm18"         /* PWMB PWM 设备名 */
#endif

#ifndef TB6612_PWMB_CHANNEL
#define TB6612_PWMB_CHANNEL         1               /* PWMB PWM 通道 */
#endif

/* ============================================================
 * PWM 默认参数
 * ============================================================ */
#ifndef TB6612_PWM_PERIOD_NS
#define TB6612_PWM_PERIOD_NS        1000000         /* PWM 周期 1ms = 1KHz */
#endif

#ifndef TB6612_PWM_MAX_PULSE_NS
#define TB6612_PWM_MAX_PULSE_NS     1000000         /* 100% 占空比对应脉宽 */
#endif

/* ============================================================
 * 类型定义
 * ============================================================ */

/* 电机编号 */
typedef enum
{
    TB6612_MOTOR_A      = 0,    /* 电机 A (左轮) */
    TB6612_MOTOR_B      = 1,    /* 电机 B (右轮) */
    TB6612_MOTOR_BOTH   = 2,    /* 双电机 (同时操作) */
} tb6612_motor_t;

/* 电机方向 */
typedef enum
{
    TB6612_DIR_STOP     = 0,    /* 停止 (Coast) */
    TB6612_DIR_FORWARD  = 1,    /* 正转 / 前进 */
    TB6612_DIR_BACKWARD = 2,    /* 反转 / 后退 */
    TB6612_DIR_BRAKE    = 3,    /* 刹车 (Short Brake) */
} tb6612_dir_t;

/* 电机通道数据结构 */
typedef struct
{
    rt_base_t    in1_pin;        /* IN1 引脚 */
    rt_base_t    in2_pin;        /* IN2 引脚 */
    const char  *pwm_dev_name;   /* PWM 设备名称 */
    rt_uint32_t  pwm_channel;    /* PWM 通道号 */
    struct rt_device_pwm *pwm_dev;  /* PWM 设备句柄 */

    /* 当前状态 */
    tb6612_dir_t direction;      /* 当前方向 */
    rt_uint8_t   speed;          /* 当前速度百分比 (0~100) */
    rt_bool_t    enabled;        /* 是否已使能 */
} tb6612_motor_dev_t;

/* ============================================================
 * 公共 API
 * ============================================================ */

/**
 * @brief   初始化 TB6612 驱动模块
 * @return  RT_EOK 成功, 其他值表示失败
 * @note    此函数初始化 STBY 引脚, 并使能驱动芯片
 *          调用后会默认停止所有电机
 */
rt_err_t tb6612_init(void);

/**
 * @brief   使能/禁用 TB6612 驱动 (STBY 控制)
 * @param   enable  RT_TRUE=正常工作, RT_FALSE=待机
 */
void tb6612_standby(rt_bool_t enable);

/**
 * @brief   设置电机速度
 * @param   motor       电机编号 (A / B / BOTH)
 * @param   speed       速度百分比 (0 ~ 100)
 * @note    速度通过 PWM 占空比控制, 0=停止, 100=全速
 */
void tb6612_set_speed(tb6612_motor_t motor, rt_uint8_t speed);

/**
 * @brief   设置电机方向
 * @param   motor       电机编号 (A / B / BOTH)
 * @param   dir         方向 (正转 / 反转 / 停止 / 刹车)
 */
void tb6612_set_direction(tb6612_motor_t motor, tb6612_dir_t dir);

/**
 * @brief   控制电机: 同时设置方向和速度
 * @param   motor       电机编号 (A / B / BOTH)
 * @param   dir         方向
 * @param   speed       速度百分比 (0 ~ 100)
 */
void tb6612_control(tb6612_motor_t motor, tb6612_dir_t dir, rt_uint8_t speed);

/**
 * @brief   前进 (双电机同时正转)
 * @param   speed       速度百分比 (0 ~ 100)
 */
void tb6612_forward(rt_uint8_t speed);

/**
 * @brief   后退 (双电机同时反转)
 * @param   speed       速度百分比 (0 ~ 100)
 */
void tb6612_backward(rt_uint8_t speed);

/**
 * @brief   左转 (左轮减速/反转, 右轮正转)
 * @param   speed       速度百分比 (0 ~ 100)
 */
void tb6612_turn_left(rt_uint8_t speed);

/**
 * @brief   右转 (左轮正转, 右轮减速/反转)
 * @param   speed       速度百分比 (0 ~ 100)
 */
void tb6612_turn_right(rt_uint8_t speed);

/**
 * @brief   停车 (所有电机停止 - Coast)
 */
void tb6612_stop(void);

/**
 * @brief   刹车 (所有电机短接刹车 - Brake)
 */
void tb6612_brake(void);

/**
 * @brief   获取指定电机的当前方向
 * @param   motor       电机编号
 * @return  当前方向
 */
tb6612_dir_t tb6612_get_direction(tb6612_motor_t motor);

/**
 * @brief   获取指定电机的当前速度
 * @param   motor       电机编号
 * @return  当前速度百分比 (0~100)
 */
rt_uint8_t tb6612_get_speed(tb6612_motor_t motor);

#ifdef __cplusplus
}
#endif

#endif /* __TB6612_H__ */
