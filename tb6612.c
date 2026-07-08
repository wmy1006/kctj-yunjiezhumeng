/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TB6612FNG 双路电机驱动模块 for PSoC Edge E84
 *
 * TB6612 控制逻辑:
 *   IN1  IN2  PWM   → 状态
 *   L    L    X     → 停止 (Coast) - 电机自由转动
 *   H    L    PWM   → 正转 (CW) - 占空比控制转速
 *   L    H    PWM   → 反转 (CCW) - 占空比控制转速
 *   H    H    X     → 刹车 (Short Brake) - 电机快速停止
 *   STBY = L        → 待机 (所有通道禁用)
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-06     user         first version
 */

#include "tb6612.h"

/* ============================================================
 * 全局变量
 * ============================================================ */
static tb6612_motor_dev_t g_motor_a;           /* 电机 A */
static tb6612_motor_dev_t g_motor_b;           /* 电机 B */
static rt_base_t           g_stby_pin;         /* STBY 引脚 */
static rt_bool_t           g_initialized = RT_FALSE;  /* 初始化标志 */

/* ============================================================
 * 初始化单个电机通道
 * ============================================================ */
static rt_err_t tb6612_motor_init(tb6612_motor_dev_t *motor,
                                   rt_base_t in1_pin,
                                   rt_base_t in2_pin,
                                   const char *pwm_name,
                                   rt_uint32_t pwm_channel)
{
    RT_ASSERT(motor != RT_NULL);

    motor->in1_pin      = in1_pin;
    motor->in2_pin      = in2_pin;
    motor->pwm_dev_name = pwm_name;
    motor->pwm_channel  = pwm_channel;
    motor->direction    = TB6612_DIR_STOP;
    motor->speed        = 0;
    motor->enabled      = RT_FALSE;

    /* 配置方向控制引脚为输出 */
    rt_pin_mode(motor->in1_pin, PIN_MODE_OUTPUT);
    rt_pin_mode(motor->in2_pin, PIN_MODE_OUTPUT);

    /* 初始化为停止状态 */
    rt_pin_write(motor->in1_pin, PIN_LOW);
    rt_pin_write(motor->in2_pin, PIN_LOW);

    /* 查找 PWM 设备 */
    motor->pwm_dev = (struct rt_device_pwm *)rt_device_find(pwm_name);
    if (motor->pwm_dev == RT_NULL)
    {
        rt_kprintf("[TB6612] Warning: PWM device '%s' not found.\n", pwm_name);
        rt_kprintf("[TB6612] Motor will run without speed control (GPIO only).\n");
        /* 没有 PWM 时, 使用 GPIO 全速运行 */
    }
    else
    {
        motor->enabled = RT_TRUE;
    }

    return RT_EOK;
}

/* ============================================================
 * 设置单通道 PWM 占空比
 * ============================================================ */
static void tb6612_motor_set_pwm(tb6612_motor_dev_t *motor, rt_uint8_t speed)
{
    if (motor->pwm_dev == RT_NULL || !motor->enabled)
    {
        return;
    }

    struct rt_pwm_configuration cfg = {0};
    cfg.channel = motor->pwm_channel;
    cfg.period  = TB6612_PWM_PERIOD_NS;

    if (speed == 0)
    {
        /* 速度为 0, 禁用 PWM 输出 */
        cfg.pulse = 0;
        rt_pwm_set(motor->pwm_dev, &cfg);
        rt_pwm_disable(motor->pwm_dev, motor->pwm_channel);
    }
    else
    {
        /* 计算占空比: pulse_ns = period_ns * speed / 100 */
        cfg.pulse = (rt_uint64_t)TB6612_PWM_PERIOD_NS * speed / 100;
        if (cfg.pulse < 1)
        {
            cfg.pulse = 1;
        }
        if (cfg.pulse > cfg.period)
        {
            cfg.pulse = cfg.period;
        }
        rt_pwm_set(motor->pwm_dev, &cfg);
        rt_pwm_enable(motor->pwm_dev, motor->pwm_channel);
    }
}

/* ============================================================
 * 设置单电机方向和速度
 * ============================================================ */
static void tb6612_motor_control(tb6612_motor_dev_t *motor,
                                  tb6612_dir_t dir,
                                  rt_uint8_t speed)
{
    RT_ASSERT(motor != RT_NULL);

    /* 限幅速度 */
    if (speed > 100) speed = 100;

    motor->direction = dir;
    motor->speed     = speed;

    switch (dir)
    {
    case TB6612_DIR_STOP:
        /* 停止: IN1=L, IN2=L */
        rt_pin_write(motor->in1_pin, PIN_LOW);
        rt_pin_write(motor->in2_pin, PIN_LOW);
        tb6612_motor_set_pwm(motor, 0);
        break;

    case TB6612_DIR_FORWARD:
        /* 正转: IN1=H, IN2=L + PWM */
        rt_pin_write(motor->in1_pin, PIN_HIGH);
        rt_pin_write(motor->in2_pin, PIN_LOW);
        tb6612_motor_set_pwm(motor, speed);
        break;

    case TB6612_DIR_BACKWARD:
        /* 反转: IN1=L, IN2=H + PWM */
        rt_pin_write(motor->in1_pin, PIN_LOW);
        rt_pin_write(motor->in2_pin, PIN_HIGH);
        tb6612_motor_set_pwm(motor, speed);
        break;

    case TB6612_DIR_BRAKE:
        /* 刹车: IN1=H, IN2=H */
        rt_pin_write(motor->in1_pin, PIN_HIGH);
        rt_pin_write(motor->in2_pin, PIN_HIGH);
        tb6612_motor_set_pwm(motor, 0);
        break;

    default:
        break;
    }
}

/* ============================================================
 * 初始化 TB6612 驱动模块
 * ============================================================ */
rt_err_t tb6612_init(void)
{
    if (g_initialized)
    {
        rt_kprintf("[TB6612] Already initialized.\n");
        return RT_EOK;
    }

    g_stby_pin = TB6612_STBY_PIN;

    /* 配置 STBY 引脚 */
    rt_pin_mode(g_stby_pin, PIN_MODE_OUTPUT);
    rt_pin_write(g_stby_pin, PIN_LOW);  /* 初始设为待机 */

    /* 初始化电机 A */
    tb6612_motor_init(&g_motor_a,
                      TB6612_AIN1_PIN,
                      TB6612_AIN2_PIN,
                      TB6612_PWMA_DEV_NAME,
                      TB6612_PWMA_CHANNEL);

    /* 初始化电机 B */
    tb6612_motor_init(&g_motor_b,
                      TB6612_BIN1_PIN,
                      TB6612_BIN2_PIN,
                      TB6612_PWMB_DEV_NAME,
                      TB6612_PWMB_CHANNEL);

    /* 使能驱动芯片 */
    rt_pin_write(g_stby_pin, PIN_HIGH);

    g_initialized = RT_TRUE;

    rt_kprintf("[TB6612] Init OK.\n");
    rt_kprintf("[TB6612]   Motor A: IN1=P%d.%d, IN2=P%d.%d, PWM=%s CH%d\n",
               CYHAL_GET_PORT(TB6612_AIN1_PIN), CYHAL_GET_PIN(TB6612_AIN1_PIN),
               CYHAL_GET_PORT(TB6612_AIN2_PIN), CYHAL_GET_PIN(TB6612_AIN2_PIN),
               TB6612_PWMA_DEV_NAME, TB6612_PWMA_CHANNEL);
    rt_kprintf("[TB6612]   Motor B: IN1=P%d.%d, IN2=P%d.%d, PWM=%s CH%d\n",
               CYHAL_GET_PORT(TB6612_BIN1_PIN), CYHAL_GET_PIN(TB6612_BIN1_PIN),
               CYHAL_GET_PORT(TB6612_BIN2_PIN), CYHAL_GET_PIN(TB6612_BIN2_PIN),
               TB6612_PWMB_DEV_NAME, TB6612_PWMB_CHANNEL);
    rt_kprintf("[TB6612]   STBY: P%d.%d\n",
               CYHAL_GET_PORT(TB6612_STBY_PIN), CYHAL_GET_PIN(TB6612_STBY_PIN));

    return RT_EOK;
}

/* ============================================================
 * 使能/禁用 TB6612 驱动 (STBY 控制)
 * ============================================================ */
void tb6612_standby(rt_bool_t enable)
{
    if (!g_initialized) return;

    if (enable)
    {
        rt_pin_write(g_stby_pin, PIN_HIGH);
    }
    else
    {
        rt_pin_write(g_stby_pin, PIN_LOW);
    }
}

/* ============================================================
 * 设置电机速度
 * ============================================================ */
void tb6612_set_speed(tb6612_motor_t motor, rt_uint8_t speed)
{
    if (!g_initialized) return;

    if (speed > 100) speed = 100;

    switch (motor)
    {
    case TB6612_MOTOR_A:
        g_motor_a.speed = speed;
        tb6612_motor_control(&g_motor_a, g_motor_a.direction, speed);
        break;

    case TB6612_MOTOR_B:
        g_motor_b.speed = speed;
        tb6612_motor_control(&g_motor_b, g_motor_b.direction, speed);
        break;

    case TB6612_MOTOR_BOTH:
        g_motor_a.speed = speed;
        g_motor_b.speed = speed;
        tb6612_motor_control(&g_motor_a, g_motor_a.direction, speed);
        tb6612_motor_control(&g_motor_b, g_motor_b.direction, speed);
        break;

    default:
        break;
    }
}

/* ============================================================
 * 设置电机方向
 * ============================================================ */
void tb6612_set_direction(tb6612_motor_t motor, tb6612_dir_t dir)
{
    if (!g_initialized) return;

    switch (motor)
    {
    case TB6612_MOTOR_A:
        tb6612_motor_control(&g_motor_a, dir, g_motor_a.speed);
        break;

    case TB6612_MOTOR_B:
        tb6612_motor_control(&g_motor_b, dir, g_motor_b.speed);
        break;

    case TB6612_MOTOR_BOTH:
        tb6612_motor_control(&g_motor_a, dir, g_motor_a.speed);
        tb6612_motor_control(&g_motor_b, dir, g_motor_b.speed);
        break;

    default:
        break;
    }
}

/* ============================================================
 * 控制电机: 同时设置方向和速度
 * ============================================================ */
void tb6612_control(tb6612_motor_t motor, tb6612_dir_t dir, rt_uint8_t speed)
{
    if (!g_initialized) return;

    if (speed > 100) speed = 100;

    switch (motor)
    {
    case TB6612_MOTOR_A:
        tb6612_motor_control(&g_motor_a, dir, speed);
        break;

    case TB6612_MOTOR_B:
        tb6612_motor_control(&g_motor_b, dir, speed);
        break;

    case TB6612_MOTOR_BOTH:
        tb6612_motor_control(&g_motor_a, dir, speed);
        tb6612_motor_control(&g_motor_b, dir, speed);
        break;

    default:
        break;
    }
}

/* ============================================================
 * 前进 (双电机正转)
 * ============================================================ */
void tb6612_forward(rt_uint8_t speed)
{
    tb6612_control(TB6612_MOTOR_BOTH, TB6612_DIR_FORWARD, speed);
}

/* ============================================================
 * 后退 (双电机反转)
 * ============================================================ */
void tb6612_backward(rt_uint8_t speed)
{
    tb6612_control(TB6612_MOTOR_BOTH, TB6612_DIR_BACKWARD, speed);
}

/* ============================================================
 * 左转 (左轮减速/反转, 右轮正转)
 * ============================================================ */
void tb6612_turn_left(rt_uint8_t speed)
{
    if (!g_initialized) return;

    if (speed > 100) speed = 100;

    /* 左轮反转 (或停止), 右轮正转实现左转 */
    tb6612_motor_control(&g_motor_a, TB6612_DIR_STOP, 0);
    tb6612_motor_control(&g_motor_b, TB6612_DIR_FORWARD, speed);
}

/* ============================================================
 * 右转 (左轮正转, 右轮减速/反转)
 * ============================================================ */
void tb6612_turn_right(rt_uint8_t speed)
{
    if (!g_initialized) return;

    if (speed > 100) speed = 100;

    /* 左轮正转, 右轮反转 (或停止) 实现右转 */
    tb6612_motor_control(&g_motor_a, TB6612_DIR_FORWARD, speed);
    tb6612_motor_control(&g_motor_b, TB6612_DIR_STOP, 0);
}

/* ============================================================
 * 停车 (Coast - 电机自由停止)
 * ============================================================ */
void tb6612_stop(void)
{
    if (!g_initialized) return;

    tb6612_motor_control(&g_motor_a, TB6612_DIR_STOP, 0);
    tb6612_motor_control(&g_motor_b, TB6612_DIR_STOP, 0);
}

/* ============================================================
 * 刹车 (Brake - 电机短接快速停止)
 * ============================================================ */
void tb6612_brake(void)
{
    if (!g_initialized) return;

    tb6612_motor_control(&g_motor_a, TB6612_DIR_BRAKE, 0);
    tb6612_motor_control(&g_motor_b, TB6612_DIR_BRAKE, 0);
}

/* ============================================================
 * 获取电机方向
 * ============================================================ */
tb6612_dir_t tb6612_get_direction(tb6612_motor_t motor)
{
    if (!g_initialized) return TB6612_DIR_STOP;

    switch (motor)
    {
    case TB6612_MOTOR_A:  return g_motor_a.direction;
    case TB6612_MOTOR_B:  return g_motor_b.direction;
    default:              return TB6612_DIR_STOP;
    }
}

/* ============================================================
 * 获取电机速度
 * ============================================================ */
rt_uint8_t tb6612_get_speed(tb6612_motor_t motor)
{
    if (!g_initialized) return 0;

    switch (motor)
    {
    case TB6612_MOTOR_A:  return g_motor_a.speed;
    case TB6612_MOTOR_B:  return g_motor_b.speed;
    default:              return 0;
    }
}

/* ============================================================
 * MSH Shell 命令 (调试用)
 * ============================================================ */
#ifdef RT_USING_FINSH
#include <finsh.h>

static void tb6612_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("  tb6612 fwd  <speed>        - forward\n");
        rt_kprintf("  tb6612 back <speed>        - backward\n");
        rt_kprintf("  tb6612 left <speed>        - turn left\n");
        rt_kprintf("  tb6612 right <speed>       - turn right\n");
        rt_kprintf("  tb6612 stop                - coast stop\n");
        rt_kprintf("  tb6612 brake               - short brake\n");
        rt_kprintf("  tb6612 stby <on|off>       - standby control\n");
        rt_kprintf("  tb6612 ma <fwd|back|stop|brake> <speed>  - motor A only\n");
        rt_kprintf("  tb6612 mb <fwd|back|stop|brake> <speed>  - motor B only\n");
        return;
    }

    /* 前进 */
    if (rt_strcmp(argv[1], "fwd") == 0)
    {
        rt_uint8_t spd = (argc >= 3) ? (rt_uint8_t)atol(argv[2]) : 50;
        tb6612_forward(spd);
        rt_kprintf("[TB6612] Forward, speed=%d%%\n", spd);
    }
    /* 后退 */
    else if (rt_strcmp(argv[1], "back") == 0)
    {
        rt_uint8_t spd = (argc >= 3) ? (rt_uint8_t)atol(argv[2]) : 50;
        tb6612_backward(spd);
        rt_kprintf("[TB6612] Backward, speed=%d%%\n", spd);
    }
    /* 左转 */
    else if (rt_strcmp(argv[1], "left") == 0)
    {
        rt_uint8_t spd = (argc >= 3) ? (rt_uint8_t)atol(argv[2]) : 50;
        tb6612_turn_left(spd);
        rt_kprintf("[TB6612] Turn Left, speed=%d%%\n", spd);
    }
    /* 右转 */
    else if (rt_strcmp(argv[1], "right") == 0)
    {
        rt_uint8_t spd = (argc >= 3) ? (rt_uint8_t)atol(argv[2]) : 50;
        tb6612_turn_right(spd);
        rt_kprintf("[TB6612] Turn Right, speed=%d%%\n", spd);
    }
    /* 停止 */
    else if (rt_strcmp(argv[1], "stop") == 0)
    {
        tb6612_stop();
        rt_kprintf("[TB6612] Stop (Coast)\n");
    }
    /* 刹车 */
    else if (rt_strcmp(argv[1], "brake") == 0)
    {
        tb6612_brake();
        rt_kprintf("[TB6612] Brake\n");
    }
    /* 待机 */
    else if (rt_strcmp(argv[1], "stby") == 0)
    {
        if (argc >= 3)
        {
            if (rt_strcmp(argv[2], "on") == 0)
            {
                tb6612_standby(RT_TRUE);
                rt_kprintf("[TB6612] Standby OFF (active)\n");
            }
            else
            {
                tb6612_standby(RT_FALSE);
                rt_kprintf("[TB6612] Standby ON (sleep)\n");
            }
        }
    }
    /* 单独控制电机 A */
    else if (rt_strcmp(argv[1], "ma") == 0 && argc >= 3)
    {
        rt_uint8_t spd = (argc >= 4) ? (rt_uint8_t)atol(argv[3]) : 50;
        tb6612_dir_t dir = TB6612_DIR_STOP;

        if (rt_strcmp(argv[2], "fwd") == 0)        dir = TB6612_DIR_FORWARD;
        else if (rt_strcmp(argv[2], "back") == 0)  dir = TB6612_DIR_BACKWARD;
        else if (rt_strcmp(argv[2], "stop") == 0)  dir = TB6612_DIR_STOP;
        else if (rt_strcmp(argv[2], "brake") == 0) dir = TB6612_DIR_BRAKE;

        tb6612_control(TB6612_MOTOR_A, dir, spd);
        rt_kprintf("[TB6612] Motor A: dir=%d, speed=%d%%\n", dir, spd);
    }
    /* 单独控制电机 B */
    else if (rt_strcmp(argv[1], "mb") == 0 && argc >= 3)
    {
        rt_uint8_t spd = (argc >= 4) ? (rt_uint8_t)atol(argv[3]) : 50;
        tb6612_dir_t dir = TB6612_DIR_STOP;

        if (rt_strcmp(argv[2], "fwd") == 0)        dir = TB6612_DIR_FORWARD;
        else if (rt_strcmp(argv[2], "back") == 0)  dir = TB6612_DIR_BACKWARD;
        else if (rt_strcmp(argv[2], "stop") == 0)  dir = TB6612_DIR_STOP;
        else if (rt_strcmp(argv[2], "brake") == 0) dir = TB6612_DIR_BRAKE;

        tb6612_control(TB6612_MOTOR_B, dir, spd);
        rt_kprintf("[TB6612] Motor B: dir=%d, speed=%d%%\n", dir, spd);
    }
    else
    {
        rt_kprintf("[TB6612] Unknown command: %s\n", argv[1]);
    }
}
MSH_CMD_EXPORT(tb6612, TB6612 motor control: tb6612 <fwd|back|left|right|stop|brake> [speed]);
#endif /* RT_USING_FINSH */
