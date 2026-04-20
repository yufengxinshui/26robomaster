

#include "chassis_pid.h"
#include "all_tou_h.h"
#include "math.h"

// 阈值定义
#define BRAKE_THRESHOLD_RPM  60.0f
#define LOCK_THRESHOLD_RPM   5.0f

PID_State_t pid_3508_state[4] = {0};

PID_Params_t pm = {
    .kp = 2.5f,
    .ki = 1.05f,
    .kd = 1.5f,
    .max_output = 16000,
    .max_step = 3000,
    .i_sep = 300,
    .actual_alpha = 0.5f,
    .kf = 0.5f
};

void PID_3508_SetTarget(uint8_t motor_id, int16_t target) {
    pid_3508_state[motor_id].target = (float) target;
}

int16_t PID_3508_Incremental(uint8_t motor_id)
{
    PID_State_t *st = &pid_3508_state[motor_id];
    const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);

    // 速度滤波
    float raw_speed = (float)motor->speed_rpm;
    st->actual = pm.actual_alpha * raw_speed + (1.0f - pm.actual_alpha) * st->actual;

    // 目标归零
    if (fabsf(st->target) < 0.1f) st->target = 0.0f;

    // 误差更新
    st->error[2] = st->error[1];
    st->error[1] = st->error[0];
    st->error[0] = st->target - st->actual;

    float current_kp, current_ki, current_max_step;

    if (st->target == 0.0f)
    {
        // 急停
        if (fabsf(st->actual) > BRAKE_THRESHOLD_RPM)
        {
            current_kp = pm.kp * 1.5f;
            current_ki = 0.0f;
            current_max_step = 16000.0f;
        }
        // 锁定
        else
        {
            current_kp = pm.kp;
            current_ki = pm.ki * 5.0f;
            current_max_step = 3000.0f;
        }
    }
    else
    {
        current_kp = pm.kp;
        current_ki = pm.ki;
        current_max_step = pm.max_step;
    }

    // PID 计算
    float delta_p = current_kp * (st->error[0] - st->error[1]);

    float delta_i = 0.0f;
    if (fabsf(st->error[0]) < pm.i_sep) {
        delta_i = current_ki * st->error[0];
    }

    float delta_d = pm.kd * (st->error[0] - 2.0f * st->error[1] + st->error[2]);
    float total_delta = delta_p + delta_i + delta_d;

    if (isnan(total_delta)) {
        total_delta = 0.0f;
        // 状态清零，防止错误积累
        st->output = 0.0f;
        st->error[0] = st->error[1] = st->error[2] = 0.0f;
    }

    if (total_delta > current_max_step) total_delta = current_max_step;
    if (total_delta < -current_max_step) total_delta = -current_max_step;
    st->output += total_delta;

    // 输出值的 NaN 保护
    if (isnan(st->output)) {
        st->output = 0.0f;
    }

    // 全局限幅
    if (st->output > pm.max_output)  st->output = pm.max_output;
    if (st->output < -pm.max_output) st->output = -pm.max_output;

    if (st->target == 0.0f)
    {
        if (fabsf(st->actual) > 10.0f)
        {
            if (st->actual > 0 && st->output > 0) {
                st->output = 0;
            }
            else if (st->actual < 0 && st->output < 0) {
                st->output = 0;
            }
        }
        if (fabsf(st->actual) < 3.0f && fabsf(st->output) < 150.0f) {
             st->output = 0;
        }
    }

    // 前馈输出
    float final_out = st->output + (st->target == 0.0f ? 0 : pm.kf * st->target);

    if (final_out > pm.max_output)  final_out = pm.max_output;
    if (final_out < -pm.max_output) final_out = -pm.max_output;
    return (int16_t)final_out;
}


void PID_3508_Reset(void) {
    for (int i = 0; i < 4; i++) {
        // 清除所有电机 PID 的状态
        pid_3508_state[i].integral = 0.0f;
        pid_3508_state[i].output = 0.0f;
        // 目标值也应该清零，虽然在 chassis_task 下一帧会设置新的目标值
        pid_3508_state[i].target = 0.0f;
        pid_3508_state[i].error[0] = pid_3508_state[i].error[1] = pid_3508_state[i].error[2] = 0.0f;
        pid_3508_state[i].last_derivative = 0.0f;
    }
}



/**
 * @brief 打印所有底盘电机的实际转速
 * @note  建议在任务循环中调用，而不是在 PID 计算中调用
 */
void PID_3508_Debug_Print(void) {
    int16_t speed[4];

    // 一次性获取 4 个电机的速度
    for (int i = 0; i < 4; i++) {
        // 注意：这里获取的是 CAN 接收到的原始 RPM
        const motor_measure_t *motor = get_chassis_motor_measure_point(i);
        speed[i] = motor->speed_rpm;
    }

    // 格式化打印：使用逗号分隔，方便波形软件读取
    // 格式例如: "100, 200, -150, 0\r\n"
    // usart_printf("%d,%d,%d,%d\n", speed[0], speed[1], speed[2], speed[3]);
}
