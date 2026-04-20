// gimbal_pid_control.h
#ifndef GIMBAL_PID_CONTROL_H
#define GIMBAL_PID_CONTROL_H

#include "struct_typedef.h" // 假设你有基础类型定义

typedef struct {
    // 参数区
    float kp;
    float ki;
    float kd;
    float max_out;       // 总输出限幅
    float max_iout;      // 积分限幅
    float integral_limit_err; // 积分分离阈值

    // 运行时数据
    float err;
    float last_err;
    float p_out;
    float i_out;
    float d_out;
    float total_out;

    // 微分先行优化
    float last_feedback;
} Gimbal_PID_t;

void Gimbal_PID_Calc(Gimbal_PID_t *pid, float target, float feedback, float feed_forward);
void Gimbal_PID_Clear(Gimbal_PID_t *pid); // 新增：清除PID历史状态

#endif