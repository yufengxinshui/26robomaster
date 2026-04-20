#include "gimbal_pid_comtrol.h"
#include <math.h>

void Gimbal_PID_Clear(Gimbal_PID_t *pid) {
    pid->err = pid->last_err = pid->p_out = pid->i_out = pid->d_out = pid->total_out = 0;
    pid->last_feedback = 0;
}

void Gimbal_PID_Calc(Gimbal_PID_t *pid, float target, float feedback, float feed_forward) {
    pid->last_err = pid->err;
    pid->err = target - feedback;

    // P项
    pid->p_out = pid->kp * pid->err;

    // I项 (积分分离 + 抗饱和)
    // 只有误差较小的时候才积分，且总输出未达到极限时才积分(可选优化，这里用简单的分离)
    if (fabs(pid->err) < pid->integral_limit_err) {
        pid->i_out += pid->ki * pid->err;

        // 积分限幅 (防点头、防疯转)
        if (pid->i_out > pid->max_iout) pid->i_out = pid->max_iout;
        else if (pid->i_out < -pid->max_iout) pid->i_out = -pid->max_iout;
    } else {
        pid->i_out = 0;
    }

    // D项 (微分先行)
    // 优点：调整目标 setpoint 时，不会产生巨大的微分冲击
    pid->d_out = pid->kd * -(feedback - pid->last_feedback);

    pid->last_feedback = feedback;

    // 总输出
    pid->total_out = pid->p_out + pid->i_out + pid->d_out + feed_forward;

    // 输出限幅
    if (pid->total_out > pid->max_out) pid->total_out = pid->max_out;
    if (pid->total_out < -pid->max_out) pid->total_out = -pid->max_out;
}