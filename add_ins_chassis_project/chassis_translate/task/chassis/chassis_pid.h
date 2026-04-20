#ifndef CHASSIS_PID_H
#define CHASSIS_PID_H

#include <stdint.h>
#include "all_tou_h.h"

// 假设您的控制周期 dt = 1ms (0.001s)
#define CONTROL_DT              0.001f
#define MAX_SPIN_REF_SPEED      360.0f // 遥控器摇满时，目标瞄准角的最大变化速度 (例如：360 deg/s)
#define MAX_CHASSIS_RATE        300.0f // 底盘最大角速度输出 (max_out)

// PID 状态结构体
typedef struct
{
    float target;          // 目标值（改成 float，确保精度足够）
    float actual;          // 实际值
    float error[3];        // 误差队列
    float integral;        // 积分项
    float output;          // 输出值
    float last_derivative; // 微分滤波缓存
} PID_State_t;


// PID 参数结构体（与 C 文件完全匹配）
typedef struct
{
    float kp;
    float ki;
    float kd;

    int16_t max_output;
    int16_t max_integral;
    int16_t max_step;

    float alpha;            // 微分低通
    float actual_alpha;     // 实际速度低通
    int16_t i_sep;          // 积分分离阈值
    float anti_windup_k;    // 反风up比例
    int16_t deadband;       // 输出死区
    float kf;               // ⭐速度前馈系数（你之前声明有，但 C 文件没用到，我帮你补上）
} PID_Params_t;


// 外部变量
extern PID_State_t pid_3508_state[4];
extern PID_Params_t pid_3508_params;

extern pid_type_def follow_pid; // 底盘跟随云台模式的 PID

// 函数
int16_t PID_3508_Incremental(uint8_t motor_id);
void PID_3508_SetTarget(uint8_t motor_id, int16_t target);
void PID_3508_Reset(void);
void PID_3508_Debug_Print(void);

#endif
