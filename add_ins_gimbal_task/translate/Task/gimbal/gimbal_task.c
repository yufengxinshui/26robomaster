#include "gimbal_task.h"
#include "gimbal_pid_comtrol.h"
#include "all_tou_h.h"
#include <math.h>
#include "INS_task.h"

/* ================== 参数配置宏 ================== */
#define GM6020_MAX_VOLTAGE  29000.0f
#define DEG2RAD             0.01745329f
#define RC_DEADBAND         10.0f

// 遥控器灵敏度 (根据手感微调)
#define YAW_RC_SEN          0.0005f     // Yaw (IMU)
#define PITCH_RC_SEN        0.0003f     // Pitch (Encoder)

// 机械中值 (必须准确!!!)
#define YAW_CENTER_ECD      4050
#define PITCH_CENTER_ECD    5487

// 机械限位 (相对于中值的角度)
// 注意：Yaw轴虽然用IMU控，但为了防断线，也需要软限位
#define PITCH_MAX_ANGLE     30.0f
#define PITCH_MIN_ANGLE     -20.0f
#define YAW_MAX_REL_ANGLE   120.0f      // 左右各120度，防止绕线

// 重力补偿系数
#define PITCH_GRAVITY_COMP  1200.0f

/* ================== 全局变量 ================== */
Gimbal_PID_t yaw_pos_pid, yaw_spd_pid;
Gimbal_PID_t pit_pos_pid, pit_spd_pid;

// 数据反馈
float ins_yaw, ins_yaw_spd;         // Yaw: IMU数据
float ecd_pitch, ecd_pitch_spd;     // Pitch: 编码器数据
float ecd_yaw_relative;             // Yaw: 编码器相对角度(仅用于限位保护)

// 目标控制量
float target_yaw_angle;   // 世界坐标系 (累计，无限位)
float target_pitch_angle; // 机械坐标系 (相对，有限位)

// 状态标志
uint8_t gimbal_is_calibrated = 0;

/* ================== 函数声明 ================== */
void Gimbal_Init_Params(void);
void Gimbal_Update_Control_Target(void);
void Gimbal_Check_Safety_Limit(float *yaw_cmd, float *pit_cmd);
float Motor_ECD_To_Angle(uint16_t ecd, uint16_t center_offset);
float Yaw_Angle_Error_Calc(float target, float current);
void Gimbal_Control_Loop();

/* ================== 任务入口 ================== */
void gimbal_task(void const *argument) {
    Gimbal_Init_Params();

    // 1. 等待传感器稳定
    osDelay(1000);

    // 2. 获取初始状态
    float dummy_pitch, dummy_roll;
    INS_Get_Angle(&ins_yaw, &dummy_pitch, &dummy_roll); // 获取当前IMU Yaw

    const motor_measure_t *pit_motor = get_pitch_gimbal_motor_measure_point();
    ecd_pitch = Motor_ECD_To_Angle(pit_motor->ecd, PITCH_CENTER_ECD); // 获取当前Pitch

    // 3. 设定初始目标 (锁住当前位置)
    target_yaw_angle = ins_yaw;
    target_pitch_angle = ecd_pitch;

    gimbal_is_calibrated = 1;

    while (1) {
        Gimbal_Control_Loop();
        osDelay(2);
    }
}

// 主控制循环 (赛级混合模式)
void Gimbal_Control_Loop() {
    // --- Yaw 轴 (主要用 IMU) ---
    float dummy_pitch, dummy_roll;
    float dummy_roll_spd, dummy_pit_spd, temp_yaw_spd;

    INS_Get_Angle(&ins_yaw, &dummy_pitch, &dummy_roll);
    INS_Get_Gyro(&dummy_roll_spd, &dummy_pit_spd, &temp_yaw_spd);
    // 简单的速度滤波
    static float last_yaw_spd = 0.0f;
    ins_yaw_spd = 0.3f * (temp_yaw_spd * 57.29578f) + 0.7f * last_yaw_spd;
    last_yaw_spd = ins_yaw_spd;
    // --- Pitch 轴 (用 Encoder) & Yaw辅助数据 ---
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    const motor_measure_t *pit_motor = get_pitch_gimbal_motor_measure_point();

    ecd_pitch = Motor_ECD_To_Angle(pit_motor->ecd, PITCH_CENTER_ECD);
    ecd_pitch_spd = (float)pit_motor->speed_rpm * 6.0f;
    // 获取 Yaw 的相对角度，用于防绕线保护
    ecd_yaw_relative = Motor_ECD_To_Angle(yaw_motor->ecd, YAW_CENTER_ECD);
    Gimbal_Update_Control_Target();
    /* -------- YAW 轴 (IMU 控制) -------- */
    // 目标是世界角度，反馈是世界角度
    float yaw_err = Yaw_Angle_Error_Calc(target_yaw_angle, ins_yaw);
    Gimbal_PID_Calc(&yaw_pos_pid, yaw_err, 0, 0);
    // 几何解耦 (防止抬头时 Yaw 变慢)
    float cos_pitch = fabsf(cosf(ecd_pitch * DEG2RAD));
    if (cos_pitch < 0.5f) cos_pitch = 0.5f;
    float yaw_feedback = ins_yaw_spd / cos_pitch;
    // 前馈：如果不是小陀螺模式，Yaw前馈可以给0，或者给一个小值抵消摩擦
    float yaw_ff = 0.0f;

    Gimbal_PID_Calc(&yaw_spd_pid, yaw_pos_pid.total_out, yaw_feedback, yaw_ff);


    /* -------- PITCH 轴 (Encoder 控制) -------- */
    // 目标是相对角度，反馈是相对角度
    // 优点：结构稳定，不会因为急停点头
    float pit_err = target_pitch_angle - ecd_pitch;
    Gimbal_PID_Calc(&pit_pos_pid, pit_err, 0, 0);

    // 重力补偿
    float gravity_comp = PITCH_GRAVITY_COMP * cosf(ecd_pitch * DEG2RAD);

    Gimbal_PID_Calc(&pit_spd_pid, pit_pos_pid.total_out, ecd_pitch_spd, gravity_comp);


    // ================= 4. 输出与保护 =================
    float yaw_voltage = yaw_spd_pid.total_out;
    float pit_voltage = pit_spd_pid.total_out;

    // 安全限位 (防止 Yaw 转断线，防止 Pitch 撞击)
    Gimbal_Check_Safety_Limit(&yaw_voltage, &pit_voltage);

    // 最终限幅
    if(yaw_voltage > GM6020_MAX_VOLTAGE) yaw_voltage = GM6020_MAX_VOLTAGE;
    if(yaw_voltage < -GM6020_MAX_VOLTAGE) yaw_voltage = -GM6020_MAX_VOLTAGE;
    if(pit_voltage > GM6020_MAX_VOLTAGE) pit_voltage = GM6020_MAX_VOLTAGE;
    if(pit_voltage < -GM6020_MAX_VOLTAGE) pit_voltage = -GM6020_MAX_VOLTAGE;

    CAN_cmd_gimbal((int16_t)yaw_voltage, (int16_t)pit_voltage, 0, 0);
}

// 遥控处理
void Gimbal_Update_Control_Target(void) {
    if(!gimbal_is_calibrated) return;
    const RC_ctrl_t *rc = get_remote_control_point();

    // Yaw: 控制的是【世界坐标系】的目标
    if (fabs(rc->rc.ch[0]) > RC_DEADBAND) {
        target_yaw_angle -= rc->rc.ch[0] * YAW_RC_SEN;
    }
    // IMU 角度是 0-360 循环的，目标也要处理
    while(target_yaw_angle > 180.0f) target_yaw_angle -= 360.0f;
    while(target_yaw_angle < -180.0f) target_yaw_angle += 360.0f;

    // Pitch: 控制的是【机械坐标系】的目标
    if (fabs(rc->rc.ch[1]) > RC_DEADBAND) {
        target_pitch_angle += rc->rc.ch[1] * PITCH_RC_SEN;
    }
    // Pitch 目标硬限位
    if (target_pitch_angle > PITCH_MAX_ANGLE) target_pitch_angle = PITCH_MAX_ANGLE;
    if (target_pitch_angle < PITCH_MIN_ANGLE) target_pitch_angle = PITCH_MIN_ANGLE;
}

// 软限位保护
void Gimbal_Check_Safety_Limit(float *yaw_cmd, float *pit_cmd) {
    // Pitch 轴保护
    if (ecd_pitch > PITCH_MAX_ANGLE && *pit_cmd > 0) *pit_cmd = 0;
    if (ecd_pitch < PITCH_MIN_ANGLE && *pit_cmd < 0) *pit_cmd = 0;

    // Yaw 轴保护 (即使是 IMU 模式，也要看机械角度防止断线)
    // 如果相对角度超过 +/- 120度，禁止往外转
    if (ecd_yaw_relative > YAW_MAX_REL_ANGLE && *yaw_cmd > 0) *yaw_cmd = 0;
    if (ecd_yaw_relative < -YAW_MAX_REL_ANGLE && *yaw_cmd < 0) *yaw_cmd = 0;
}

// 辅助函数
float Motor_ECD_To_Angle(uint16_t ecd, uint16_t center_offset) {
    int32_t temp = ecd - center_offset;
    if (temp > 4096) temp -= 8192;
    if (temp < -4096) temp += 8192;
    return (float)temp * 360.0f / 8192.0f;
}

float Yaw_Angle_Error_Calc(float target, float current) {
    float err = target - current;
    while(err > 180.0f) err -= 360.0f;
    while(err < -180.0f) err += 360.0f;
    return err;
}

// 赛级参数初始化
void Gimbal_Init_Params() {
    // --- YAW 轴 (IMU 模式) ---
    // IMU 反馈比较敏感，参数偏软
    yaw_pos_pid.kp = 2.0f;
    yaw_pos_pid.ki = 13.0f;
    yaw_pos_pid.kd = 1.0f; // 关D防震
    yaw_pos_pid.max_out = 400.0f;
    yaw_pos_pid.integral_limit_err = 5.0f;

    yaw_spd_pid.kp = 160.0f;
    yaw_spd_pid.ki = 10.0f;
    yaw_spd_pid.kd = 0.0f;
    yaw_spd_pid.max_out = GM6020_MAX_VOLTAGE;
    yaw_spd_pid.max_iout = 6000.0f;
    yaw_spd_pid.integral_limit_err = 200.0f;

    // --- PITCH 轴 (Encoder 模式) ---
    // 编码器反馈干净，参数可以硬一点
    // 【关键修正】你之前的代码这里全是0，导致无力！
    pit_pos_pid.kp = 0.0f;
    pit_pos_pid.ki = 0.0f;
    pit_pos_pid.kd = 0.5f;
    pit_pos_pid.max_out = 2000.0f;

    pit_spd_pid.kp =0.0f;
    pit_spd_pid.ki = 0.0f;  // Ki 辅助重力补偿
    pit_spd_pid.kd = 0.0f;
    pit_spd_pid.max_out = GM6020_MAX_VOLTAGE;
    pit_spd_pid.max_iout = 5000.0f;
    pit_spd_pid.integral_limit_err = 100.0f;

    Gimbal_PID_Clear(&yaw_pos_pid);
    Gimbal_PID_Clear(&yaw_spd_pid);
    Gimbal_PID_Clear(&pit_pos_pid);
    Gimbal_PID_Clear(&pit_spd_pid);
}