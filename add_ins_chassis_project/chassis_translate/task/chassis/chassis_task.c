#include "all_tou_h.h"
#include "chassis_task.h"
#include "chassis_pid.h"
#include  "remote_control.h"
#include "math.h"
#include "INS_task.h"


// 定义跟随模式的 PID 结构体
pid_type_def chassis_follow_pid;
// 定义 PID 参数数组 {Kp, Ki, Kd}
// 调参建议：
// P: 4.0 ~ 8.0 (响应速度)
// I: 0.0 (通常不需要)
// D: 0.5 ~ 2.0 (抑制震荡)
static const fp32 follow_pid_params[3] = {1000, 0, 0.5};

// PID 输出限幅 (最大旋转速度)
#define FOLLOW_MAX_OUT  5000.0f
// 积分限幅
#define FOLLOW_MAX_IOUT 300.0f


// JoystickData js_data;   // 摇杆数据
int16_t output_speed[4] = {0}; // 4个电机输出速度
int16_t target_speeds[4] = {0}; // 各电机目标速度

chassis_move_t chassis_move;

// 存储当前平滑后的速度值，仅供 chassis_ramp_speed_update 使用
static fp32 s_ramped_vx = 0.0f;
static fp32 s_ramped_vy = 0.0f;
static fp32 s_ramped_omega = 0.0f;

#define CHASSIS_MAX_ACCEL_VX   15.0f  // 假设目标速度最大值是 2500，希望 0.5s 达到最大，则 2500/(0.5/0.01) = 50
#define CHASSIS_MAX_ACCEL_VY    15.0f
#define CHASSIS_MAX_ACCEL_OMEGA 80.0f   // 角速度的斜坡应该更小
#define CHASSIS_RAMP_STOP_THRESHOLD 0.1f // m/s 

//函数声明
void chassis_task(void const *argument);

void chassis_set_mode();

void omni_wheel_kinematics(const fp32 vx_set, const fp32 vy_set, const fp32 omega, fp32 motor_speeds[4]);

void chassis_control_loop(chassis_move_t *chassis_control_loop);

void usart_printf(const char *fmt, ...);

void remote_control_data_cale(float Targe_Speed, float Omega_Speed, fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

void chassis_set_control(chassis_move_t *chassis_set_control);

void parse_joystick_data();

void chassis_ramp_speed_update(chassis_move_t *chassis_set_control);

float get_chassis_relative_angle_rad(void);

float chassis_follow_gimbal_cal(void);

// 添加死区处理函数
float apply_deadzone(float input, float deadzone) {
    if (fabs(input) <= deadzone) {
        return 0.f; // 在死区内，返回中位值
    }
    return input;
}

void chassis_task(void const *argument) {
    // local_rc_ctrl = get_remote_control_point();
    // parse_joystick_data();
    // 【初始化跟随 PID】
    // 使用 PID_POSITION (位置环)
    PID_init(&chassis_follow_pid, PID_POSITION, follow_pid_params, FOLLOW_MAX_OUT, FOLLOW_MAX_IOUT);
    while (1) {

        // sbusPreparePacket(sbus_rx_buf[0]);

        // usart_printf("%d,%d,%d,%d,%d,%d\n",
        //             local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3],
        //             local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1]);

        parse_joystick_data();
        // usart_printf("%d,%d\n",(int)js_data.ly,(int)js_data.lx);

        chassis_set_mode(); //模式更新记录
        // modo_test();

        chassis_set_control(&chassis_move);

        chassis_ramp_speed_update(&chassis_move);

        chassis_control_loop(&chassis_move);

        // 打印：模式、相对角度(角度制)、PID输出速度
        // usart_printf("%.2f,%.2f\n",
        //
        //             get_chassis_relative_angle_rad() * 57.3f,
        //             chassis_move.omega_set);

        usart_printf("%d, %.2f, %d\n",
                     chassis_behaviour_mode,
                     chassis_move.vx_set,
                     output_speed[0]);

        if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
            // 无力模式：发送 0
            CAN_cmd_chassis(0, 0, 0, 0);
        } else {
            // 正常模式：发送 PID 计算出的电流
            CAN_cmd_chassis(output_speed[0], output_speed[1], output_speed[2], output_speed[3]);
        }
        // CAN_cmd_chassis(output_speed[0], output_speed[1], output_speed[2], output_speed[3]);
        osDelay(10);
    }
}

void chassis_set_mode() {
    chassis_behaviour_mode_set();
}


// void chassis_set_control(chassis_move_t *chassis_set_control) {
//     fp32 vx_set = 0.0f, vy_set = 0.0f, omega_set = 0.0f;
//     chassis_behaviour_control_set(&vx_set, &vy_set, &omega_set);
//     chassis_set_control->vx_set = vx_set;
//     chassis_set_control->vy_set = vy_set;
//     chassis_set_control->omega_set = omega_set;
//     // usart_printf("%.2f\n", chassis_set_control->vx_set);
// }

void chassis_set_control(chassis_move_t *chassis_set_control) {
    fp32 vx_raw = 0.0f, vy_raw = 0.0f, omega_raw = 0.0f;

    // 1. 获取遥控器解析出的原始目标值 (vx, vy, omega)
    // 注意：chassis_behaviour_control_set 内部会根据模式给这三个变量赋值
    chassis_behaviour_control_set(&vx_raw, &vy_raw, &omega_raw);

    // 2. 根据【当前行为模式】修正旋转速度 omega
    // 这里统一使用你在 chassis_behaviour.h 中定义的枚举
    switch (chassis_behaviour_mode) {

        case CHASSIS_OPEN:
            // 【跟随模式】：覆盖遥控器的旋转输入，使用 PID 计算值实现对齐
            omega_raw = chassis_follow_gimbal_cal();
            break;

        case CHASSIS_REVOLVE: // 如果你想实现小陀螺，对应你枚举里的 REVOLVE
            // 【小陀螺】：设定恒定旋转速度
            chassis_set_control->vx_set = vx_raw;
            chassis_set_control->vy_set = vy_raw ;
            chassis_set_control->omega_set = omega_raw;
            break;

        case CHASSIS_ZERO_FORCE:
            // 【无力模式】：强制全部归零
            vx_raw = 0.0f;
            vy_raw = 0.0f;
            omega_raw = 0.0f;
            break;

        default:
            // 其他模式（如 CHASSIS_OPEN 或 CHASSIS_ROLL_MOVE）
            // 直接维持 remote_control_data_cale 计算出的原始 omega_raw
            break;
    }

    // 3. 最终赋值给底盘任务结构体
    chassis_set_control->vx_set = vx_raw;
    chassis_set_control->vy_set = vy_raw;
    chassis_set_control->omega_set = omega_raw;
}


void omni_wheel_kinematics(const fp32 vx_set, const fp32 vy_set, const fp32 omega_set, fp32 motor_speeds[4]) {
    //十字型
    // motor_speeds[0] = ( -SQRT2/2.0f * vy + omega * r) / s;
    // motor_speeds[1] = ( SQRT2/2.0f * vx  + omega * r) / s;
    // motor_speeds[3] = ( SQRT2/2.0f * vy + omega * r) / s;
    // motor_speeds[2] = ( -SQRT2/2.0f * vx  + omega * r) / s;
    // usart_printf("%f,%f\n", vx_set, vy_set);
    motor_speeds[0] = (-SQRT2  * vx_set - SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
    motor_speeds[1] = (-SQRT2 * vx_set + SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
    motor_speeds[2] = (SQRT2  * vx_set + SQRT2  * vy_set + omega_set * r_wheel) / s_wheel;
    motor_speeds[3] = (SQRT2  * vx_set - SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
}


void chassis_control_loop(chassis_move_t *chassis_control_loop) {
    float max_vector = 0.0f, vector_rate = 0.0f;
    float temp = 0.0f;
    uint8_t i = 0;
    float motor_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int16_t max_speed = 2500;

    // 获取目标速度 (云台坐标系)
    float vx_set = chassis_control_loop->vx_set;
    float vy_set = chassis_control_loop->vy_set;
    float omega_set = chassis_control_loop->omega_set;
    // usart_printf("%f,%f\n",vy_set,vx_set);

    // 获取相对角度 (弧度)
    float theta_rad = get_chassis_relative_angle_rad();

    // 向量旋转 (World -> Body)
    // 注：如果发现推前变成了往后跑，把下面的 sin/cos 前面的正负号整体反过来
    float cos_a = cos(theta_rad);
    float sin_a = -sin(theta_rad);
    float vx_chassis = vx_set * cos_a + vy_set * sin_a;
    float vy_chassis = -vx_set * sin_a + vy_set * cos_a;

    omni_wheel_kinematics(vx_chassis, vy_chassis, omega_set,
                          motor_speeds);

    // 计算最大轮速并进行限幅
    for (i = 0; i < 4; i++) {
        target_speeds[i] = motor_speeds[i];
        temp = fabs(target_speeds[i]);
        if (max_vector < temp) {
            max_vector = temp;
        }
    }
    // 超过最大速度则等比例缩小
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++) {
            target_speeds[i] *= vector_rate;
        }
    }
    // PID计算
    for (i = 0; i < 4; i++) {
        PID_3508_SetTarget(i, target_speeds[i]);
        output_speed[i] = PID_3508_Incremental(i);
    }
    // PID_3508_Debug_Print();

}

//处理遥控数值配比为底盘控制值
void remote_control_data_cale(float Targe_Speed, float Omega_Speed, fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {

    parse_joystick_data();

    float vx = 0.0f, vy = 0.0f, omega = 0.0f;
    float deadzone = 50.0f; //添加死区大小
    //应用死区处理
    float processed_ch3 = apply_deadzone(js_data.ly, deadzone);
    float processed_ch4 = apply_deadzone(js_data.lx, deadzone);

    float P_coefficient = Targe_Speed / 660.f;

    vy = (processed_ch3) * P_coefficient;
    vx = (processed_ch4) * P_coefficient;
    omega = Omega_Speed;

    // 将计算得到的vx和vy通过指针返回
    *vx_set = vx;
    *vy_set = vy;
    *omega_set = omega;
    // usart_printf("%d,%d\n",(int)js_data.ly,(int)js_data.lever1);
    // usart_printf("%.2f,%.2f\n",vy,vx);
    // usart_printf("%d,%d,%d,%d,%d,%d\n",
    //             local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3],
    //             local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1]);
}

// [chassis_task.c]

void parse_joystick_data() {
    const rc_can2_t *rc_rx = get_rc_can2_point();

    // 【新增】简单的数据有效性检查
    // 如果所有通道都是0，极可能是遥控器接收机断电或线松了
    // 此时不更新数据，保持上一次的值，或者进入安全模式
    if (rc_rx->ch[0] == 0 && rc_rx->ch[1] == 0 && rc_rx->ch[2] == 0 && rc_rx->ch[3] == 0 && rc_rx->s[0] == 0) {
        // 数据无效，直接返回，不要更新 js_data，防止误触急停
        return;
    }

    js_data.ly = rc_rx->ch[3];
    js_data.lx = -(rc_rx->ch[2]);
    js_data.lever1 = rc_rx->s[0];
    js_data.lever2 = rc_rx->s[1];
}

/**
 * @brief 对单个速度值进行速率限制（Ramp）
 * @param target 期望的原始目标速度
 * @param current 当前平滑后的速度指针
 * @param max_delta 每周期最大变化量
 * @return fp32 平滑后的新速度
 */
// [chassis_task.c]

static fp32 calculate_single_ramp(fp32 target, fp32 *current, fp32 max_delta)
{
    // 【新增】NaN (非数) 异常保护
    // 如果 current 变成了 NaN (例如因震动导致的除以0)，强制复位
    if (isnan(*current)) {
        *current = 0.0f;
    }

    fp32 error = target - *current;

    if (error > max_delta) {
        error = max_delta;
    } else if (error < -max_delta) {
        error = -max_delta;
    }

    *current += error;

    return *current;
}

/**
 * @brief 对底盘目标速度进行斜坡处理并更新结构体
 * @param chassis_set_control 指向底盘控制结构体的指针
 */
// void chassis_ramp_speed_update(chassis_move_t *chassis_set_control)
// {
//     // 获取原始目标速度
//     fp32 raw_vx = chassis_set_control->vx_set;
//     fp32 raw_vy = chassis_set_control->vy_set;
//     fp32 raw_omega = chassis_set_control->omega_set;
//
//
//     // 处理 VX 轴
//     if (fabsf(raw_vx) < CHASSIS_RAMP_STOP_THRESHOLD) // 摇杆回中
//     {
//         s_ramped_vx = 0.0f; // 强制平滑速度目标为 0
//     }
//     else
//     {
//         // 正常斜坡控制
//         s_ramped_vx = calculate_single_ramp(raw_vx, &s_ramped_vx, CHASSIS_MAX_ACCEL_VX);
//     }
//     chassis_set_control->vx_set = s_ramped_vx;
//
//
//     // 处理 VY 轴 
//     if (fabsf(raw_vy) < CHASSIS_RAMP_STOP_THRESHOLD)
//     {
//         s_ramped_vy = 0.0f;
//     }
//     else
//     {
//         s_ramped_vy = calculate_single_ramp(raw_vy, &s_ramped_vy, CHASSIS_MAX_ACCEL_VY);
//     }
//     chassis_set_control->vy_set = s_ramped_vy;
//
//
//     // 处理 Omega 轴 
//     if (fabsf(raw_omega) < CHASSIS_RAMP_STOP_THRESHOLD)
//     {
//         s_ramped_omega = 0.0f;
//     }
//     else
//     {
//         s_ramped_omega = calculate_single_ramp(raw_omega, &s_ramped_omega, CHASSIS_MAX_ACCEL_OMEGA);
//     }
//     chassis_set_control->omega_set = s_ramped_omega;
// }

void chassis_ramp_speed_update(chassis_move_t *chassis_set_control)
{
    // 斜坡计算
    s_ramped_vx = calculate_single_ramp(chassis_set_control->vx_set, &s_ramped_vx, CHASSIS_MAX_ACCEL_VX);
    chassis_set_control->vx_set = s_ramped_vx;

    s_ramped_vy = calculate_single_ramp(chassis_set_control->vy_set, &s_ramped_vy, CHASSIS_MAX_ACCEL_VY);
    chassis_set_control->vy_set = s_ramped_vy;

    s_ramped_omega = calculate_single_ramp(chassis_set_control->omega_set, &s_ramped_omega, CHASSIS_MAX_ACCEL_OMEGA);
    chassis_set_control->omega_set = s_ramped_omega;
}


// 获取底盘相对于云台的相对角度（弧度制）
// float get_chassis_relative_angle_rad(void) {
//
//     // 1. 定义变量获取原始数据
//     float c_yaw, c_pitch, c_roll; // 底盘角度
//     const imu_euler_t *gimbal_imu = get_imu_euler_point(); // 云台角度指针
//
//     // 2. 获取数据
//     INS_Get_Angle(&c_yaw, &c_pitch, &c_roll); // 获取底盘Yaw
//     float chassis_yaw_deg = c_yaw;
//     float gimbal_yaw_deg  = gimbal_imu->yaw;
//
//     // 3. 计算差值 (底盘 - 云台)
//     // 含义：底盘相对于云台转了多少度
//     float relative_deg = chassis_yaw_deg - gimbal_yaw_deg;
//
//     // 4. 【关键步骤】过零点处理 (归一化到 -180 ~ 180)
//     // 修复 179 - (-179) = 358 的情况
//     if (relative_deg > 180.0f) {
//         relative_deg -= 360.0f;
//     } else if (relative_deg < -180.0f) {
//         relative_deg += 360.0f;
//     }
//
//     // 5. 角度转弧度
//     float relative_rad = relative_deg * (PI / 180.0f);
//
//     // 【核心修复1：修正零点偏差】
//     // 先把物理上的"斜"减掉
//     relative_rad = relative_rad - CHASSIS_IMU_OFFSET;
//
//     // 【核心修复2：修正旋转方向】
//     // 统一转成数学上的"逆时针为正"
//     relative_rad = relative_rad * IMU_DIRECTION_COEF;
//
//     // 再次规范化到 -PI ~ PI (防止减去偏差后越界)
//     if (relative_rad > PI) relative_rad -= 2*PI;
//     if (relative_rad < -PI) relative_rad += 2*PI;
//
//     // usart_printf("%.2f\n", relative_rad);
//
//     return relative_rad;
// }

/**
 * @brief 使用云台电机编码器获取底盘相对于云台的相对角度
 * @return float 相对弧度值，范围 [-PI, PI]
 */
float get_chassis_relative_angle_rad(void) {
    // 1. 获取云台 Yaw 电机的结构体指针
    // 注意：这里需要替换为你代码中实际存储云台电机数据的变量名
    // 假设你的变量叫 gimbal_motor_measure
    const motor_measure_t *yaw_from_gimbal_ptr = get_yaw_motor_measure_from_gimbal_point();

    // 2. 获取当前编码器值
    float current_ecd = (float)yaw_from_gimbal_ptr->ecd;

    // 3. 计算与中值的差值
    float diff = current_ecd - GIMBAL_YAW_CENTER_ECD;

    // 4. 处理编码器过零点 (8192 环形处理)
    // 确保差值在 [-4096, 4096] 之间
    if (diff > 4096.0f) {
        diff -= 8192.0f;
    } else if (diff < -4096.0f) {
        diff += 8192.0f;
    }

    // 5. 转换为弧度并应用方向系数
    float relative_rad = diff * ECD_TO_RAD * GIMBAL_DIR_COEF;

    // 6. (可选) 调试打印，确认对齐时为 0，左转变大或变小
    // usart_printf("RelRad: %.2f\n", relative_rad);

    return relative_rad;
}

/**
 * @brief 计算跟随模式下的旋转速度
 * @return float 计算出的旋转速度 omega
 */
float chassis_follow_gimbal_cal(void) {
    // 1. 获取当前相对角度 (-PI ~ PI)
    float current_relative_rad = get_chassis_relative_angle_rad();

    if (fabs(current_relative_rad) < 0.134f) {  // 0.087 rad ≈ 5度
        return 0.0f;
    }
    // 2. 计算误差 (目标 0 - 当前)
    float angle_error = 0.0f - current_relative_rad;

    // 3. 角度过零点处理 (最短路径逻辑)
    // 确保误差在 -PI 到 PI 之间
    if (angle_error > PI) {
        angle_error -= 2.0f * PI;
    } else if (angle_error < -PI) {
        angle_error += 2.0f * PI;
    }

    // 4. 死区处理 (防抖动)
    // 如果误差非常小 (比如小于 2 度)，直接输出 0
    if (fabs(angle_error) < 0.02f) { // 0.05 rad ≈ 2.8度
        return 0.0f;
    }

    // 5. 调用你的 PID 库计算
    // 技巧：因为我们已经在上面手动处理了 angle_error 的过零点逻辑，
    // 这里我们可以欺骗一下 PID，把 ref 设为 0，把 set 设为 angle_error。
    // 这样 PID 内部：error = set - ref = angle_error - 0 = angle_error
    // 这比直接传入 (current, 0) 更安全，因为 PID 库内部不懂角度突变。

    return -PID_calc(&chassis_follow_pid, 0.0f, angle_error);
}

void chassis_ramp_reset(void) {
    s_ramped_vx = 0.0f;
    s_ramped_vy = 0.0f;
    s_ramped_omega = 0.0f;
}