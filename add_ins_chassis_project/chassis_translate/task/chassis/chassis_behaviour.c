//
// Created by LEGION on 25-8-18.
//
#include "all_tou_h.h"
#include "remote_control.h"
#include "chassis_pid.h"
#include "INS_task.h"

// static float c_yaw,c_pitch,c_roll;
// fp32 pid_error = 0.0f;
// fp32 omega_out = 0.0f; // 最终输出给运动学解算的角速度

//变量宏定义
chassis_behaviour_all chassis_behaviour_mode = CHASSIS_ZERO_FORCE;  // 底盘当前行为模式，初始为无力模式
chassis_behaviour_all last_behaviour_mode = CHASSIS_ZERO_FORCE;     // 上一次行为模式
int8_t lastMode, nowMode;  // 用于检测模式切换的状态变量
float vx = 0.0f, vy = 0.0f, omega = 0.0f;

JoystickData js_data;   // 摇杆数据

// 函数声明
void chassis_behaviour_mode_set();

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set); // 修改声明，添加omega参数

chassis_behaviour_all detect_mode_switch(void);

void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) ;
void chassis_roll_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);
void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

void modo_test(void);

void chassis_follow_gimbal_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

fp32 angle_diff_fast(fp32 target, fp32 current);




chassis_behaviour_all detect_mode_switch(void)
{
    // 静态变量记录上一次拨杆状态
    static int last_lever1_state = 2;
    static int last_lever2_state = 2;

    parse_joystick_data();

    int current_lever1_state = js_data.lever1;
    int current_lever2_state = js_data.lever2;

    // 检查变化
    bool lever1_changed = (current_lever1_state != last_lever1_state);
    bool lever2_changed = (current_lever2_state != last_lever2_state);

    // 默认模式：保持当前生效的模式 (让 chassis_behaviour_mode_set 的全局变量决定)
    // 只有在检测到变化时，我们才更新模式。
    // 这里需要一个静态变量来存储【上一次返回的模式】
    static chassis_behaviour_all last_returned_mode = CHASSIS_ZERO_FORCE;

    chassis_behaviour_all new_mode = last_returned_mode;

    // --- 步骤 1: 确定哪个是【最新改变】并查找其模式 ---

    if (lever1_changed && !lever2_changed)
    {
        // A. 只有 Lever 1 改变了：模式由 Lever 1 的新状态决定
        if (current_lever1_state == 1) {
            new_mode = CHASSIS_ZERO_FORCE;
        } else if (current_lever1_state == 2) {
            new_mode = CHASSIS_OPEN;
        } else if (current_lever1_state == 3) {
            new_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
        }
    }
    else if (lever2_changed && !lever1_changed)
    {
        // B. 只有 Lever 2 改变了：模式由 Lever 2 的新状态决定
        if (current_lever2_state == 1) {
            new_mode = CHASSIS_ROLL_MOVE;
        } else if (current_lever2_state == 3) {
            new_mode = CHASSIS_OPEN;
        }
        // 注意：Lever 2=2 是空闲档，不应在这里被捕获并改变模式
    }
    else if (lever1_changed && lever2_changed)
    {
        // C. 同时改变：仲裁，例如 Lever 1 优先
        if (current_lever1_state == 1) {
            new_mode = CHASSIS_ZERO_FORCE;
        } else if (current_lever1_state == 2) {
            new_mode = CHASSIS_OPEN;
        } else if (current_lever1_state == 3) {
            new_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
        }
        // 如果 Lever 1 没有定义在 1, 2, 3，则检查 Lever 2
        else if (current_lever2_state == 1) {
            new_mode = CHASSIS_ROLL_MOVE;
        } else if (current_lever2_state == 3) {
            new_mode = CHASSIS_OPEN;
        }
    }

    // --- 步骤 2: 更新历史状态和返回模式 ---
    last_lever1_state = current_lever1_state;
    last_lever2_state = current_lever2_state;

    // 记录本次返回的模式，用于下一次循环的默认值
    last_returned_mode = new_mode;

    return new_mode;
}
// // 模式切换检测函数
// chassis_behaviour_all detect_mode_switch(void)
// {
//     parse_joystick_data();
//     // usart_printf("%d,%d\n",(int)js_data.lever1,(int)js_data.lever2);
//
//
//     // if (local_rc_ctrl->rc.s[0] == 1) {return CHASSIS_ZERO_FORCE;}
//     if (js_data.lever1 == 1) {return CHASSIS_ZERO_FORCE;}
//     if (js_data.lever2 == 3) {return CHASSIS_OPEN;}
//
//     if (js_data.lever1 == 2) {return CHASSIS_OPEN;}
//
//     if (js_data.lever1 == 3) {return CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;}
//
//
//     // if (local_rc_ctrl->rc.s[1] == 1) {return CHASSIS_ROLL_MOVE;}
//     if (js_data.lever2 == 1) {return CHASSIS_ROLL_MOVE;}
//     // if (local_rc_ctrl->rc.s[1] == 3) {return CHASSIS_OPEN;}
//
//     return CHASSIS_ZERO_FORCE;
//
//
//
//     // // 3. 默认返回值：如果所有拨杆状态都没有明确指定一个模式
//     return CHASSIS_ZERO_FORCE;
// }

//行为模式设定更新记录
void chassis_behaviour_mode_set() {

    // 检测新模式
    chassis_behaviour_all new_mode = detect_mode_switch();

    // 记录上一次模式
    last_behaviour_mode = chassis_behaviour_mode;

    if (new_mode != last_behaviour_mode) {
        // 如果模式发生切换，立即重置所有电机的PID状态
        chassis_ramp_reset();
        PID_3508_Reset();
        osDelay(2);

    }

    // 更新当前模式
    chassis_behaviour_mode = new_mode;
}

// 根据行为控制设设定函数
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
    switch(chassis_behaviour_mode) {
        case CHASSIS_OPEN:
            chassis_open_set_control(vx_set, vy_set, omega_set);
            break;

        case CHASSIS_ROLL_MOVE:
            chassis_roll_control(vx_set, vy_set,omega_set);
            break;
        case CHASSIS_ZERO_FORCE:
            chassis_zero_force_control(vx_set, vy_set,omega_set);
            break;
        case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:
            chassis_follow_gimbal_control(vx_set, vy_set, omega_set);
            break;
        default:
            break;
    }
}

// 普通控制
void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
    // 传递vx和vy的指针给remote_control_data_cale
    // PID_3508_Reset();
    remote_control_data_cale(200, 0,vx_set, vy_set, omega_set);

}


void chassis_roll_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) //区别于无力模式 经过pid控制速度为零
{
    // PID_3508_Reset();
    remote_control_data_cale(200, 600,vx_set, vy_set, omega_set);
}

// 无力模式
void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set)
{
    // PID_3508_Reset();
    *vx_set=0;
    *vy_set=0;
    *omega_set=0;

    // CAN_cmd_chassis(0, 0, 0, 0);
}

void chassis_follow_gimbal_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
    // 1. 数据获取
    // 假设您的函数已经准备好
    // INS_Get_Angle(&c_yaw, &c_pitch, &c_roll);//底盘imu
    // const imu_euler_t *imu = get_imu_euler_point();
    //
    // float chassis_yaw = c_yaw;//自身的绝对角度
    // float gimbal_yaw  = imu->yaw;  // 云台发来的绝对角度
    // float num = chassis_yaw-gimbal_yaw;

    // usart_printf("%.2f\n",num);

}

// void modo_test(void) {
//     float n = 0;
//     if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) {
//         n = 4;
//     }
//     if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
//         n = 3;
//     }
//     if (chassis_behaviour_mode == CHASSIS_ROLL_MOVE) {
//         n = 2;
//     }
//     if (chassis_behaviour_mode == CHASSIS_OPEN) {
//         n = 1;
//     }
//
//     usart_printf("%d,%d\n", js_data.lever2, (int)n);
// }

void modo_test(void) {
    float n = 0;

    switch (chassis_behaviour_mode) {
        case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:
            n = 4;
            break;
        case CHASSIS_ZERO_FORCE:
            n = 3;
            break;
        case CHASSIS_ROLL_MOVE:
            n = 2;
            break;
        case CHASSIS_OPEN:
            n = 1;
            break;
        default:
            // 确保所有未定义的模式都有一个默认值，例如 n=0
            n = 0;
            break;
    }

    // 打印当前 Lever 2 的状态和当前生效模式的数值代号
    // usart_printf("%d,%d\n", js_data.lever2, (int)n);
}

// /**
//   * @brief          计算两个角度之间的最小差值 (target - current)
//   * @note           假设 target 和 current 都已在 [-180, 180] 范围内
//   * @param[in]      target: 目标角度 (SP)
//   * @param[in]      current: 当前角度 (PV)
//   * @retval         最小角度差值，范围在 [-180, 180]
//   */
// fp32 angle_diff_fast(fp32 target, fp32 current) {
//     fp32 diff = target - current;
//
//     if (diff > 180.0f) {
//         diff -= 360.0f;
//     } else if (diff <= -180.0f) {
//         diff += 360.0f;
//     }
//
//     return diff;
// }
