//
// Created by LEGION on 25-9-13.
//

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

// 初始化参数
#define MAX_INIT_RETRIES 20     // 最大重试次数
#define INIT_RETRY_DELAY 50    // 重试间隔(ms)
#define INIT_STABLE_DELAY 300  // 初始化稳定时间(ms)
// #define MAX_INIT_RETRIES 20     // 最大重试次数
// #define INIT_RETRY_DELAY 50    // 重试间隔(ms)
// #define INIT_STABLE_DELAY 300  // 初始化稳定时间(ms)
//
// #define YAW_ANGLE_MIN 1400.0f
// #define YAW_ANGLE_MAX 5490.0f
// #define PITCH_ANGLE_MIN 4700.0f
// #define PITCH_ANGLE_MAX 6100.0f


extern void gimbal_task(void const *argument);


#endif //GIMBAL_TASK_H
