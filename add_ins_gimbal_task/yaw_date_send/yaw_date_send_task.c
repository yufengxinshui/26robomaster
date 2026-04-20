//
// Created by LEGION on 25-12-22.
//

#include "yaw_date_send_task.h"
#include "../../translate/all_tou_h.h"
#include "remote_control.h"
#include "yaw_date_send_task.h"

void yaw_date_send_task(void const *argument) {

    // 获取 i6x 控制数据指针
    // const i6x_ctrl_t *rc = get_i6x_point();

    const motor_measure_t *yaw_measure = get_yaw_gimbal_motor_measure_point();
    // osDelay(100);  // 等待 CAN 初始化（可选）

    for (;;) {
        CAN_send_yaw_motor_data(yaw_measure);
        // usart_printf("%d\n",yaw_measure->ecd);
        osDelay(5);  // 200Hz 发送
    }
}