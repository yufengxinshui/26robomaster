//
// Created by LEGION on 25-9-16.
//

#include "can2LaunchTransmission .h"
#include "../../translate/all_tou_h.h"
#include "ix6.h"
#include "remote_control.h"

// const RC_ctrl_t *local_rc_ctrl;


void send_task(void const *argument);
void canDataSend ();

// void send_task(void const *argument) {
//     // local_rc_ctrl = get_remote_control_point();
//     // remote_control_init();
//     local_i6x_ctrl = get_i6x_point();
//     i6x_ctrl_t *rc=get_i6x_point();
//     for (;;) {
//         // canDataSend ();
//         CAN_send_i6x_ctrl(rc);
//         // usart_printf("%d,%d,%d,%d\n",rc->ch[0],rc->ch[1],rc->ch[2],rc->ch[3]);
//         osDelay(5);     // 必须要加
//     }
// }
void send_task(void const *argument) {

    // 获取 i6x 控制数据指针
    // const i6x_ctrl_t *rc = get_i6x_point();
    local_rc_ctrl = get_remote_control_point();
    const motor_measure_t *yaw_measure = get_yaw_gimbal_motor_measure_point();
    // osDelay(100);  // 等待 CAN 初始化（可选）

    for (;;) {

        // // 串口打印 10 个数据（调试用）
        // usart_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        //     rc->ch[0], rc->ch[1], rc->ch[2], rc->ch[3], rc->ch[4], rc->ch[5],
        //     rc->s[0], rc->s[1], rc->s[2], rc->s[3]);

        usart_printf("%d,%d,%d,%d,%d,%d\n",
               local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3],
               local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1]);

        //  **发送 10 个通道数据 via CAN**
        // CAN_send_rc10(rc);
        CAN_send_rc_ctrl_data(local_rc_ctrl);

        // CAN_send_yaw_motor_data(yaw_measure);
        // usart_printf("%d\n",yaw_measure->ecd);
        osDelay(5);  // 200Hz 发送
    }
}



void canDataSend () {
    sbusPreparePacket(sbus_rx_buf[0]); // 如果需要周期性处理SBUS数据，放在循环内
    CAN_send_camera_signal(SBUS_channels[3],SBUS_channels[4],SBUS_channels[6],SBUS_channels[8],SBUS_channels[5]); // 其他操作
    // usart_printf("%d,%d\n", (int)SBUS_channels[3], (int)SBUS_channels[5]); // 每次循环都打印
    osDelay(1); // 延迟1毫秒，控制打印频率
}

