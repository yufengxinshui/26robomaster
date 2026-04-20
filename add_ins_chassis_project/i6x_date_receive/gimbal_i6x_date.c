//
// Created by LEGION on 25-11-13.
//

#include "gimbal_i6x_date.h"
#include "ix6.h"
#include "usart_printf_task.h"
#include "CAN_receive.h"
#include "cmsis_os.h"


void ix6_CanDate_task(void) {
    const rc_can2_t *rc_rx = get_rc_can2_point();


    // 在初始化或循环开始处获取指针
    const motor_measure_t *yaw_from_gimbal_ptr = get_yaw_motor_measure_from_gimbal_point();


    for (;;) {

        // // usart_printf("666");
        // usart_printf("%d,%d,%d,%d,%d,%d\n",
        //              rc_rx->ch[0], rc_rx->ch[1], rc_rx->ch[2], rc_rx->ch[3],
        //
        //              rc_rx->s[0], rc_rx->s[1]);
        usart_printf("%d\n",yaw_from_gimbal_ptr->ecd);
        osDelay(5);
    }
}
