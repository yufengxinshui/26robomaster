//
// Created by LEGION on 25-11-12.
//

#include "chassis_imu_date.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "usart_printf/usart_printf_task.h"

void gimbal_receive_task(void)
{
    const imu_euler_t *imu = get_imu_euler_point();
    for (;;) {
        float yaw   = imu->yaw;
        float pitch = imu->pitch;
        float roll  = imu->roll;

        // 例如打印或用于控制
        // usart_printf("%.2f,%.2f,%.2f\n", yaw, pitch, roll);
        // usart_printf("666");
        // osDelay("5");
    }
}