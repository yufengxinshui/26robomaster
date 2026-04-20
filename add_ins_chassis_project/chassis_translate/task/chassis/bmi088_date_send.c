#include "CAN_receive.h"
#include "INS_task.h"
#include "all_tou_h.h"
static float yaw,pitch,roll;

void test_print_euler(void);

void imu_date_send_task(void const *argument) {
    for (;;) {
        test_print_euler();
        // usart_printf("666");
        osDelay(5);  // 强烈建议：200Hz 足够了
   }
}



void test_print_euler(void)
{
    INS_Get_Angle(&yaw, &pitch, &roll);

    CAN_send_imu_angle(yaw,pitch,roll);

    // usart_printf("%.2f\n",yaw);
}