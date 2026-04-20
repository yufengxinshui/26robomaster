/**
  ******************************************************************************
  * @file    can_receive.c/h
  * @brief   CAN总线电机数据接收与控制的实现文件
  * @author  DJI
  * @version V1.0.0
  * @date    2018-12-26
  * @note    本文件实现以下功能：
  *          1. CAN中断接收处理电机反馈数据
  *          2. 提供电机控制指令的发送接口
  *          3. 提供电机数据获取接口
  *          支持电机类型：
  *          - 3508底盘电机
  *          - 6020云台电机
  *          - 2006拨弹电机
  ******************************************************************************
  */

#include "CAN_receive.h"
#include "main.h"

#include "usart_printf/usart_printf_task.h"

/* 外部变量声明 -------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;  // CAN1外设句柄，用于云台电机通信
extern CAN_HandleTypeDef hcan2;  // CAN2外设句柄，用于底盘电机通信

/* 宏定义 -------------------------------------------------------------------*/
/**
  * @brief  电机数据解析宏
  * @param  ptr: 电机数据结构体指针
  * @param  data: 接收到的CAN数据数组(8字节)
  * @note   从CAN数据中解析以下电机参数：
  *         - 编码器值(ecd)
  *         - 转速(rpm)
  *         - 给定电流
  *         - 电机温度
  */
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;             /* 保存上一次编码器值 */ \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);  /* 当前编码器值(高字节在前) */ \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);  /* 电机转速(RPM) */ \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); /* 实际给定电流值 */ \
        (ptr)->temperate = (data)[6];             /* 电机温度(单位:℃) */ \
    }

/* 电机数据结构体数组 -------------------------------------------------------*/
/**
  * @brief  电机数据存储数组
  * @note   数组索引对应关系：
  *         0: 底盘电机1 (3508)
  *         1: 底盘电机2 (3508)
  *         2: 底盘电机3 (3508)
  *         3: 底盘电机4 (3508)
  *         4: 云台Yaw电机 (6020)
  *         5: 云台Pitch电机 (6020)
  *         6: 拨弹电机 (2006)
  *
  */
static motor_measure_t motor_chassis[7];

/* CAN发送相关变量 ---------------------------------------------------------*/
static CAN_TxHeaderTypeDef  gimbal_tx_message;      // 云台CAN发送报文头
static uint8_t              gimbal_can_send_data[8]; // 云台CAN发送数据缓冲区
static CAN_TxHeaderTypeDef  chassis_tx_message;     // 底盘CAN发送报文头
static uint8_t              chassis_can_send_data[8]; // 底盘CAN发送数据缓冲区

camera_signal_t camera_signal_buf = {0};
const camera_signal_t *get_camera_signal_point(void);

imu_euler_t imu_euler_buf = {0};
const imu_euler_t *get_imu_euler_point(void);


rc_can2_t rc_can2_buf = {0};
const rc_can2_t *get_rc_can2_point(void);




/* 函数实现 ---------------------------------------------------------------*/

/**
  * @brief  CAN接收中断回调函数
  * @param  hcan: 触发中断的CAN外设句柄指针
  * @retval 无
  * @note   在CAN接收到数据时自动调用，解析电机反馈数据
  */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef rx_header;  // CAN接收报文头
//     uint8_t rx_data[8];             // CAN接收数据缓冲区(8字节)
//
//     /* 从CAN接收FIFO中获取报文 */
//     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//
//     /* 根据CAN ID区分不同电机并解析数据 */
//     switch (rx_header.StdId)
//     {
//         /* 处理3508底盘电机数据 */
//         case CAN_3508_M1_ID:  // 底盘电机1
//         case CAN_3508_M2_ID:  // 底盘电机2
//         case CAN_3508_M3_ID:  // 底盘电机3
//         case CAN_3508_M4_ID:  // 底盘电机4
//         /* 处理6020云台电机数据 */
//         case CAN_YAW_MOTOR_ID:   // 云台Yaw电机
//         case CAN_PIT_MOTOR_ID:   // 云台Pitch电机
//         /* 处理2006拨弹电机数据 */
//         case CAN_TRIGGER_MOTOR_ID: // 拨弹电机
//         {
//             static uint8_t i = 0;
//             /* 计算电机索引：ID号减去底盘电机1的基准ID */
//             i = rx_header.StdId - CAN_3508_M1_ID;
//             /* 调用宏定义解析电机数据 */
//             get_motor_measure(&motor_chassis[i], rx_data);
//             break;
//         }
//
//         default:
//             /* 其他CAN ID不处理 */
//             break;
//     }
// }

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef rx_header;
//     uint8_t rx_data[8];
//
//     if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
//     {
//         // ���մ�����
//         return;
//     }
//
//     // ����CAN2���յ���Ϣ (ID = 0x301)
//     if(hcan->Instance == CAN2 && rx_header.StdId == 0x301)
//     {
//         // �������� (С�˸�ʽ)
//         camera_signal_buf.ch3_value = (rx_data[0] << 8) | rx_data[1];
//         camera_signal_buf.ch4_value = (rx_data[2] << 8) | rx_data[3];
//         camera_signal_buf.ch6_value = (rx_data[4] << 8) | rx_data[5];
//         camera_signal_buf.ch8_value = (rx_data[6] << 8) | rx_data[7];
//     }
//     else if (rx_header.StdId == 0x302)
//     {
//         camera_signal_buf.ch5_value = (rx_data[0] << 8) | rx_data[1];
//     }
//     // ԭ�еĵ�����ݴ���
//     else
//     {
//         switch (rx_header.StdId)
//         {
//             /* 处理3508底盘电机数据 */
//             case CAN_3508_M1_ID:
//             case CAN_3508_M2_ID:
//             case CAN_3508_M3_ID:
//             case CAN_3508_M4_ID:
//
//                 /* 处理6020云台电机数据 */
//             case CAN_YAW_MOTOR_ID:// 云台Yaw电机
//             case CAN_PIT_MOTOR_ID:// 云台Pitch电机
//
//                 /* 处理2006拨弹电机数据 */
//             case CAN_TRIGGER_MOTOR_ID:
//             {
//                 static uint8_t i = 0;
//                 /* 计算电机索引：ID号减去底盘电机1的基准ID */
//                 i = rx_header.StdId - CAN_3508_M1_ID;
//                 /* 调用宏定义解析电机数据 */
//                 get_motor_measure(&motor_chassis[i], rx_data);
//                 break;
//             }
//             default:
//                 /* 其他CAN ID不处理 */
//                 break;
//         }
//     }
// }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    /*========== 新增：接收欧拉角 ==========*/
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x305)
    {
        // int16_t roll_raw  = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        // int16_t pitch_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
        // int16_t yaw_raw   = (int16_t)((rx_data[4] << 8) | rx_data[5]);

        int16_t yaw_raw   = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        int16_t pitch_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
        int16_t roll_raw  = (int16_t)((rx_data[4] << 8) | rx_data[5]);

        imu_euler_buf.roll  = roll_raw  / 100.0f;
        imu_euler_buf.pitch = pitch_raw / 100.0f;
        imu_euler_buf.yaw   = yaw_raw   / 100.0f;

        // 如果需要调试输出，可以打开
        // usart_printf("%.2f,%.2f,%.2f\n", imu_euler_buf.roll, imu_euler_buf.pitch, imu_euler_buf.yaw);

        return;   // ✅ 已处理完欧拉角，不继续往下走
    }
    /*====================================*/

    /*================ 接收 RC 的 10 个数据 ================*/
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x310)
    {
        // 确保数据长度为 8 字节
        if (rx_header.DLC == 8) {
            rc_can2_buf.ch[0] = (int16_t)((rx_data[0] << 8) | rx_data[1]);
            rc_can2_buf.ch[1] = (int16_t)((rx_data[2] << 8) | rx_data[3]);
            rc_can2_buf.ch[2] = (int16_t)((rx_data[4] << 8) | rx_data[5]);
            rc_can2_buf.ch[3] = (int16_t)((rx_data[6] << 8) | rx_data[7]);
        }
        return;
    }

    if (hcan->Instance == CAN2 && rx_header.StdId == 0x311)
    {
        // 确保数据长度为 8 字节
        if (rx_header.DLC == 8) {
            rc_can2_buf.ch[4] = (int16_t)((rx_data[0] << 8) | rx_data[1]);
            rc_can2_buf.ch[5] = (int16_t)((rx_data[2] << 8) | rx_data[3]);

            rc_can2_buf.s[0] = (int8_t)rx_data[4];
            rc_can2_buf.s[1] = (int8_t)rx_data[5];
            rc_can2_buf.s[2] = (int8_t)rx_data[6];
            rc_can2_buf.s[3] = (int8_t)rx_data[7];
        }
        return;
    }
    // if (hcan->Instance == CAN2 && rx_header.StdId == 0x310)
    // {
    //     rc_can2_buf.ch[0] = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    //     rc_can2_buf.ch[1] = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    //     rc_can2_buf.ch[2] = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    //     rc_can2_buf.ch[3] = (int16_t)((rx_data[6] << 8) | rx_data[7]);
    //     return;
    // }
    //
    // if (hcan->Instance == CAN2 && rx_header.StdId == 0x311)
    // {
    //     rc_can2_buf.ch[4] = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    //     rc_can2_buf.ch[5] = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    //
    //     rc_can2_buf.s[0] = (int8_t)rx_data[4];
    //     rc_can2_buf.s[1] = (int8_t)rx_data[5];
    //     rc_can2_buf.s[2] = (int8_t)rx_data[6];
    //     rc_can2_buf.s[3] = (int8_t)rx_data[7];
    //     return;
    // }


    /*======= 原相机控制信号反馈 =======*/
    if(hcan->Instance == CAN2 && rx_header.StdId == 0x301)
    {
        camera_signal_buf.ch3_value = (rx_data[0] << 8) | rx_data[1];
        camera_signal_buf.ch4_value = (rx_data[2] << 8) | rx_data[3];
        camera_signal_buf.ch6_value = (rx_data[4] << 8) | rx_data[5];
        camera_signal_buf.ch8_value = (rx_data[6] << 8) | rx_data[7];
    }
    else if(hcan->Instance == CAN2 && rx_header.StdId == 0x302)
    {
        camera_signal_buf.ch5_value = (rx_data[0] << 8) | rx_data[1];
    }
    /*================================*/


    /*======= 原电机接收保持不变 =======*/
    else
    {
        switch (rx_header.StdId)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            case CAN_YAW_MOTOR_ID:
            case CAN_PIT_MOTOR_ID:
            case CAN_TRIGGER_MOTOR_ID:
            {
                static uint8_t i = 0;
                i = rx_header.StdId - CAN_3508_M1_ID;
                get_motor_measure(&motor_chassis[i], rx_data);
                break;
            }
            default:
                break;
        }
    }
}


/**
  * @brief  发送云台电机控制指令
  * @param  yaw:   Yaw电机控制电流，范围[-30000,30000]
  * @param  pitch: Pitch电机控制电流，范围[-30000,30000]
  * @param  shoot: 拨弹电机控制电流，范围[-10000,10000]
  * @param  rev:   保留电机控制电流
  * @retval 无
  * @note   通过CAN发送云台电机控制指令，ID为0x200+电机号
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;  // CAN发送邮箱号

    /* 配置CAN发送报文头 */
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID; // 标准ID(0x200)
    gimbal_tx_message.IDE = CAN_ID_STD;          // 标准帧
    gimbal_tx_message.RTR = CAN_RTR_DATA;        // 数据帧
    gimbal_tx_message.DLC = 0x08;               // 数据长度8字节

    /* 填充发送数据缓冲区(大端格式) */
    gimbal_can_send_data[0] = (yaw >> 8);    // Yaw电流高字节
    gimbal_can_send_data[1] = yaw;           // Yaw电流低字节
    gimbal_can_send_data[2] = (pitch >> 8);  // Pitch电流高字节
    gimbal_can_send_data[3] = pitch;         // Pitch电流低字节
    gimbal_can_send_data[4] = (shoot >> 8);  // 拨弹电机电流高字节
    gimbal_can_send_data[5] = shoot;         // 拨弹电机电流低字节
    gimbal_can_send_data[6] = (rev >> 8);    // 保留电流高字节
    gimbal_can_send_data[7] = rev;           // 保留电流低字节

    /* 将报文添加到发送邮箱 */
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief  发送底盘电机ID重置指令
  * @param  无
  * @retval 无
  * @note   发送ID为0x700的CAN报文，用于重置3508电机的ID设置
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;  // CAN发送邮箱号

    /* 配置CAN发送报文头 */
    chassis_tx_message.StdId = 0x700;    // 标准ID(0x700)
    chassis_tx_message.IDE = CAN_ID_STD;  // 标准帧
    chassis_tx_message.RTR = CAN_RTR_DATA;// 数据帧
    chassis_tx_message.DLC = 0x08;       // 数据长度8字节

    /* 填充发送数据缓冲区(全0) */
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    /* 将报文添加到发送邮箱 */
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief  发送底盘电机控制指令
  * @param  motor1: 电机1控制电流，范围[-16384,16384]
  * @param  motor2: 电机2控制电流，范围[-16384,16384]
  * @param  motor3: 电机3控制电流，范围[-16384,16384]
  * @param  motor4: 电机4控制电流，范围[-16384,16384]
  * @retval 无
  * @note   通过CAN发送底盘电机控制指令，ID为0x200+电机号
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;  // CAN发送邮箱号

    /* 配置CAN发送报文头 */
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID; // 标准ID(0x200)
    chassis_tx_message.IDE = CAN_ID_STD;           // 标准帧
    chassis_tx_message.RTR = CAN_RTR_DATA;         // 数据帧
    chassis_tx_message.DLC = 0x08;                 // 数据长度8字节

    /* 填充发送数据缓冲区(大端格式) */
    chassis_can_send_data[0] = motor1 >> 8;  // 电机1电流高字节
    chassis_can_send_data[1] = motor1;       // 电机1电流低字节
    chassis_can_send_data[2] = motor2 >> 8;  // 电机2电流高字节
    chassis_can_send_data[3] = motor2;       // 电机2电流低字节
    chassis_can_send_data[4] = motor3 >> 8;  // 电机3电流高字节
    chassis_can_send_data[5] = motor3;       // 电机3电流低字节
    chassis_can_send_data[6] = motor4 >> 8;  // 电机4电流高字节
    chassis_can_send_data[7] = motor4;       // 电机4电流低字节

    /* 将报文添加到发送邮箱 */
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


void CAN_send_camera_signal(int16_t ch3, int16_t ch4, int16_t ch6,  int16_t ch8, int16_t ch5)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8];
    uint32_t send_mailbox;

    // 第一帧：发送 ch3, ch4, ch6, ch8 (ID = 0x301)
    tx_header.StdId = 0x301;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    send_data[0] = ch3 >> 8;
    send_data[1] = ch3 & 0xFF;
    send_data[2] = ch4 >> 8;
    send_data[3] = ch4 & 0xFF;
    send_data[4] = ch6 >> 8;
    send_data[5] = ch6 & 0xFF;
    send_data[6] = ch8 >> 8;
    send_data[7] = ch8 & 0xFF;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox) != HAL_OK)
    {
        Error_Handler();
    }

    // 第二帧：发送 ch5, ch7 (ID = 0x302)
    tx_header.StdId = 0x302;
    tx_header.DLC = 4; // 只需要4字节

    send_data[0] = ch5 >> 8;
    send_data[1] = ch5 & 0xFF;

    // 剩余字节可以设置为0或用于其他数据

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_send_imu_angle(float yaw, float pitch, float roll)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8];
    uint32_t send_mailbox;

    // 转换 float → int16
    int16_t yaw_int = (int16_t)(yaw * 100);
    int16_t pitch_int = (int16_t)(pitch * 100);
    int16_t roll_int = (int16_t)(roll * 100);

    // 发送帧 ID 自定义 0x303
    tx_header.StdId = 0x303;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // 填充数据（高字节在前）
    send_data[0] = yaw_int >> 8;
    send_data[1] = yaw_int & 0xFF;
    send_data[2] = pitch_int >> 8;
    send_data[3] = pitch_int & 0xFF;
    send_data[4] = roll_int >> 8;
    send_data[5] = roll_int & 0xFF;
    send_data[6] = 0;
    send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox);
}


// void CAN_send_rc10(const i6x_ctrl_t *rc)
// {
//     CAN_TxHeaderTypeDef tx_header;
//     uint8_t data[8];
//     uint32_t mailbox;
//
//     /*================= 发送帧 1 ：ch0~ch3 =================*/
//     tx_header.StdId = 0x310;
//     tx_header.IDE = CAN_ID_STD;
//     tx_header.RTR = CAN_RTR_DATA;
//     tx_header.DLC = 8;
//
//     data[0] = rc->ch[0] >> 8;
//     data[1] = rc->ch[0] & 0xFF;
//     data[2] = rc->ch[1] >> 8;
//     data[3] = rc->ch[1] & 0xFF;
//     data[4] = rc->ch[2] >> 8;
//     data[5] = rc->ch[2] & 0xFF;
//     data[6] = rc->ch[3] >> 8;
//     data[7] = rc->ch[3] & 0xFF;
//
//     HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &mailbox);
//
//     /*================= 发送帧 2 ：ch4~ch5 + s0~s3 =================*/
//     tx_header.StdId = 0x311;
//
//     data[0] = rc->ch[4] >> 8;
//     data[1] = rc->ch[4] & 0xFF;
//     data[2] = rc->ch[5] >> 8;
//     data[3] = rc->ch[5] & 0xFF;
//     data[4] = (uint8_t)rc->s[0];
//     data[5] = (uint8_t)rc->s[1];
//     data[6] = (uint8_t)rc->s[2];
//     data[7] = (uint8_t)rc->s[3];
//
//     HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &mailbox);
// }

void CAN_send_rc_ctrl_data(const RC_ctrl_t *rc_data) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8]; // 数据缓冲区
    uint32_t send_mailbox;

    // ================== 发送第一包 (ID: 0x310) ==================
    // 包含 ch[0], ch[1], ch[2], ch[3]
    tx_header.StdId = 0x310;        // 【修改点1】改为接收端监听的 0x310
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    send_data[0] = (rc_data->rc.ch[0] >> 8) & 0xFF;
    send_data[1] = rc_data->rc.ch[0] & 0xFF;
    send_data[2] = (rc_data->rc.ch[1] >> 8) & 0xFF;
    send_data[3] = rc_data->rc.ch[1] & 0xFF;
    send_data[4] = (rc_data->rc.ch[2] >> 8) & 0xFF;
    send_data[5] = rc_data->rc.ch[2] & 0xFF;
    send_data[6] = (rc_data->rc.ch[3] >> 8) & 0xFF;
    send_data[7] = rc_data->rc.ch[3] & 0xFF;

    if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox) != HAL_OK) {
        // Error_Handler(); // 建议加上错误处理
    }

    // ================== 发送第二包 (ID: 0x311) ==================
    // 包含 ch[4], ch[5](补0), s[0], s[1]...
    // 【修改点2】为了适配接收端逻辑，必须调整字节顺序

    tx_header.StdId = 0x311; // 【修改点1】改为接收端监听的 0x311

    // ch[4] (Bytes 0-1)
    send_data[0] = (rc_data->rc.ch[4] >> 8) & 0xFF;
    send_data[1] = rc_data->rc.ch[4] & 0xFF;

    // ch[5] (Bytes 2-3) - 接收端期待这里有数据，如果你的遥控器没有ch5，就填0
    send_data[2] = 0;
    send_data[3] = 0;

    // 开关量 (Bytes 4-7) - 接收端从 Byte 4 开始读取 s[0]
    send_data[4] = rc_data->rc.s[0];
    send_data[5] = rc_data->rc.s[1];
    send_data[6] = 0; // s[2] 如果有就填
    send_data[7] = 0; // s[3] 如果有就填

    if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox) != HAL_OK) {
        // Error_Handler();
    }
}


/**
  * @brief  将Yaw轴电机数据通过CAN2发送给底盘
  * @param  ptr: 指向Yaw轴电机数据的指针
  */
void CAN_send_yaw_motor_data(const motor_measure_t *ptr)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8];
    uint32_t send_mailbox;

    tx_header.StdId = 0x304;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    send_data[0] = (ptr->ecd >> 8) & 0xFF;
    send_data[1] = ptr->ecd & 0xFF;
    send_data[2] = (ptr->speed_rpm >> 8) & 0xFF;
    send_data[3] = ptr->speed_rpm & 0xFF;
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 0;

    // ⚡️修改点：检查是否还有空闲邮箱
    // 如果没有空闲邮箱，HAL_CAN_AddTxMessage 会失败。
    // 在这里我们可以简单地等待一下（虽然不建议在中断中死等，但在任务中可以）

    // 尝试发送，如果忙则最多重试3次（简单版防止丢包）
    // 或者直接调用，但要确保总线负载不高
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox);
    }
    else
    {
        // 邮箱满！说明发得太快或者总线堵了。
        // 可以选择记录错误，或者 osDelay(1) 后重试
    }
}



/* 电机数据获取接口 -------------------------------------------------------*/
/**
  * @brief  获取Yaw电机数据指针
  * @param  无
  * @retval Yaw电机数据结构体指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];  // 返回Yaw电机(索引4)的数据指针
}

/**
  * @brief  获取Pitch电机数据指针
  * @param  无
  * @retval Pitch电机数据结构体指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];  // 返回Pitch电机(索引5)的数据指针
}

/**
  * @brief  获取拨弹电机数据指针
  * @param  无
  * @retval 拨弹电机数据结构体指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];  // 返回拨弹电机(索引6)的数据指针
}

/**
  * @brief  获取底盘电机数据指针
  * @param  i: 电机编号，范围[0,3]
  * @retval 对应底盘电机数据结构体指针
  * @note   使用位掩码确保索引在有效范围内
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];  // 通过掩码确保索引在0-3范围内
}

const camera_signal_t *get_camera_signal_point(void)
{
    return &camera_signal_buf;
}

const imu_euler_t *get_imu_euler_point(void)
{
    return &imu_euler_buf;
}

const rc_can2_t *get_rc_can2_point(void)
{
    return &rc_can2_buf;
}
