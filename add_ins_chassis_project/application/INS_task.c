/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      使用 BMI088 计算欧拉角（不启用 IST8310 的 data ready 中断以节省 CPU）
  *             - 使用 BMI088 的 DATA READY 中断触发 SPI DMA 传输以读取传感器数据
  *             - 通过 DMA 做 SPI 收发以减轻 CPU 负担
  *             - 包含对 BMI088 加热器（温控 PWM）的 PID 控制逻辑
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  ******************************************************************************/
#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "MahonyAHRS.h"
#include "math.h"
#include "usart_printf_task.h"

/* 宏：把 PWM 设置封装，便于在代码中直接用 IMU_temp_PWM(x) 控制加热器占空比 */
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)    // 调用底层 PWM 设置函数

/* 函数原型（文件内静态函数） */
static void imu_temp_control(fp32 temp);      // BMI088 温度控制（内部使用）
static void imu_cmd_spi_dma(void);            // 根据更新标志选择合适的 SPI DMA 读传感器

/* AHRS 接口封装（外部调用 MahonyAHRS） */
void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;               // SPI1 在 HAL 中的句柄（在 main.c 或 stm32f4xx_hal_msp 中配置）
static TaskHandle_t INS_task_local_handler;   // 本任务的 FreeRTOS 任务句柄，用于 ISR 唤醒任务

/* --------------------------- DMA 缓冲区定义 ---------------------------
 * 这里的 tx_buf 在每次 SPI DMA 启动时会被发送出去（一般是读取寄存器指令 + 填充 0xFF）
 * rx_buf 用于接收从设备返回的数据（DMA 会填充）
 * SPI_DMA_*_LENGHT 是对应的缓冲区长度常量（在头文件中定义）
 * ------------------------------------------------------------------*/
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] =
    {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // 0x82: 假设为读取 gyro 起始寄存器的命令（视驱动设定）

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] =
    {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // 0x92: accel 读取命令

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF}; // temp 读取命令

/* --------------------------- 更新标志（位域风格） ---------------------------
 * 使用多个 volatile 标志位变量来表示各个传感器、DMA、notify 的状态。
 * 这些变量被 ISR（外部中断）和 DMA 完成中断及任务上下文共同访问，因此需注意原子性。
 *
 * 约定（代码中常用的位偏移常量见头文件）:
 *  - IMU_DR_SHFITS: 表示该传感器 DATA READY 中断发生（需要启动 SPI DMA）
 *  - IMU_SPI_SHFITS: 表示该传感器正在通过 SPI DMA 传输（SPI BUS 占用中）
 *  - IMU_UPDATE_SHFITS: 表示该传感器的 DMA 已完成，数据可由上层处理（task 中解析）
 *  - IMU_NOTIFY_SHFITS: 表示发送通知给 INS_task（唤醒进行数据解析）
 * ------------------------------------------------------------------*/
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0; // 任务初始化后置 1，允许 DMA 被 ISR 启动

/* 传感器数据结构：实际数据会通过 BMI088/IST8310 驱动解析后填入这些结构 */
bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;

/* 温控 PID：控制 BMI088 的加热器（保持目标温度，防止温漂）
 * first_temperate: 初始阶段标志（发动机还未稳定到目标温度）
 */
static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

/* AHRS / 角度相关数据：
 * - INS_quat: 四元数（q0..q3）
 * - INS_angle: 欧拉角 yaw/pitch/roll（单位：度）
 */
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      // euler angle, 单位 degree（代码内部 get_angle 转换为 57.3）

/**
  * @brief  IMU 任务：初始化 BMI088、IST8310，并按中断 + DMA 流程读取数据、运行 AHRS 计算欧拉角
  * @param  pvParameters: 任务输入参数（这里为 NULL）
  * @retval none
  *
  * 任务总体逻辑：
  *  1. 延时等待系统稳定（INS_TASK_INIT_TIME）
  *  2. 初始化 BMI088 与 IST8310（循环直到成功）
  *  3. 读取一次初始数据以初始化 AHRS
  *  4. 初始化 PID 温控器
  *  5. 配置 SPI 波特率并重新初始化 SPI（把 SPI 频率调整到需要的值）
  *  6. 使用 SPI DMA 初次配置（为 gyro DMA），并将 imu_start_dma_flag 置位，允许 ISR 启动 DMA
  *  7. 主循环：等待任务通知（由 DMA 完成或 ISR 触发），解析接收缓冲区，调用 AHRS 更新并输出角度
  */
void INS_task(void const *pvParameters)
{
    // 1) 等待系统其他部分就绪
    osDelay(INS_TASK_INIT_TIME);

    // 2) 循环初始化 BMI088，直到成功返回（驱动内部处理 I2C/SPI 等）
    while (BMI088_init())
    {
        osDelay(100); // 初始化失败则每 100ms 再试一次
    }

    // 3) 循环初始化 IST8310（磁力计）
    while (ist8310_init())
    {
        osDelay(100);
    }

    // 4) 读取一次传感器数据以获得初始 accel/gyro/temp（为 AHRS 初始值）
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    // 5) 初始化温控 PID（位置式 PID）
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    // 6) 使用当前读取到的 accel/mag 初始化 AHRS 四元数（Mahony）
    AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);

    // 7) 获取当前任务句柄（用于 ISR 中唤醒）
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    // 8) 调整 SPI 时钟分频（根据 BMI088 的最大 SPI 频率选择合适 Prescaler）
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler(); // SPI 初始化失败则进入错误处理（一般为死循环或复位）
    }

    // 9) 为 SPI1/DMA 做一次静态初始化（传入默认 buffer），便于之后由 DMA 启动函数激活
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1; // 允许 ISR 启动 DMA（之前禁止，防止在初始化阶段触发）

    /* 主循环：
     *  - 通过任务通知（ulTaskNotifyTake）等待唤醒（ISR 会调用 vTaskNotifyGiveFromISR 或者通过软件中断触发）
     *  - 根据各个 *_update_flag 的 IMU_UPDATE_SHFITS 位解析相应缓冲区
     *  - 调用 AHRS_update 并把四元数转为角度
     */
    while (1)
    {
        // 等待通知（阻塞式），只有收到通知才继续。portMAX_DELAY 表示无限等待。
        // ulTaskNotifyTake 会在其他地方（ISR）调用 vTaskNotifyGiveFromISR 唤醒
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
            // 这里是保险性循环，一般不会多次回到此处
        }

        /* 解析 gyro 数据：当 gyro_update_flag 的 IMU_NOTIFY_SHFITS 位被设置时表示 DMA + 中断通知已完成 */
        if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            // 清除 notify 位（原子操作不是必需的，但标志量是 volatile）
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);

            // 调用驱动层函数解析从 DMA RX 缓冲区（偏移 BMI088_GYRO_RX_BUF_DATA_OFFSET 处开始是有用数据）
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        /* 解析 accel 数据：同上 */
        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                   bmi088_real_data.accel, &bmi088_real_data.time);
        }

        /* 解析温度数据（来自 accel 的 temp 寄存器读取）并做温控 */
        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                         &bmi088_real_data.temp);

            // 使用 PID 控制加热器 PWM 以保持目标温度
            imu_temp_control(bmi088_real_data.temp);
        }

        /* 运行 AHRS 更新函数（使用 Mahony 实现）
         * 注意：time 传入 0.001f（1ms） —— 这是这里的采样周期假设，需与实际中断频率一致
         */
        AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);

        /* 把四元数转换为欧拉角（度） */
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET,
                  INS_angle + INS_PITCH_ADDRESS_OFFSET,
                  INS_angle + INS_ROLL_ADDRESS_OFFSET);

        /* 输出角度数据到串口（调试用） */
        // usart_printf("%.2f,%.2f,%.2f\n", INS_angle[0], INS_angle[1], INS_angle[2]);
    }
}


void INS_Get_Angle(fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = bmi088_real_data.gyro[0];
    *pitch = bmi088_real_data.gyro[1];
    *roll = bmi088_real_data.gyro[2];
}


/* --------------------------- AHRS 接口实现 --------------------------- */
/* 这里封装对四元数的初始化与更新的简单调用，便于未来替换算法 */

/**
 * @brief  AHRS init: 把四元数设置为单位四元数（无旋转）
 * @param  quat: 长度 4 的数组，按 q0 q1 q2 q3 存放
 */
void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

/**
 * @brief  AHRS update: 调用 MahonyAHRS 更新四元数
 * @param  quat: 四元数数组（输入输出）
 * @param  time: 采样时间间隔（秒）（此实现中未直接使用 time，而是在 Mahony 内部使用固定滤波率）
 * @param  gyro: 角速度（单位：？？），需与 Mahony 函数所需单位一致（通常 rad/s）
 * @param  accel: 加速度（单位：g）
 * @param  mag: 磁力计（单位：uT）
 */
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    // MahonyAHRSupdate 内部会更新 quat 数组（基于传入的传感器数据）
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
                     accel[0], accel[1], accel[2],
                     mag[0], mag[1], mag[2]);
}

/**
 * @brief  将四元数转换为 Euler 角（度）
 * @param  q: 四元数
 * @param  yaw, pitch, roll: 输出对应角度指针（单位：度）
 *
 * 注意：返回值采用角度制（通过乘以 57.3 将弧度转度）
 * 公式来源：常见四元数->欧拉角转换（注意坐标系和符号约定）
 */
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    // yaw (Z axis rotation)
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.3f;

    // pitch (Y axis rotation)
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.3f;

    // roll (X axis rotation)
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.3f;
}

/* --------------------------- 温控 PID 实现 ---------------------------
 * imu_temp_control: 根据当前温度控制 BMI088 的加热器 PWM。
 *
 * 逻辑说明：
 *  - 当 first_temperate == 0（初始尚未稳温），会一直把 PWM 打到最大，直到检测到温度达到某一阈值并稳定一段时间后，
 *    把 first_temperate 置 1 并启用 PID 控制。此处的 temp_constant_time 用来统计温度稳定持续时间，避免瞬态误判。
 *  - PID_calc 使用的是位置式 PID，控制目标为 45.0f（目标温度），输出被限制在设置的最大 PWM 范围内。
 *
 * 参数与限制：
 *  - temp: 传感器读回的温度（单位与驱动约定，一般为摄氏度）
 * ------------------------------------------------------------------*/
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;

    if (first_temperate)
    {
        // 正常工作阶段：使用 PID 计算输出占空比
        PID_calc(&imu_temp_pid, temp, 45.0f); // 将当前温度与目标 45℃ 送入 PID

        // PID 输出下限保护（防止负输出）
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }

        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM); // 将 PID 输出转换为 PWM 占空比写入硬件
    }
    else
    {
        // 初始阶段：直接给最大功率，加速升温，直到温度稳定（防止启动阶段被 PID 误控制）
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // 当检测到温度持续高于阈值超过一定次数（这里 200 次，具体时间取决于采样频率）
                // 将初始标志置为已稳定，开启 PID 控制
                // 注意这里注释中原作者似乎打算把 first_temperate 置 1，但代码中缺少赋值（原注释里有但被注释掉）
                // 我们在这里保持不改原逻辑，只初始化 PID Iout 为一个中间值以避免扳机突变
                // 如果你想真正把 first_temperate 置 1，可在此取消注释并赋值：
                // first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f; // 给 Iout 一个初值，减小突变
            }
        }

        // 初始阶段保持最大占空比（或接近最大）
        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/* --------------------------- 外部中断回调（GPIO EXTI） ---------------------------
 * 本回调由 HAL 库在 GPIO 引脚产生中断时调用（在 stm32xx_it.c 或 HAL 层链入）。
 *
 * 约定：
 *  - INT1_ACCEL_Pin: BMI088 accel 的 data ready 引脚
 *  - INT1_GYRO_Pin: BMI088 gyro 的 data ready 引脚
 *  - DRDY_IST8310_Pin: IST8310 磁力计 data ready
 *  - GPIO_PIN_0: 软件中断（在 DMA 完成后生成，用于唤醒任务）
 *
 * 处理流程：
 *  1) 加速计/陀螺仪 data ready：在对应的标志上设置 IMU_DR_SHFITS，若允许 DMA 启动（imu_start_dma_flag），则调用 imu_cmd_spi_dma
 *  2) 磁力计 data ready：读取磁力计（ist8310_read_mag）或设置标志
 *  3) GPIO_PIN_0：这是任务唤醒用的软件中断，调用 vTaskNotifyGiveFromISR 唤醒 INS_task
 *
 * 注意：
 *  - 在 ISR 中尽量避免耗时操作（例如解析数据），这里 ISR 只设置标志并在必要时启动 DMA 或直接读取轻量数据（磁力计）。
 *  - 对于磁力计，我们在此处直接做一个阻塞式读取（ist8310_read_mag），如果该函数耗时较久，建议改成在任务上下文中读取或使用异步方式。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        /* accel data ready：设置 DR 位并准备 SPI DMA */
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS; // accel 同时触发温度读取

        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma(); // 在 ISR 内尝试启动 SPI DMA（该函数会处理竞态与 DMA 占用检查）
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        /* gyro data ready */
        gyro_update_flag |= 1 << IMU_DR_SHFITS;

        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == DRDY_IST8310_Pin)
    {
        /* ist8310 磁力计 data ready */
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        /* 注意：原代码中有一个看起来像笔误的赋值 if(mag_update_flag &= 1 << IMU_DR_SHFITS)
         * 这会把 mag_update_flag 改写为只有该位的值并返回真/假。正确的意图很可能是判断该位是否置位。
         * 在此保留原行为但同时修正为更清晰的写法（判断语句采用 & 而不是 &=）。
         */
        if (mag_update_flag & (1 << IMU_DR_SHFITS))
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);  // 清除 DR 位
            mag_update_flag |= (1 << IMU_SPI_SHFITS);  // 表示磁力计正在 SPI 传输（如需要）

            /* 直接读取磁力计数据（阻塞或非阻塞取决于驱动实现）：
             * 如果 ist8310_read_mag 内部使用 I2C 或 SPI 同步传输且耗时较长，
             * 在 ISR 中执行会阻塞，建议将读取改到任务线程中执行。
             * 这里保持原来实现（直接在 ISR 中读取）。
             */
            ist8310_read_mag(ist8310_real_data.mag);
        }
    }
    else if (GPIO_Pin == GPIO_PIN_0)
    {
        /* 软件中断：用于从 ISR 唤醒 INS_task（由 DMA 完成时触发） */
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* --------------------------- 基于标志的 SPI DMA 启动逻辑 ---------------------------
 * imu_cmd_spi_dma() 在 ISR 中被调用（当某个传感器触发 DR 中断时），通过检查各个标志与 DMA 状态来决定：
 *  - 哪个 sensor 的 DMA 优先级高（一般先 gyro 再 accel 再 temp）
 *  - 只有当当前 SPI 的 TX/RX DMA 通道未使能时（即 SPI 空闲）才启动下一次 DMA
 *
 * 设计要点与注意：
 *  - 该函数在 ISR 中调用（taskENTER_CRITICAL_FROM_ISR），因此需要用 ISR 版本的临界区
 *  - 检查 DMA 的方式是读取 HAL 的 DMA CR 寄存器判断 DMA 是否正在工作：
 *      !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
 *    表示 TX 或 RX DMA 通道没有被使能
 *  - 启动 DMA 前要根据对应的片选（CS）拉低，DMA 完成后在 DMA IRQ 中拉高 CS
 */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR(); // 进入临界段（ISR 版本）

    /* 情形 1：gyro 优先启动（当有 gyro DR 且 SPI 空闲且 accel/temp 未占用 SPI） */
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        // 从 DR -> SPI 状态迁移
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        // 选中 gyro CS（拉低）并启动 DMA
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus); // 退出临界段
        return;
    }

    /* 情形 2：accel 优先（如果 accel 有 DR 且 SPI 空闲且 gyro 未占用 SPI） */
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    /* 情形 3：accel_temp（温度寄存器）读取，优先级最低 */
    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus); // 若没有条件满足则退出临界段
}

/* --------------------------- DMA IRQ Handler ---------------------------
 * 当 DMA RX 完成（传输完成标志）时，该中断会调用。通常这个 IRQ 是由 SPI RX 的 DMA 流触发。
 *
 * 流程：
 *  1) 检查 RX 完成标志并清除
 *  2) 根据哪个 SPI_TRANSFER 状态被占用（gyro/accel/accel_temp 的 IMU_SPI_SHFITS），把该状态转换为 IMU_UPDATE_SHFITS
 *     并拉高对应的 CS（结束 SPI 传输）
 *  3) 再次调用 imu_cmd_spi_dma() 尝试启动下一个等待的传输（实现链式传输）
 *  4) 如果有数据准备好（IMU_UPDATE_SHFITS），把它转换为 IMU_NOTIFY_SHFITS 并通过软件中断触发任务唤醒
 *
 * 注意：
 *  - 这里使用 __HAL_DMA_GET_FLAG 与 __HAL_DMA_CLEAR_FLAG 等 HAL 宏来检查/清除 DMA 传输完成标志
 *  - 触发任务唤醒使用 __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0)，这是一个软件触发的外部中断，随后会在 HAL_GPIO_EXTI_Callback 里处理唤醒逻辑
 */
void DMA2_Stream2_IRQHandler(void)
{
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        // 清除 DMA 传输完成标志
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        /* 如果是 gyro 的 SPI 传输刚完成（IMU_SPI_SHFITS 被置位），切换到 IMU_UPDATE_SHFITS */
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            // 结束 gyro CS（拉高），释放总线
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        /* accel 传输完成处理 */
        if (accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        /* accel temperature 传输完成处理 */
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        /* 尝试启动下一个等待的 SPI DMA（链式传输），例如：gyro->accel->temp */
        imu_cmd_spi_dma();

        /* 如果有任何数据已迁移到 IMU_UPDATE_SHFITS（表示缓冲区可解析），将其映射为 NOTIFY 并通过软件中断唤醒任务 */
        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);

            // 通过模拟 EXTI 触发来唤醒任务（会在 HAL_GPIO_EXTI_Callback 中调用 vTaskNotifyGiveFromISR）
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
