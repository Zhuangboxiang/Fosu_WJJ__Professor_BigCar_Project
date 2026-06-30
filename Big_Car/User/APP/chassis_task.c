/**
 ******************************************************************************
 * @file    chassis_task.c
 * @brief   底盘控制任务
 *
 *  架构说明:
 *  - 每种控制模式对应一个独立的 Mode_Handler 函数
 *  - 主循环通过 Chassis_Mode_Dispatch 根据当前模式分派到对应 handler
 *  - 速度来源: RC模式由PS2任务设置 | NAV模式由上位机设置
 *
 *  +------------------+      +-----------------------+
 *  |  chassis_task    | ---> | Chassis_Mode_Dispatch |
 *  +------------------+      +-----------------------+
 *                                     |
 *              +----------------------+----------------------+
 *              |                      |                      |
 *     CHASSIS_MODE_RC       CHASSIS_MODE_NAV          (default)
 *              |                      |                      |
 *     RC_Mode_Handler       NAV_Mode_Handler        Chassis_Stop
 *   (PS2任务直接设速)     (上位机速度→SetSpeed)   (安全停车)
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "PC_Comm.h"

/* Private define ------------------------------------------------------------*/
#define RPM_TO_MOTOR(r)  ((int16_t)((r) * Stepper_Ratio))

/* Global variable -----------------------------------------------------------*/
Chassis_Info_Typedef Chassis = {
    .Motor[WHEEL_LF] = {
        .Set.motor_Addr = 1,
        .Set.Firmware_v = Firmware_Emm,
    },
    .Motor[WHEEL_RF] = {
        .Set.motor_Addr = 2,
        .Set.Firmware_v = Firmware_Emm,
    },
    .Motor[WHEEL_LB] = {
        .Set.motor_Addr = 3,
        .Set.Firmware_v = Firmware_Emm,
    },
    .Motor[WHEEL_RB] = {
        .Set.motor_Addr = 4,
        .Set.Firmware_v = Firmware_Emm,
    },
    .mode       = CHASSIS_MODE_RC,
    .vx_target  = 0,
    .vy_target  = 0,
    .wz_target  = 0,
    .pc_speed   = {0},
    .init_flag  = 0,
};

/* Private function prototypes -----------------------------------------------*/
static void Chassis_RC_Mode_Handler(Chassis_Info_Typedef *chassis);
static void Chassis_NAV_Mode_Handler(Chassis_Info_Typedef *chassis);
static void Chassis_Mode_Dispatch(Chassis_Info_Typedef *chassis);
static void Chassis_Motor_Output(Chassis_Info_Typedef *chassis);
static void Mecanum_Wheel_Calc(Chassis_Info_Typedef *chassis, float vx, float vy, float wz);

/* ---------------------------------------------------------------------------*/
/*                            Public Functions                                */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  底盘控制任务入口
 */
void chassis_task(void)
{
    Chassis_Init(&Chassis);

    while (1)
    {
        Chassis_Mode_Dispatch(&Chassis);                         /* 按模式分发控制逻辑 */
        Chassis_Motor_Output(&Chassis);                          /* 统一输出到电机 */
        PC_Info_Upload(Chassis.vx_target, Chassis.vy_target, Chassis.wz_target);

        osDelay(1);
    }
}

/**
 * @brief  底盘初始化: 使能四路电机
 */
void Chassis_Init(Chassis_Info_Typedef *chassis)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        Stepper_Motor_Set_Cmd(&chassis->Motor[i], Stepper_Enable, 10u);
    }
    osDelay(500);
    chassis->init_flag = 1;
}

/**
 * @brief  设置底盘目标速度 (带限幅)
 */
void Chassis_Set_Velocity(Chassis_Info_Typedef *chassis, float vx, float vy, float wz)
{
    VAL_LIMIT(vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
    VAL_LIMIT(vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
    VAL_LIMIT(wz, -CHASSIS_MAX_W, CHASSIS_MAX_W);

    chassis->vx_target = vx;
    chassis->vy_target = vy;
    chassis->wz_target = wz;
}

/**
 * @brief  底盘急停
 */
void Chassis_Stop(Chassis_Info_Typedef *chassis)
{
    Chassis_Set_Velocity(chassis, 0, 0, 0);
}

/* ---------------------------------------------------------------------------*/
/*                          Mode Dispatch Layer                               */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  根据当前模式分派到对应的模式处理函数
 * @note   新增控制模式时只需: 1)添加 case 分支  2)实现对应的 Handler
 */
static void Chassis_Mode_Dispatch(Chassis_Info_Typedef *chassis)
{
    switch (chassis->mode)
    {
        case CHASSIS_MODE_RC:
            Chassis_RC_Mode_Handler(chassis);
            break;

        case CHASSIS_MODE_NAV:
            Chassis_NAV_Mode_Handler(chassis);
            break;

        default:
            Chassis_Stop(chassis);
            break;
    }
}

/* ---------------------------------------------------------------------------*/
/*                         Mode Handler Functions                             */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  RC 遥控器模式 —— 速度由 PS2 任务通过 Chassis_Set_Velocity() 设置
 */
static void Chassis_RC_Mode_Handler(Chassis_Info_Typedef *chassis)
{
    (void)chassis;
    /* 无额外处理: PS2 任务已直接写入 vx_target/vy_target/wz_target */
}

/**
 * @brief  NAV 导航模式 —— 读取上位机下发速度并应用到底盘
 */
static void Chassis_NAV_Mode_Handler(Chassis_Info_Typedef *chassis)
{
    Chassis_Set_Velocity(chassis,
                         chassis->pc_speed.rx_vx,
                         chassis->pc_speed.rx_vy,
                         chassis->pc_speed.rx_vw);
}

/* ---------------------------------------------------------------------------*/
/*                          Motor Output Layer                                */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  输出电机控制指令 (初始化完成前不输出)
 */
static void Chassis_Motor_Output(Chassis_Info_Typedef *chassis)
{
    if (chassis->init_flag == 0)
        return;

    Mecanum_Wheel_Calc(chassis, chassis->vx_target, chassis->vy_target, chassis->wz_target);
}

/**
 * @brief  麦轮逆运动学解算: vx/vy [m/s], wz [rad/s] → 四轮转速 [rpm]
 * @note   右手系: vx>0 前进,  vy>0 右移,  wz>0 逆时针
 */
static void Mecanum_Wheel_Calc(Chassis_Info_Typedef *chassis, float vx, float vy, float wz)
{
    float rpm[4];

    rpm[WHEEL_LF] = (+vx + vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM;
    rpm[WHEEL_RF] = (-vx + vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM;
    rpm[WHEEL_LB] = (+vx - vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM;
    rpm[WHEEL_RB] = (-vx - vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM;

    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_LF], RPM_TO_MOTOR(rpm[WHEEL_LF]), 0, 3);
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_RF], RPM_TO_MOTOR(rpm[WHEEL_RF]), 0, 3);
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_LB], RPM_TO_MOTOR(rpm[WHEEL_LB]), 0, 3);
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_RB], RPM_TO_MOTOR(rpm[WHEEL_RB]), 0, 3);
}
