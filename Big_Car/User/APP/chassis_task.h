#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "Stepper_Motor.h"
#include "PC_Comm.h"

#define CHASSIS_WHEEL_R         0.0625f     /* 轮半径 [m] */
#define CHASSIS_WHEEL_BASE      0.30f       /* 轴距(前后) [m] */
#define CHASSIS_TRACK_WIDTH     0.50f       /* 轮距(左右) [m] */
#define CHASSIS_L               ((CHASSIS_WHEEL_BASE + CHASSIS_TRACK_WIDTH) * 0.5f)  /* 等效旋转半径 [m] */

#define CHASSIS_MAX_V           0.8f        /* 最大线速度 [m/s] */
#define CHASSIS_MAX_W           0.8f        /* 最大角速度 [rad/s] */

#define WHEEL_RAD_TO_RPM        (60.0f / (2.0f * 3.1415926f * CHASSIS_WHEEL_R))  /* 线速度→转速 [m/s → rpm] */

#define WHEEL_LF  0
#define WHEEL_RF  1
#define WHEEL_LB  2
#define WHEEL_RB  3

typedef enum
{
    CHASSIS_MODE_RC  = 0,   /* 遥控器模式 */
    CHASSIS_MODE_NAV = 1,   /* 导航模式(上位机) */
} Chassis_Mode_e;

typedef struct
{
    Stepper_Motor_Info_Typedef Motor[4];
    Chassis_Mode_e mode;			/* 控制模式 */
    float vx_target;				/* 目标速度 X [m/s] */
    float vy_target;				/* 目标速度 Y [m/s] */
    float wz_target;				/* 目标角速度 Z [rad/s] */
    PC_Speed_Typedef pc_speed;		/* 上位机通信速度(收发统一) */
    uint8_t init_flag;
} Chassis_Info_Typedef;

extern Chassis_Info_Typedef Chassis;

void chassis_task(void);
void Chassis_Set_Velocity(Chassis_Info_Typedef *chassis, float vx, float vy, float wz);
void Chassis_Stop(Chassis_Info_Typedef *chassis);
void Chassis_Init(Chassis_Info_Typedef *chassis);

#endif
