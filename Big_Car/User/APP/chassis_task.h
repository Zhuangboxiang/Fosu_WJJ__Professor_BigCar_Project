#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "Stepper_Motor.h"

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

typedef struct
{
    Stepper_Motor_Info_Typedef Motor[4];
    float vx_target;
    float vy_target;
    float wz_target;
    uint8_t init_flag;
} Chassis_Info_Typedef;

extern Chassis_Info_Typedef Chassis;

void chassis_task(void);
void Chassis_Set_Velocity(Chassis_Info_Typedef *chassis, float vx, float vy, float wz);
void Chassis_Stop(Chassis_Info_Typedef *chassis);
void Chassis_Init(Chassis_Info_Typedef *chassis);

#endif
