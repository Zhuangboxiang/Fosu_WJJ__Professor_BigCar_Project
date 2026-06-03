#include "chassis_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "usart.h"

#define CHASSIS_TASK_PERIOD_MS  10
#define RPM_TO_MOTOR(r)         ((int16_t)((r) * Stepper_Ratio))  /* 轮端RPM → 电机端RPM（减速比1:13.7） */

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
    .vx_target = 0,
    .vy_target = 0,
    .wz_target = 0,
    .init_flag = 0,
};

static void Mecanum_Wheel_Calc(Chassis_Info_Typedef *chassis, float vx, float vy, float wz);
static void Chassis_Motor_Output(Chassis_Info_Typedef *chassis);

void chassis_task(void)
{
    Chassis_Init(&Chassis);

    while (1)
    {
//        Stepper_Motor_Call_Info(&Chassis.Motor[WHEEL_LF], 2);
//        Stepper_Motor_Call_Info(&Chassis.Motor[WHEEL_RF], 2);
//        Stepper_Motor_Call_Info(&Chassis.Motor[WHEEL_LB], 2);
//        Stepper_Motor_Call_Info(&Chassis.Motor[WHEEL_RB], 2);

        Chassis_Motor_Output(&Chassis);
        osDelay(1);
    }
}

void Chassis_Init(Chassis_Info_Typedef *chassis)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        Stepper_Motor_Set_Cmd(&chassis->Motor[i], Stepper_Enable, 10u);
    }
    osDelay(500);
    chassis->init_flag = 1;
}

void Chassis_Set_Velocity(Chassis_Info_Typedef *chassis, float vx, float vy, float wz)
{
    VAL_LIMIT(vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
    VAL_LIMIT(vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
    VAL_LIMIT(wz, -CHASSIS_MAX_W, CHASSIS_MAX_W);

    chassis->vx_target = vx;
    chassis->vy_target = vy;
    chassis->wz_target = wz;
}

void Chassis_Stop(Chassis_Info_Typedef *chassis)
{
    Chassis_Set_Velocity(chassis, 0, 0, 0);
}

/**
 * @brief  麦轮逆运动学解算：vx/vy [m/s], wz [rad/s] → 四轮转速 [rpm]
 * @note   右手系：vx>0 前进, vy>0 右移, wz>0 逆时针
 */
static void Mecanum_Wheel_Calc(Chassis_Info_Typedef *chassis, float vx, float vy, float wz)
{
    float rpm[4];   /* [rpm] */

    /*  vx[m/s], vy[m/s], wz[rad/s] → 轮边线速度 [m/s] → rpm */
    rpm[WHEEL_LF] = (+vx + vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM; /* LF, [rpm] */
    rpm[WHEEL_RF] = (-vx + vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM; /* RF, [rpm] */
    rpm[WHEEL_LB] = (+vx - vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM; /* LB, [rpm] */
    rpm[WHEEL_RB] = (-vx - vy + wz * CHASSIS_L) * WHEEL_RAD_TO_RPM; /* RB, [rpm] */

    /* 机械正负号修正 + 发速度命令（加速度=0） */
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_LF], RPM_TO_MOTOR(rpm[WHEEL_LF]), 0, 3);
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_RF], RPM_TO_MOTOR(rpm[WHEEL_RF]), 0, 3);
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_LB], RPM_TO_MOTOR(rpm[WHEEL_LB]), 0, 3);
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_RB], RPM_TO_MOTOR(rpm[WHEEL_RB]), 0, 3);
}

static void Chassis_Motor_Output(Chassis_Info_Typedef *chassis)
{
    if (chassis->init_flag == 0)
        return;

    Mecanum_Wheel_Calc(chassis, chassis->vx_target, chassis->vy_target, chassis->wz_target);
}
