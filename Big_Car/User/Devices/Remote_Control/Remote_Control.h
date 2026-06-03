/**
 ******************************************************************************
 * @file    Remote_Control.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   遥控器数据处理函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 待测试
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdlib.h"

/* Defines ------------------------------------------------------------------ */
#define SBUS_RX_BUF_LEN		18u			/* SBUS接收数据长度 */

#define RC_CH_VALUE_OFFSET	1024U		/* 遥控器通道数据偏移 */

#define KEY_SET_SHORT_TIME	50U			/* 键盘置位短时间 */
#define KEY_SET_LONG_TIME	1000U		/* 键盘置位长时间 */

#define KEY_UP				0x00U		/* 键盘抬起状态 */
#define KEY_DOWN			0x01U		/* 键盘按下状态 */
/* 鼠标最大速度 */
#define MOUSE_SPEED_MAX		300U		
/* 遥控器拨杆状态 */
#define RC_SW_UP			1			/* 遥控器开关(上) */
#define RC_SW_MID			3			/* 遥控器开关(中) */
#define RC_SW_DOWN			2			/* 遥控器开关(下) */
#define RC_CH_DEADZONE		5U			/* 遥控器通道死区 */
/* 遥控器控制灵敏度 */
#define RC_CONTROL_SENSITIVITY	0.7f		
/* 遥控器拨杆 */
#define RC_SW_RIGHT			(remote_ctrl.rc.s[1])
#define RC_SW_LEFT			(remote_ctrl.rc.s[0])
/* 鼠标控制 */
#define MOUSE_X_MOVE_SPEED	(remote_ctrl.mouse.x)
#define MOUSE_Y_MOVE_SPEED	(remote_ctrl.mouse.y)
#define MOUSE_Z_MOVE_SPEED	(remote_ctrl.mouse.z)
#define MOUSE_PRESSED_LEFT	(remote_ctrl.mouse.press_l)
#define MOUSE_PRESSED_RIGHT	(remote_ctrl.mouse.press_r)
/* 键盘控制 */
#define KeyBoard_W		(remote_ctrl.key.set.W)
#define KeyBoard_S		(remote_ctrl.key.set.S)
#define KeyBoard_A		(remote_ctrl.key.set.A)
#define KeyBoard_D		(remote_ctrl.key.set.D)
#define KeyBoard_SHIFT	(remote_ctrl.key.set.SHIFT)
#define KeyBoard_CTRL	(remote_ctrl.key.set.CTRL)
#define KeyBoard_Q		(remote_ctrl.key.set.Q)
#define KeyBoard_E		(remote_ctrl.key.set.E)
#define KeyBoard_R		(remote_ctrl.key.set.R)
#define KeyBoard_F		(remote_ctrl.key.set.F)
#define KeyBoard_G		(remote_ctrl.key.set.G)
#define KeyBoard_Z		(remote_ctrl.key.set.Z)
#define KeyBoard_X		(remote_ctrl.key.set.X)
#define KeyBoard_C		(remote_ctrl.key.set.C)
#define KeyBoard_V		(remote_ctrl.key.set.V)
#define KeyBoard_B		(remote_ctrl.key.set.B)

/* Enums -------------------------------------------------------------------- */
/**
 * @brief 键盘状态枚举
 */
typedef enum
{
	UP,			/*!< 抬起 */
	SHORT_DOWN,	/*!< 短按 */
	DOWN,		/*!< 长按 */
	PRESS,		/*!< 按下 */
	RELAX,		/*!< 松开 */
	KeyBoard_Status_NUM,
}KeyBoard_Status_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 键盘信息结构体
 */
typedef struct
{
	uint16_t Count;
	KeyBoard_Status_e Status;
	KeyBoard_Status_e last_Status;
	bool last_KEY_PRESS;
	bool KEY_PRESS;
}KeyBoard_Info_Typedef;

/**
 * @brief 遥控器按键结构体
 */
typedef struct
{
	KeyBoard_Info_Typedef press_l;
	KeyBoard_Info_Typedef press_r;
	KeyBoard_Info_Typedef W;
	KeyBoard_Info_Typedef S;
	KeyBoard_Info_Typedef A;
	KeyBoard_Info_Typedef D;
	KeyBoard_Info_Typedef SHIFT;
	KeyBoard_Info_Typedef CTRL;
	KeyBoard_Info_Typedef Q;
	KeyBoard_Info_Typedef E;
	KeyBoard_Info_Typedef R;
	KeyBoard_Info_Typedef F;
	KeyBoard_Info_Typedef G;
//	KeyBoard_Info_Typedef Z;
//	KeyBoard_Info_Typedef X;
	KeyBoard_Info_Typedef C;
	KeyBoard_Info_Typedef V;
	KeyBoard_Info_Typedef B;
}Remote_Pressed_Typedef;

/**
 * @brief 遥控器信息结构体
 */
typedef  struct
{
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;

	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
//			uint16_t Z:1;
//			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;			/* 丢失标志 */
	bool rc_active[5];		/* 活动标志 */
	uint8_t online_cnt;		/* 在线计数 */
	uint32_t Last_Remote_Active_Time[5];
} Remote_Info_Typedef;

/* Externs ------------------------------------------------------------------ */
extern Remote_Info_Typedef remote_ctrl;
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_LEN];

/* Functions ---------------------------------------------------------------- */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
void Remote_Active_Detect( Remote_Info_Typedef  *remote_ctrl);
void Remote_Offline_Detect( Remote_Info_Typedef  *remote_ctrl);

/* -------------------------------------------------------------------------- */
#endif /* REMOTE_CONTROL_H */
