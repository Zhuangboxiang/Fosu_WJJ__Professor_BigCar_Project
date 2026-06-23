/**
 ******************************************************************************
 * @file    PC_Comm.h
 * @version V1.0.0
 * @date    2026.04.11
 * @brief   上位机通信驱动实现声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __PC_COMM_H
#define __PC_COMM_H

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os.h"
#include "stdbool.h"

/* Defines ------------------------------------------------------------------ */
#define PC_TX_HEAD						0xAA		/* 帧头(小车→上位机) */
#define PC_TX_FRAME_LEN					22u			/* 发送帧长度 */

#define PC_RX_HEAD						0x5A		/* 包头(上位机→小车) */
#define PC_RX_TAIL						0xA5		/* 包尾(上位机→小车) */
#define PC_RX_FRAME_LEN					15u			/* 接收帧: 包头(1)+vx(4)+vy(4)+vw(4)+校验(1)+包尾(1) */

/* Enums -------------------------------------------------------------------- */

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 数据类型转换联合体
 */
typedef union {
	float fval;			/* 浮点数值 */
	uint32_t uval;		/* 无符号整数值 */
} PC_turn_Typedef;

/**
 * @brief 发送帧结构体(小车→上位机)
 * @note  packed 防对齐, 帧头 0xAA, 固定22字节
 */
typedef struct __attribute__((packed)) {
    uint8_t HEAD;				/* 帧头: 0xAA */
    PC_turn_Typedef  Vx;		/* X方向速度 [m/s] */
    PC_turn_Typedef  Vy;		/* Y方向速度 [m/s] */
    PC_turn_Typedef  Vw;		/* Z方向角速度 [rad/s] */
    int32_t Reserved1;			/* 保留位1 */
    int32_t Reserved2;			/* 保留位2 */
    uint8_t Checksum;			/* 校验和 (前21字节异或) */
} PC_TxFrame_Typedef;

/**
 * @brief 上位机通信速度结构体(收发统一)
 */
typedef struct {
    float tx_vx;		/* 发往上位机: X方向速度 [m/s] */
    float tx_vy;		/* 发往上位机: Y方向速度 [m/s] */
    float tx_vw;		/* 发往上位机: Z方向角速度 [rad/s] */
    float rx_vx;		/* 上位机接收: X方向速度 [m/s] */
    float rx_vy;		/* 上位机接收: Y方向速度 [m/s] */
    float rx_vw;		/* 上位机接收: Z方向角速度 [rad/s] */
} PC_Speed_Typedef;

/* Externs ------------------------------------------------------------------ */
extern PC_TxFrame_Typedef PC_TxFrame;
extern uint8_t PC_TxBuf[PC_TX_FRAME_LEN];

/* Functions ---------------------------------------------------------------- */
void PC_Info_Update(uint8_t *buff, uint16_t len);
void PC_Info_Upload(float vx, float vy, float vw);
void PC_Set_Chassis_Mode(uint8_t mode);

/* -------------------------------------------------------------------------- */
#endif /* __PC_COMM_H */
