/**
 ******************************************************************************
 * @file    PC_Comm.c
 * @version V1.0.0
 * @date    2026.04.11
 * @brief   上位机通信驱动实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "PC_Comm.h"
#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "chassis_task.h"
#include <string.h>

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
PC_TxFrame_Typedef PC_TxFrame;
uint8_t PC_TxBuf[PC_TX_FRAME_LEN];

/* Static Fun -------------------------------------------------------------- */
/**
 * @brief  计算前21字节异或校验和(发送帧用)
 */
static uint8_t TX_XOR_Checksum(uint8_t *p)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < PC_TX_FRAME_LEN - 1; i++)
    {
        sum ^= p[i];
    }
    return sum;
}

/**
 * @brief  计算前13字节累加和低八位(接收帧用)
 */
static uint8_t RX_Sum_L8(uint8_t *p)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < PC_RX_FRAME_LEN - 2; i++)
    {
        sum += p[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* Functions --------------------------------------------------------------- */
/**
 * @brief  上位机→小车 数据解包 (USB接收回调中调用)
 * @param  buff: 接收数据缓冲区
 * @param  len: 本次接收长度
 *
 * @note   上位机下发格式 (15字节):
 *         包头(0x5A) + Vx(f4) + Vy(f4) + Vw(f4) + 校验和(u8,前13字节累加低八位) + 包尾(0xA5)
 */
void PC_Info_Update(uint8_t *buff, uint16_t len)
{
    if (len != PC_RX_FRAME_LEN)
        return;

    if (buff[0] != PC_RX_HEAD || buff[PC_RX_FRAME_LEN - 1] != PC_RX_TAIL)
        return;

    if (RX_Sum_L8(buff) != buff[PC_RX_FRAME_LEN - 2])
        return;

    /* 解析 vx/vy/vw → Chassis.pc_speed (小端序 float) */
    memcpy(&Chassis.pc_speed.rx_vx, &buff[1], 4);
    memcpy(&Chassis.pc_speed.rx_vy, &buff[5], 4);
    memcpy(&Chassis.pc_speed.rx_vw, &buff[9], 4);
}

/**
 * @brief  小车→上位机 数据上传 (周期调用)
 * @param  vx/vy/vw: 速度值
 *
 * @note   发送格式 (22字节):
 *         帧头(0xAA) + Vx(f4) + Vy(f4) + Vw(f4) + mode(i4) + Reserved2(i4) + Checksum(XOR u8)
 */
void PC_Info_Upload(float vx, float vy, float vw)
{
    PC_TxFrame.HEAD        = PC_TX_HEAD;
    PC_TxFrame.Vx.fval     = vx;
    PC_TxFrame.Vy.fval     = vy;
    PC_TxFrame.Vw.fval     = vw;
    PC_TxFrame.Reserved1   = (int32_t)Chassis.mode;
    PC_TxFrame.Reserved2   = 0;
    PC_TxFrame.Checksum    = TX_XOR_Checksum((uint8_t *)&PC_TxFrame);

    memcpy(PC_TxBuf, &PC_TxFrame, PC_TX_FRAME_LEN);
    if (CDC_Transmit_HS(PC_TxBuf, PC_TX_FRAME_LEN) == USBD_BUSY)
    {
        /* 上一帧未发完, 本帧丢弃 */
    }
}

/**
 * @brief  切换底盘控制模式
 * @param  mode: CHASSIS_MODE_RC(0) / CHASSIS_MODE_NAV(1)
 */
void PC_Set_Chassis_Mode(uint8_t mode)
{
    if (mode <= CHASSIS_MODE_NAV)
    {
        Chassis.mode = (Chassis_Mode_e)mode;
    }
}

/* ------------------------------------------------------------------------- */
