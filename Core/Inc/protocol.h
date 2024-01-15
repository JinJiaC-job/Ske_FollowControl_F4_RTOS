
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "stm32f4xx.h"


#ifdef _cplusplus
extern "C" {
#endif   

/* 数据接收缓冲区大小 */
#define PROT_FRAME_LEN_RECV  128

/* 校验数据的长度 */
#define PROT_FRAME_LEN_CHECKSUM    1

/* 数据头结构体 */
typedef __packed struct
{ 
  uint16_t head;    // 包头
	uint8_t head_t;
  uint8_t ch;       // 通道
  uint8_t len;     // 包长度
  uint8_t cmd;      // 命令
  
}packet_head_t;

/* 联合体（方便数据转换） */
typedef union
{
  float f;
  int i;
}type_cast_t;

extern int16_t master_finger_angle[5][4];
extern int16_t master_finger_press[5];
extern float master_arm_angle[5];

#define FRAME_HEADER     0x4C4246    // 帧头

/* 通道宏定义 */
#define CURVES_CH1      0x01
#define CURVES_CH2      0x02
#define CURVES_CH3      0x03
#define CURVES_CH4      0x04
#define CURVES_CH5      0x05
#define CURVES_CH6      0x06
#define CURVES_CH7      0x07
#define CURVES_CH8      0x08

/* 指令(下位机 -> 上位机) */
#define SEND_TARGET_CMD      0x01     // 发送上位机通道的目标值
#define SEND_FACT_CMD        0x02     // 发送通道实际值
#define SEND_P_I_D_CMD       0x03     // 发送 PID 值（同步上位机显示的值）
#define SEND_START_CMD       0x04     // 发送启动指令（同步上位机按钮状态）
#define SEND_STOP_CMD        0x05     // 发送停止指令（同步上位机按钮状态）
#define SEND_PERIOD_CMD      0x06     // 发送周期（同步上位机显示的值）

/* 指令(上位机 -> 下位机) */
#define SET_P_I_D_CMD        0x10     // 设置 PID 值
#define SET_TARGET_CMD       0x11     // 设置目标值
#define START_CMD            0x12     // 启动指令
#define STOP_CMD             0x13     // 停止指令
#define RESET_CMD            0x14     // 复位指令
#define SET_PERIOD_CMD       0x15     // 设置周期

/* 指令无线信号传输 */
#define READ_MESSAGE_CMD     0x20     // 无线读取指令
#define SEND_MESSAGE_CMD     0x21     // 无线发送指令

/* 空指令 */
#define CMD_NONE             0xFF     // 空指令

/* 索引值宏定义 */
//#define HEAD_INDEX_VAL       0x2u     // 包头索引值（3字节）
#define CHX_INDEX_VAL        0x3u     // 通道索引值（1字节）
#define LEN_INDEX_VAL        0x4u     // 包长索引值（1字节）
#define CMD_INDEX_VAL        0x5u     // 命令索引值（1字节）

#define EXCHANGE_H_L_BIT(data)      ((((data) << 24) & 0xFF000000) |\
                                     (((data) <<  8) & 0x00FF0000) |\
                                     (((data) >>  8) & 0x0000FF00) |\
                                     (((data) >> 24) & 0x000000FF))     // 交换高低字节

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))      // 合成为一个字

#define COMPOUND_64BIT(data)        (((*(data-0) << 56) & 0xFF00000000000000) |\
                                     ((*(data-1) << 48) & 0x00FF000000000000) |\
                                     ((*(data-2) << 40) & 0x0000FF0000000000) |\
                                     ((*(data-3) << 32) & 0x000000FF00000000) |\
                                     ((*(data-4) << 24) & 0x00000000FF000000) |\
                                     ((*(data-5) << 16) & 0x0000000000FF0000) |\
                                     ((*(data-6) <<  8) & 0x000000000000FF00) |\
                                     ((*(data-7) <<  0) & 0x00000000000000FF))      // 合成为一个字
  

#define COMPOUND_16BIT(data)        (((*(data-0) << 8) & 0xFF00) |\
                                     ((*(data-1) << 0) & 0x00FF))
/**
 * @brief   接收数据处理
 * @param   *data:  要计算的数据的数组.
 * @param   data_len: 数据的大小
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len);

/**
 * @brief   初始化接收协议
 * @param   void
 * @return  初始化结果.
 */
int32_t protocol_init(void);

/**
 * @brief   接收的数据处理
 * @param   void
 * @return  -1：没有找到一个正确的命令.
 */
int8_t receiving_process(void);

/**
  * @brief 设置上位机的值
  * @param cmd：命令
  * @param ch: 曲线通道
  * @param data：参数指针
  * @param num：参数个数
  * @retval 无
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num);
void set_computer_32value(uint8_t cmd, uint8_t ch, void *data, uint8_t num);

extern uint8_t receivech;
extern type_cast_t target[20];
//extern uint32_t target;
extern uint8_t i;

#ifdef _cplusplus
}
#endif   

#endif
