
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "stm32f4xx.h"


#ifdef _cplusplus
extern "C" {
#endif   

/* ���ݽ��ջ�������С */
#define PROT_FRAME_LEN_RECV  128

/* У�����ݵĳ��� */
#define PROT_FRAME_LEN_CHECKSUM    1

/* ����ͷ�ṹ�� */
typedef __packed struct
{ 
  uint16_t head;    // ��ͷ
	uint8_t head_t;
  uint8_t ch;       // ͨ��
  uint8_t len;     // ������
  uint8_t cmd;      // ����
  
}packet_head_t;

/* �����壨��������ת���� */
typedef union
{
  float f;
  int i;
}type_cast_t;

extern int16_t master_finger_angle[5][4];
extern int16_t master_finger_press[5];
extern float master_arm_angle[5];

#define FRAME_HEADER     0x4C4246    // ֡ͷ

/* ͨ���궨�� */
#define CURVES_CH1      0x01
#define CURVES_CH2      0x02
#define CURVES_CH3      0x03
#define CURVES_CH4      0x04
#define CURVES_CH5      0x05
#define CURVES_CH6      0x06
#define CURVES_CH7      0x07
#define CURVES_CH8      0x08

/* ָ��(��λ�� -> ��λ��) */
#define SEND_TARGET_CMD      0x01     // ������λ��ͨ����Ŀ��ֵ
#define SEND_FACT_CMD        0x02     // ����ͨ��ʵ��ֵ
#define SEND_P_I_D_CMD       0x03     // ���� PID ֵ��ͬ����λ����ʾ��ֵ��
#define SEND_START_CMD       0x04     // ��������ָ�ͬ����λ����ť״̬��
#define SEND_STOP_CMD        0x05     // ����ָֹͣ�ͬ����λ����ť״̬��
#define SEND_PERIOD_CMD      0x06     // �������ڣ�ͬ����λ����ʾ��ֵ��

/* ָ��(��λ�� -> ��λ��) */
#define SET_P_I_D_CMD        0x10     // ���� PID ֵ
#define SET_TARGET_CMD       0x11     // ����Ŀ��ֵ
#define START_CMD            0x12     // ����ָ��
#define STOP_CMD             0x13     // ָֹͣ��
#define RESET_CMD            0x14     // ��λָ��
#define SET_PERIOD_CMD       0x15     // ��������

/* ָ�������źŴ��� */
#define READ_MESSAGE_CMD     0x20     // ���߶�ȡָ��
#define SEND_MESSAGE_CMD     0x21     // ���߷���ָ��

/* ��ָ�� */
#define CMD_NONE             0xFF     // ��ָ��

/* ����ֵ�궨�� */
//#define HEAD_INDEX_VAL       0x2u     // ��ͷ����ֵ��3�ֽڣ�
#define CHX_INDEX_VAL        0x3u     // ͨ������ֵ��1�ֽڣ�
#define LEN_INDEX_VAL        0x4u     // ��������ֵ��1�ֽڣ�
#define CMD_INDEX_VAL        0x5u     // ��������ֵ��1�ֽڣ�

#define EXCHANGE_H_L_BIT(data)      ((((data) << 24) & 0xFF000000) |\
                                     (((data) <<  8) & 0x00FF0000) |\
                                     (((data) >>  8) & 0x0000FF00) |\
                                     (((data) >> 24) & 0x000000FF))     // �����ߵ��ֽ�

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))      // �ϳ�Ϊһ����

#define COMPOUND_64BIT(data)        (((*(data-0) << 56) & 0xFF00000000000000) |\
                                     ((*(data-1) << 48) & 0x00FF000000000000) |\
                                     ((*(data-2) << 40) & 0x0000FF0000000000) |\
                                     ((*(data-3) << 32) & 0x000000FF00000000) |\
                                     ((*(data-4) << 24) & 0x00000000FF000000) |\
                                     ((*(data-5) << 16) & 0x0000000000FF0000) |\
                                     ((*(data-6) <<  8) & 0x000000000000FF00) |\
                                     ((*(data-7) <<  0) & 0x00000000000000FF))      // �ϳ�Ϊһ����
  

#define COMPOUND_16BIT(data)        (((*(data-0) << 8) & 0xFF00) |\
                                     ((*(data-1) << 0) & 0x00FF))
/**
 * @brief   �������ݴ���
 * @param   *data:  Ҫ��������ݵ�����.
 * @param   data_len: ���ݵĴ�С
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len);

/**
 * @brief   ��ʼ������Э��
 * @param   void
 * @return  ��ʼ�����.
 */
int32_t protocol_init(void);

/**
 * @brief   ���յ����ݴ���
 * @param   void
 * @return  -1��û���ҵ�һ����ȷ������.
 */
int8_t receiving_process(void);

/**
  * @brief ������λ����ֵ
  * @param cmd������
  * @param ch: ����ͨ��
  * @param data������ָ��
  * @param num����������
  * @retval ��
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
