#ifndef SENSORS_READING_H
#define SENSORS_READING_H

#include "adc.h"
#include "stdio.h"
#include "tim.h"
#include "math.h"
#include "ids830can.h"

/*
���ǹ��ڴ�������ȡ��ͷ�ļ�

*/
#define ids830_PID_INTEGRAL_ON    //λ��ʽPID�Ƿ����������������PD���ƣ�ע�ͱ���

#define ADC_CHANNELS 5
extern uint32_t ADC_value[ADC_CHANNELS];
extern float ADC_Pressure_Value[ADC_CHANNELS];
extern float ADC_Pressure_InitValue[ADC_CHANNELS];
extern float ADC_Pressure_Value_filter[ADC_CHANNELS];

typedef struct PID
{ 
    float P;               
    float I;
    float D;	
#ifdef 	ids830_PID_INTEGRAL_ON
    float Integral;        //λ��ʽPID������
    float IntegralMax;     //λ��ʽPID���������ֵ�������޷�
#endif	
    float iError;          //��ǰ���
    float Last_Error;      //��һ�����	
    float pre_Error;       //���ϴ����
    float OutputMax;       //λ��ʽPID������ֵ�������޷�
    float OutputMin;       //λ��ʽPID�����Сֵ�������޷�

    float Output;          //pid���
    float Last_Output;     //��һ�����
    float Last_position_Output;
}PID;

extern PID *ids830_pid;

//��ѹ�����������ݶ�ȡ
void pressure_SensorReading(void);
void L_Pres_filter_1(int filter_N);
float LinearActuator_Pressure_Out(void);
void ids830_compensation_PID_Init(PID *ids830_pid);
float ids830_compensation_positionPID_Cal(PID *pid, float NowValue, float AimValue);
float ids830_compensation_incrementPID_Cal(PID *pid, float NowValue, float AimValue);

void LinearActuator_compensation(void);
void LinearActuator_compensation_LOW(void);


#endif



