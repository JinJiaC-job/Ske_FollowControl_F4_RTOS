#ifndef SENSORS_READING_H
#define SENSORS_READING_H

#include "adc.h"
#include "stdio.h"
#include "tim.h"
#include "math.h"
#include "ids830can.h"

/*
这是关于传感器读取的头文件

*/
#define ids830_PID_INTEGRAL_ON    //位置式PID是否包含积分项。如果仅用PD控制，注释本行

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
    float Integral;        //位置式PID积分项
    float IntegralMax;     //位置式PID积分项最大值，用于限幅
#endif	
    float iError;          //当前误差
    float Last_Error;      //上一次误差	
    float pre_Error;       //上上次误差
    float OutputMax;       //位置式PID输出最大值，用于限幅
    float OutputMin;       //位置式PID输出最小值，用于限幅

    float Output;          //pid输出
    float Last_Output;     //上一次输出
    float Last_position_Output;
}PID;

extern PID *ids830_pid;

//拉压力传感器数据读取
void pressure_SensorReading(void);
void L_Pres_filter_1(int filter_N);
float LinearActuator_Pressure_Out(void);
void ids830_compensation_PID_Init(PID *ids830_pid);
float ids830_compensation_positionPID_Cal(PID *pid, float NowValue, float AimValue);
float ids830_compensation_incrementPID_Cal(PID *pid, float NowValue, float AimValue);

void LinearActuator_compensation(void);
void LinearActuator_compensation_LOW(void);


#endif



