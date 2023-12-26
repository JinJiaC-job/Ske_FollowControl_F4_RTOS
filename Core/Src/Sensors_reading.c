#include "adc.h"
#include "stdio.h"
#include "Sensors_reading.h"
#include "math.h"

#define ids830_PID_INTEGRAL_ON    //位置式PID是否包含积分项。如果仅用PD控制，注释本行

typedef struct ids830_PID
{ 
    float P;               
    float I;
    float D;	
#ifdef 	ids830_PID_INTEGRAL_ON
    float Integral;        //位置式PID积分项
    float IntegralMax;     //位置式PID积分项最大值，用于限幅
#endif	
    float Last_Error;      //上一次误差	
    float OutputMax;       //位置式PID输出最大值，用于限幅
}ids830_PID;

// struct ids830_PID ids830_pid;
 

float ADC_Pressure_Value[ADC_CHANNELS];
uint32_t ADC_value[ADC_CHANNELS];

void pressure_SensorReading(void)
{
	for(int i=0; i<ADC_CHANNELS; i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,10);
		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
		{
			ADC_value[i] = HAL_ADC_GetValue(&hadc1);
		}
		if(i == 4)
		{
			ADC_Pressure_Value[i] = (float)ADC_value[i]*3.3f/4096 / 0.033f*9.8f;
		}
		else
		{
			ADC_Pressure_Value[i] = ((float)ADC_value[i]*3.3f/4096 - 1.65) / 0.033f*9.8f;
		}
		printf("L_Pres%d = %.3fN \t", i, ADC_Pressure_Value[i]);
	}
	printf("\r\n");
	HAL_ADC_Stop(&hadc1);
}

//void pressure_SensorReading(void)
//{
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1,10);
//		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
//		{
//			ADC_value[1] = HAL_ADC_GetValue(&hadc1);
//		}
//		ADC_Pressure_Value[1] = (float)ADC_value[1]*3.3f/4096;
//		printf("LinearActuator_Pressure%d = %.3fV \t", 1, ADC_Pressure_Value[1]);
//	  printf("\r\n");
//	HAL_ADC_Stop(&hadc1);
//}



float ids830_compensation_PID_Cal(ids830_PID *pid, int32_t NowValue, int32_t AimValue)
{
 
    float  iError, Output;     //当前误差, 控制输出	
 
    iError = AimValue - NowValue;                   //计算当前误差
	
#ifdef 	ids830_PID_INTEGRAL_ON
    pid->Integral += pid->I * iError;	            //位置式PID积分项累加
    pid->Integral = pid->Integral >  pid->IntegralMax ?  pid->IntegralMax : pid->Integral;  //积分项上限幅
    pid->Integral = pid->Integral < -pid->IntegralMax ? -pid->IntegralMax : pid->Integral; //积分项下限幅
#endif		
	
    Output = pid->P * iError                        //比例P            
           + pid->D * (iError - pid->Last_Error);   //微分D
	
#ifdef 	ids830_PID_INTEGRAL_ON	
    Output += pid->Integral;                        //积分I
#endif	
 
    Output = Output > pid->OutputMax ?  pid->OutputMax : Output;  //控制输出上限幅
    Output = Output <-pid->OutputMax ? -pid->OutputMax : Output; //控制输出下限幅
	
	pid->Last_Error = iError;		  	                     //更新上次误差，用于下次计算 
	return Output;	//返回控制输出值
}









