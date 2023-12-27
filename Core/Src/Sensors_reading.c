/*
connection：
PA5 <--> blue
PA6 <--> yellow1(small)
PB0 <--> white
PB1 <--> green
PC0 <--> yellow2(big)
*/
#include "Sensors_reading.h"

float ADC_Pressure_Value[ADC_CHANNELS];
float ADC_Pressure_InitValue[ADC_CHANNELS] = {0};
float ADC_Pressure_Value_filter[ADC_CHANNELS];
uint32_t ADC_value[ADC_CHANNELS];

PID pid;
PID *ids830_pid = &pid;

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
			ADC_Pressure_Value[i] = (float)ADC_value[i]*3.3f/4096 / 0.033f*9.8f - ADC_Pressure_InitValue[i];
		}
		else
		{
			ADC_Pressure_Value[i] = ((float)ADC_value[i]*3.3f/4096 - 1.65) / 0.033f*9.8f - ADC_Pressure_InitValue[i];
		}
		// printf("L_Pres%d = %.3fN \t", i, ADC_Pressure_Value[i]);
	}
	// printf("\r\n");
	// HAL_ADC_Stop(&hadc1);
}

void L_Pres_filter_1(int filter_N)
{
	int count,i,j;
	float Value_buf[ADC_CHANNELS][filter_N], temp;
	float sum[ADC_CHANNELS]={0};
	for(count=0;count<filter_N;count++)
	{
		pressure_SensorReading();
		for(i=0; i<ADC_CHANNELS; i++)
		{
			Value_buf[i][count] = ADC_Pressure_Value[i];
		}
	}
	for(i=0; i<ADC_CHANNELS; i++)
	{
		for(j=1; j<filter_N-1; j++)
		{
			if(Value_buf[i][0]>Value_buf[i][j])
			{
				temp = Value_buf[i][0];
				Value_buf[i][0]= Value_buf[i][j];
				Value_buf[i][j]=temp;
			}
			if(Value_buf[i][filter_N-1]<Value_buf[i][j])
			{
				temp = Value_buf[i][filter_N-1];
				Value_buf[i][filter_N-1]= Value_buf[i][j];
				Value_buf[i][j]=temp;
			}
		}
	}
	for(i=0; i<ADC_CHANNELS; i++)
	{
		for(count =1;count<filter_N-1;count++)
		{
			sum[i] += Value_buf[i][count];
		}	
		ADC_Pressure_Value_filter[i] = (float)(sum[i]/(filter_N-2));	
		// printf("L_Pres%d = %.3fN \t", i, ADC_Pressure_Value_filter[i]);
	}
	// printf("\r\n");

	// for(j=0;j<(filter_N-1);j++)
	// 	for(i=0;i<(filter_N-j);i++)
	// 		if(Value_buf[i]>Value_buf[i+1])
	// 		{
	// 			temp = Value_buf[i];
	// 			Value_buf[i]= Value_buf[i+1];
	// 			Value_buf[i+1]=temp;
	// 		}

	// return (int32_t)(sum/(filter_N-2));
}

// five pressure sensors -> linear actuator force output strategy:
float LinearActuator_Pressure_Out(void)
{
	float force_output, sum=0;
	for(int i=0; i<ADC_CHANNELS; i++)
	{
		sum += ADC_Pressure_Value_filter[i];
	}
	force_output = sum/ADC_CHANNELS;
	return force_output;
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

void ids830_compensation_PID_Init(PID *ids830_pid)
{
	ids830_pid->P = 20;
	ids830_pid->I = 0.1;
	ids830_pid->D = 10;
#ifdef 	ids830_PID_INTEGRAL_ON
	float Integral = 0;        //位置式PID积分项
	ids830_pid->IntegralMax = 100;
#endif
	float iError = 0;          //当前误差
	float Last_Error = 0;      //上一次误差
	float pre_Error = 0;       //上上次误差
	ids830_pid->OutputMax = 10;
	ids830_pid->OutputMin = -150;

	ids830_pid->Output = 0;          //pid输出
	ids830_pid->Last_Output = 0;
	ids830_pid->Last_position_Output = 0;
}

float ids830_compensation_positionPID_Cal(PID *pid, float NowValue, float AimValue)
{
    pid->iError = AimValue - NowValue;                   //计算当前误差		
	if(fabs(pid->iError)>1)
	{
#ifdef 	ids830_PID_INTEGRAL_ON
		pid->Integral += pid->I * pid->iError;	            //位置式PID积分项累加
		pid->Integral = pid->Integral >  pid->IntegralMax ?  pid->IntegralMax : pid->Integral;  //积分项上限幅
		pid->Integral = pid->Integral < -pid->IntegralMax ? -pid->IntegralMax : pid->Integral; //积分项下限幅
#endif		
	
		pid->Output = pid->P * pid->iError + pid->D * (pid->iError - pid->Last_Error);                      //比例P 微分D      
		// printf("pid->P:%d, pid->D:%d\r\n", pid->P, pid->D);      
		// printf("Output1:%.3f\r\n", Output);
#ifdef 	ids830_PID_INTEGRAL_ON	
		pid->Output += pid->Integral;                        //积分I
#endif	

		pid->Output = pid->Output > pid->OutputMax ?  pid->OutputMax : pid->Output;  //控制输出上限幅
		pid->Output = pid->Output < pid->OutputMin ?  pid->OutputMin : pid->Output; //控制输出下限幅
		
		pid->Last_Error = pid->iError;		  	                     //更新上次误差，用于下次计算 
		// printf("Output2:%.3f\r\n", Output);
	}
	else
	{
		pid->Output = pid->Last_Output;
	}
	pid->Last_Output = pid->Output;
	return pid->Output;	//返回控制输出值
}

//
//pidout+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
float ids830_compensation_incrementPID_Cal(PID *pid, float NowValue, float AimValue)
{
	pid->iError = AimValue - NowValue;                   //计算当前误差		

	pid->Output += pid->P * (pid->iError - pid->Last_Error) + pid->I * pid->iError + pid->D * (pid->iError - 2*pid->Last_Error + pid->pre_Error);
//	PID->out+=PID->kp*PID->ek-PID->ki*PID->ek1+PID->kd*PID->ek2;
	pid->pre_Error = pid->Last_Error;
	pid->Last_Error = pid->iError;  
		
	pid->Output = pid->Output > pid->OutputMax ?  pid->OutputMax : pid->Output;  //控制输出上限幅
	pid->Output = pid->Output < pid->OutputMin ?  pid->OutputMin : pid->Output; //控制输出下限幅
	
	return pid->Output;
}

void LinearActuator_compensation()
{
	float force_output = 0, position_output = 0;

	L_Pres_filter_1(10);
    force_output = LinearActuator_Pressure_Out();
    // position_output = ids830_compensation_positionPID_Cal(ids830_pid, force_output, 0);
	position_output = ids830_compensation_incrementPID_Cal(ids830_pid, force_output, 0);

	printf("force_output:%.3f\r\n", force_output);
	printf("position_output:%.3f\r\n", position_output);

	LinearActuator_startRun_maxspeed_position(1, position_output, position_output/0.05);
	Delay_ms(50);


}

void LinearActuator_compensation_LOW(void)
{
	float force_output = 0, position_output = 0, current_position = 0;

	L_Pres_filter_1(10);
    force_output = LinearActuator_Pressure_Out();
	LinearActuator_read_position(1);
	current_position = LinAcr_position_float;
	// if(current_position<(-10) || current_position>(200))
	// {

	// }
	if(force_output >= 2)
	{
		LinearActuator_startRun_maxspeed_position(1, -2+current_position, 5*fabs(-5)/0.05);
	}
	// else if(force_output >= 5 && force_output < 10)
	// {
	// 	LinearActuator_startRun_maxspeed_position(1, -1+current_position, 5*fabs(-5)/0.05);
	// }
	// else if(force_output >= 10 && force_output < 15)
	// {
	// 	LinearActuator_startRun_maxspeed_position(1, -1+current_position, 10*fabs(-5)/0.05);
	// }
	// else if(force_output >= 15 && force_output < 20)
	// {
	// 	LinearActuator_startRun_maxspeed_position(1, -1+current_position, 15*fabs(-5)/0.05);
	// }

	else if(force_output < -2)
	{
		LinearActuator_startRun_maxspeed_position(1, 1+current_position, 5*fabs(5)/0.05);
	}

	printf("force_output:%.3f\r\n", force_output);
	// printf("position_output:%.3f\r\n", position_output);

	// Delay_ms(50);

}

