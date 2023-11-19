#include "adc.h"
#include "stdio.h"
#include "Sensors_reading.h"

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
		ADC_Pressure_Value[i] = (float)ADC_value[i]*3.3f/4096;
		printf("LinearActuator_Pressure%d = %.3fV \t", i, ADC_Pressure_Value[i]);
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
