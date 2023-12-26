#ifndef SENSORS_READING_H
#define SENSORS_READING_H


/*
这是关于传感器读取的头文件

*/

#define ADC_CHANNELS 5
extern uint32_t ADC_value[ADC_CHANNELS];
extern float ADC_Pressure_Value[ADC_CHANNELS];
typedef struct ids830_PID;


//拉压力传感器数据读取
void pressure_SensorReading(void);
float ids830_compensation_PID_Cal(ids830_PID *pid, int32_t NowValue, int32_t AimValue);


#endif



