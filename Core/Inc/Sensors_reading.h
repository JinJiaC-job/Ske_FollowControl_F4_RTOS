#ifndef SENSORS_READING_H
#define SENSORS_READING_H


/*
这是关于传感器读取的头文件

*/

#define ADC_CHANNELS 4
extern uint32_t ADC_value[ADC_CHANNELS];
extern float ADC_Pressure_Value[ADC_CHANNELS];


//拉压力传感器数据读取
void pressure_SensorReading(void);



#endif



