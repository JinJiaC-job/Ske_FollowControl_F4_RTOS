#ifndef SENSORS_READING_H
#define SENSORS_READING_H


/*
���ǹ��ڴ�������ȡ��ͷ�ļ�

*/

#define ADC_CHANNELS 4
extern uint32_t ADC_value[ADC_CHANNELS];
extern float ADC_Pressure_Value[ADC_CHANNELS];


//��ѹ�����������ݶ�ȡ
void pressure_SensorReading(void);



#endif



