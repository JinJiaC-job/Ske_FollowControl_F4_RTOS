// LK 电机驱动文件

#include "lkmoto.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "stdio.h"
#include "ids830can.h"
#include "tim.h"
#include "Sensors_reading.h"
#include "usart.h"

//void ms_Delay(uint16_t t_ms)
//{
//	uint32_t t=t_ms*8000;//要考虑主频3127
//	while(t--);
//}

uint8_t CAN_motor_data[8] = {0};//电机接收数据
uint8_t CAN_motor_angle[8] = {0};//电机接收角度数据
uint8_t CAN_motor_currentAndspeed[8] = {0};//电机接收电流和速度数据
uint8_t CAN_motor_PosCurntSpd_ans[8] = {0};//电机控制命令23驱动回复
uint8_t CAN_motor_accel[8] = {0};//电机控制命令4(读取加速度信息)驱动回复
uint32_t circleAngle;//电机角度�??
int32_t motorAccel;//电机加速度�??

float motor_current_float[7] = {0};
float motor_speed_float[7] = {0};
float motorAngle_float[7] = {0};
float motorAccel_float[7] = {0};
int16_t motor_current=0;
int16_t motor_speed=0;

//can 总线 send 函数 移植仅需修改此函�??
// id 0x140 + (1~32)

void can_send(uint8_t *buf,uint8_t id)
{
    uint8_t motorId = id;
    hCAN1_TxHeader.StdId = 0x140 + motorId;
    hCAN1_TxHeader.ExtId = 0x00;
    hCAN1_TxHeader.RTR = CAN_RTR_DATA;
    hCAN1_TxHeader.IDE = CAN_ID_STD;
    hCAN1_TxHeader.DLC = 8;
    static uint32_t txMailBox;
	HAL_CAN_AddTxMessage(&hcan1, &hCAN1_TxHeader, buf, &txMailBox);
//    while(HAL_CAN_AddTxMessage(&hcan, &hCAN1_TxHeader, buf, &txMailBox) != HAL_OK)
//		{
//			printf("TxMsg Failed!!");
//      HAL_Delay(100);
//		
//		}
//		printf("\nSend Tx Message Success!!Tx_Mail:%d", txMailBox);
//    uint8_t i;
//    for(i=0; i<LEN; i++)
//    {
//        //printf("%02x ",id, buf[i]);
//    }
    //printf("\r\n");
}

//消息处理函数 名称 命令数据
// 读取 PID 参数命令 0x30
// 写入 PID 参数�?? RAM 命令 0x31
// 写入 PID 参数�?? ROM 命令 0x32
// 读取加速度命令 0x33
// 写入加速度�?? RAM 命令 0x34
// 读取编码器命�?? 0x90
// 写入编码器值到 ROM 作为电机零点命令 0x91
// 写入当前位置�?? ROM 作为电机零点命令 0x19
// 读取多圈角度命令 0x92
// 读取单圈角度命令 0x94
// 清除电机角度命令（设置电机初始位置） 0x95
// 读取电机状�? 1 和错误标志命�?? 0x9A
// 清除电机错误标志命令 0x9B
// 读取电机状�? 2 命令 0x9C
// 读取电机状�? 3 命令 0x9D
// 电机关闭命令 0x80
// 电机停止命令 0x81
// 电机运行命令 0x88
// 转矩开环控制命�?? 0xA0
// 转矩闭环控制命令 0xA1
// 速度闭环控制命令 0xA2
// 位置闭环控制命令 1 0xA3
// 位置闭环控制命令 2 0xA4
// 位置闭环控制命令 3 0xA5
// 位置闭环控制命令 4 0xA6
// 位置闭环控制命令 5 0xA7
// 位置闭环控制命令 6 0xA8
void can_msg_process(uint8_t id,uint8_t *buf)
{  
    switch (buf[0])
    {
    case 0x30:
        /* code */
        break;
    
    default:
        break;
    }
}





//只需要移植上述两个函数即�??
//(1)
// 发�?
//  读取 PID 参数命令 0x30
// 主机发送该命令读取当前电机的的 PID 参数
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x30
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

//回复 
// 驱动回复数据中包含了各个控制环路�?? PI 参数�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x30
// DATA[1] NULL 0x00
// DATA[2] 位置�?? P 参数 DATA[2] = anglePidKp
// DATA[3] 位置�?? I 参数 DATA[3] = anglePidKi
// DATA[4] 速度�?? P 参数 DATA[4] = speedPidKp
// DATA[5] 速度�?? I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩�?? P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩�?? I 参数 DATA[7] = iqPidKi

void read_pid(uint8_t id){
    uint8_t buf[LEN] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}

//(2)
//  写入 PID 参数�?? RAM 命令 0x31
// DATA[0] 命令字节 0x31
// DATA[1] NULL 0x00
// DATA[2] 位置�?? P 参数 DATA[2] = anglePidKp
// DATA[3] 位置�?? I 参数 DATA[3] = anglePidKi
// DATA[4] 速度�?? P 参数 DATA[4] = speedPidKp
// DATA[5] 速度�?? I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩�?? P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩�?? I 参数 DATA[7] = iqPidKi

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??

void write_pid(uint8_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi){
    uint8_t buf[LEN] = {0x31, 0x00, anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi};
    can_send(buf, id);
}

//(3)
//  写入 PID 参数�?? ROM 命令 0x32
// DATA[0] 命令字节 0x32
// DATA[1] NULL 0x00
// DATA[2] 位置�?? P 参数 DATA[2] = anglePidKp
// DATA[3] 位置�?? I 参数 DATA[3] = anglePidKi
// DATA[4] 速度�?? P 参数 DATA[4] = speedPidKp
// DATA[5] 速度�?? I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩�?? P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩�?? I 参数 DATA[7] = iqPidKi

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??
void write_pid_to_rom(uint8_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi){
    uint8_t buf[LEN] = {0x32, 0x00, anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi};
    can_send(buf, id);
}




//(4)
//  读取加速度命令 0x33
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x33
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复数据中包含了加速度参数。加速度数据 Accel �?? int32_t 类型，单�?? 1dps/s
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x33
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 加速度低字�?? 1 DATA[4] = *(uint8_t *)(&Accel)
// DATA[5] 加速度字节 2 DATA[5] = *((uint8_t *)(&Accel)+1)
// DATA[6] 加速度字节 3 DATA[6] = *((uint8_t *)(&Accel)+2)
// DATA[7] 加速度字节 4 DATA[7] = *((uint8_t *)(&Accel)+3)

void read_acc(uint8_t id){
	uint8_t buf[LEN] = {0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	can_send(buf, id);
	Delay_us(600);
	
	for(int i=0; i<4; i++)
	{
		*((uint8_t *)(&motorAccel)+i) = CAN_motor_accel[i+4];
	}
	switch(id)
	{
		case 6: motorAccel_float[id] = (float)((int32_t)motorAccel/10.0f); break;
		case 5: motorAccel_float[id] = (float)((int32_t)motorAccel/8.0f); break;
		case 4: motorAccel_float[id] = (float)((int32_t)motorAccel/36.0f); break;
		case 3: motorAccel_float[id] = (float)((int32_t)motorAccel/8.0f); break;
		case 2: motorAccel_float[id] = (float)((int32_t)motorAccel/36.0f); break;
		default:printf("id error\n"); break;
	}
}



//(5)
//  写入加速度�?? RAM 命令 0x34
// DATA[0] 命令字节 0x34
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 加速度低字�?? 1 DATA[4] = *(uint8_t *)(&Accel)
// DATA[5] 加速度字节 2 DATA[5] = *((uint8_t *)(&Accel)+1)
// DATA[6] 加速度字节 3 DATA[6] = *((uint8_t *)(&Accel)+2)
// DATA[7] 加速度字节 4 DATA[7] = *((uint8_t *)(&Accel)+3)

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??

void write_acc(uint8_t id, float Accel){
    uint8_t buf[LEN] = {0x34, 0x00, 0x00, 0x00, *(uint8_t *)(&Accel), *((uint8_t *)(&Accel)+1), *((uint8_t *)(&Accel)+2), *((uint8_t *)(&Accel)+3)};
    can_send(buf, id);
}



//(6)
//  读取编码器命�?? 0x90
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x90
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 编码器位�?? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383），为编码器原始位置�??
// 去编码器零偏后的值�?
// 2. 编码器原始位�?? encoderRaw（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 3. 编码器零�?? encoderOffset（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383），该点作为电机�??
// 度的 0 点�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x90
// DATA[1] NULL 0x00
// DATA[2] 编码器位置低字节 DATA[2] = *(uint8_t *)(&encoder)
// DATA[3] 编码器位置高字节 DATA[3] = *((uint8_t *)(&encoder)+1)
// DATA[4] 编码器原始位置低字节 DATA[4] = *(uint8_t *)(&encoderRaw)
// DATA[5] 编码器原始位置高字节 DATA[5] = *((uint8_t *)(&encoderRaw)+1)
// DATA[6] 编码器零偏低字节 DATA[6] = *(uint8_t *)(&encoderOffset)
// DATA[7] 编码器零偏高字节 DATA[7] = *((uint8_t *)(&encoderOffset)+1)

void read_encoder(uint8_t id){
    uint8_t buf[LEN] = {0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
		Delay_ms(command_interval_time);
}





//(7)
//  写入编码器值到 ROM 作为电机零点命令 0x91
// 主机发送该命令以设置编码器的零偏，其中，需要写入的编码器�? encoderOffset �?? uint16_t 类型�??
// 14bit 编码器的数值范�?? 0~16383�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x91
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] 编码器零偏低字节 DATA[6] = *(uint8_t *)(&encoderOffset)
// DATA[7] 编码器零偏高字节 DATA[7] = *((uint8_t *)(&encoderOffset)+1)

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??

void write_encoder_offset(uint8_t id, uint16_t encoderOffset){
    uint8_t buf[LEN] = {0x91, 0x00, 0x00, 0x00, 0x00, 0x00, *(uint8_t *)(&encoderOffset), *((uint8_t *)(&encoderOffset)+1)};
    can_send(buf, id);
		Delay_ms(command_interval_time);
}


//(8)
//  写入当前位置�?? ROM 作为电机零点命令 0x19
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x19
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??

void write_current_position_to_rom(uint8_t id){
    uint8_t buf[LEN] = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}


//(9)
//  读取多圈角度命令 0x92
// 主机发送该命令以读取当前电机的多圈绝对角度�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x92
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机角度 motorAngle，为 int64_t 类型数据，正值表示顺时针累计角度，负值表示逆时针累计角
// 度，单位 0.01°/LSB�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x92
// DATA[1] 角度低字�?? 1 DATA[1] = *(uint8_t *)(&motorAngle)
// DATA[2] 角度字节 2 DATA[2] = *((uint8_t *)(& motorAngle)+1)
// DATA[3] 角度字节 3 DATA[3] = *((uint8_t *)(& motorAngle)+2)
// DATA[4] 角度字节 4 DATA[4] = *((uint8_t *)(& motorAngle)+3)
// DATA[5] 角度字节 5 DATA[5] = *((uint8_t *)(& motorAngle)+4)
// DATA[6] 角度字节 6 DATA[6] = *((uint8_t *)(& motorAngle)+5)
// DATA[7] 角度字节 7 DATA[7] = *((uint8_t *)(& motorAngle)+6)



void read_angle(uint8_t id){
    uint8_t buf[LEN] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int64_t motorAngle;

    can_send(buf, id);
	Delay_us(600);
	for(int i=0; i<7; i++)
	{
		*((uint8_t *)(&motorAngle)+i) = CAN_motor_angle[i+1];
	}
	switch(id)
	{
		case 6: motorAngle_float[id] = (float)((int32_t)motorAngle/1000.0); break;
		case 5: motorAngle_float[id] = (float)((int32_t)motorAngle/800.0); break;
		case 4: motorAngle_float[id] = -(float)((int32_t)motorAngle/3600.0); break;//joint4 DH angle = - meassured angle;
		case 3: motorAngle_float[id] = (float)((int32_t)motorAngle/800.0); break;
		case 2: motorAngle_float[id] = (float)((int32_t)motorAngle/3600.0); break;
		default:printf("id error\n"); break;
	}	
//		printf("motor_angle%d: %.3fdu\r\n", id, motorAngle_float);
//    Delay_ms(command_interval_time);
}


//(10)
//  读取单圈角度命令 0x94
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x94
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机单圈角度 circleAngle，为 uint16_t 类型数据，以编码器零点为起始点，顺时针增加，再次�??
// 达零点时数值回 0，单�?? 0.01°/LSB，数值范�?? 0~35999�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x94
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 单圈角度低字�?? DATA[4] = *(uint8_t *)(& circleAngle)
// DATA[5] 单圈角度高字�?? DATA[5] = *((uint8_t *)(& circleAngle)+1)
// DATA[6] 单圈角度高字�?? DATA[6] = *((uint8_t *)(& circleAngle)+2)
// DATA[7] 单圈角度高字�?? DATA[7] = *((uint8_t *)(& circleAngle)+3)

void read_angle_single(uint8_t id){
    uint8_t buf[LEN] = {0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
    Delay_ms(command_interval_time);
	for(int i=0;i<4;i++)
	{
		*((uint8_t *)(& circleAngle)+i) = CAN_motor_data[4+i];
	}
	
}


//(11)
//  清除电机角度命令（设置电机初始位置） 0x95 厂家暂未实现
// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??


//(12)
//  读取电机状�? 1 和错误标志命�?? 0x9A
// 该命令读取当前电机的温度、电压和错误状态标�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下参数�??
// 1. 电机温度 temperature（int8_t 类型，单�?? 1�??/LSB）�?
// 2. 电压 voltage（uint16_t 类型，单�?? 0.1V/LSB）�?
// 3. 错误标志 errorState（为 uint8_t 类型，各个位代表不同的电机状态）
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] NULL 0x00
// DATA[3] 电压低字�?? DATA[3] = *(uint8_t *)(&voltage)
// DATA[4] 电压高字�?? DATA[4] = *((uint8_t *)(& voltage)+1)
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] 错误状态字�?? DATA[7]=errorState
// 备注�??
// 1. errorState 各个位具体状态表如下
// errorState �?? 状态说�?? 0 1
// 0 电压状�? 电压正常 低压保护
// 1 无效
// 2 无效
// 3 温度状�? 温度正常 过温保护
// 4 无效
// 5 无效
// 6 无效
// 7 无效
void read_status(uint8_t id){
    uint8_t buf[LEN] = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}


//(13)
//  清除电机错误标志命令 0x9B

// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9B
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

//驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下参数�??
// 1. 电机温度 temperature（int8_t 类型，单�?? 1�??/LSB）�?
// 2. 电压 voltage（uint16_t 类型，单�?? 0.1V/LSB）�?
// 3. 错误标志 errorState（为 uint8_t 类型，各个位代表不同的电机状态）�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] NULL 0x00
// DATA[3] 电压低字�?? DATA[3] = *(uint8_t *)(&voltage)
// DATA[4] 电压高字�?? DATA[4] = *((uint8_t *)(& voltage)+1)
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] 错误状态字�?? DATA[7]=errorState
// 备注�??
// 1. 电机状态没有恢复正常时，错误标志无法清除�?
// 2. errorState 各个位具体状态参考读取电机状�?? 1 和错误标志命令�?

void clear_error(uint8_t id){
    uint8_t buf[LEN] = {0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}


//(14)
//  读取电机状�? 2 命令 0x9C
// 该命令读取当前电机的温度、电压、转速、编码器位置�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9C
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9C
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)



void read_status2(uint8_t id){
//	uint16_t motor_current, motor_speed;
	
	uint8_t buf[LEN] = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	can_send(buf, id);
	Delay_ms(1);
	
	*(uint8_t *)(&motor_current) = CAN_motor_currentAndspeed[2];
	*((uint8_t *)(&motor_current)+1) = CAN_motor_currentAndspeed[3];
	if(id == 4)
	{
		motor_current_float[id] = -(float)((int16_t)motor_current/2048.0*33.0);//joint4 DH current = - meassured current;
	}
	else
		motor_current_float[id] = (float)((int16_t)motor_current/2048.0*33.0);
//	printf("motor_current%d: %.3fA\r\n", id, motor_current_float);
	
	*(uint8_t *)(&motor_speed) = CAN_motor_currentAndspeed[4];
	*((uint8_t *)(&motor_speed)+1) = CAN_motor_currentAndspeed[5];
//	motor_speed_float = (int16_t)motor_speed;
	switch(id)
	{
		case 6: motor_speed_float[id] = (float)((int16_t)motor_speed/10.0); break;
		case 5: motor_speed_float[id] = (float)((int16_t)motor_speed/8.0); break;
		case 4: motor_speed_float[id] = -(float)((int16_t)motor_speed/36.0); break;//joint4 DH speed = - meassured speed;
		case 3: motor_speed_float[id] = (float)((int16_t)motor_speed/8.0); break;
		case 2: motor_speed_float[id] = (float)((int16_t)motor_speed/36.0); break;
		default:printf("id error\n"); break;
	}
//	printf("motor_speed%d: %ddps\r\n", id, (int16_t)motor_speed);
	
//	Delay_ms(command_interval_time);
}


//(15)
//  读取电机状�? 3 命令 0x9D
// 该命令读取当前电机的温度和相电流数据
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9D
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下数据�??
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB�??
// 2. A 相电流数据，数据类型�?? int16_t 类型，对应实际相电流�?? 1A/64LSB�??
// 3. B 相电流数据，数据类型�?? int16_t 类型，对应实际相电流�?? 1A/64LSB�??
// 4. C 相电流数据，数据类型�?? int16_t 类型，对应实际相电流�?? 1A/64LSB�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x9D
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] A 相电流低字节 DATA[2] = *(uint8_t *)(&iA)
// DATA[3] A 相电流高字节 DATA[3] = *((uint8_t *)(& iA)+1)
// DATA[4] B 相电流低字节 DATA[4] = *(uint8_t *)(&iB)
// DATA[5] B 相电流高字节 DATA[5] = *((uint8_t *)(& iB)+1)
// DATA[6] C 相电流低字节 DATA[6] = *(uint8_t *)(&iC)
// DATA[7] C 相电流高字节 DATA[7] = *((uint8_t *)(& iC)+1)


void read_status3(uint8_t id){
    uint8_t buf[LEN] = {0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}





//(16)
//  电机关闭命令 0x80

// 关闭电机，同时清除电机运行状态和之前接收的控制指�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x80
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??

void motor_close(uint8_t id){
    uint8_t buf[LEN] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}

//(17)
//  电机停止命令 0x81
// 停止电机，但不清除电机运行状态和之前接收的控制指�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x81
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??

void motor_stop(uint8_t id){
    uint8_t buf[LEN] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
}


//(18)
//  电机运行命令 0x88
// 从电机停止命令中恢复电机运行（恢复停止前的控制方式）
// 数据�?? 说明 数据
// DATA[0] 命令字节 0x88
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一�??

void motor_run(uint8_t id){
    uint8_t buf[LEN] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(buf, id);
    Delay_ms(command_interval_time);
}


//(19)
//  转矩开环控制命�?? 0xA0
// 主机发送该命令以控制电机的开环输出功率，控制�?? powerControl �?? int16_t 类型，数值范�??-1000~
// 1000，（电机母线电流和扭矩因不同电机而异）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA0
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 输出功率控制值低字节 DATA[4] = *(uint8_t *)(& powerControl)
// DATA[5] 输出功率控制值高字节 DATA[5] = *((uint8_t *)(& powerControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 1. 该命令中的控制�? powerControl 不受上位�??(LK-Motor Tool)中的 Max Power 值限制�?

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的输出功率值（int16_t 类型，范�??-1000~1000）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA0
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 输出功率值低字节 DATA[2] = *(uint8_t *)(& power)
// DATA[3] 输出功率值高字节 DATA[3] = *((uint8_t *)(&power)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void torque_open_loop(uint8_t id, int16_t powerControl){
    uint8_t buf[LEN] = {0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& powerControl);
    buf[5] = *((uint8_t *)(& powerControl)+1);
    can_send(buf, id);
}


//(20)
//  转矩闭环控制命令 0xA1
// 主机发送该命令以控制电机的转矩电流输出，控制�? iqControl �?? int16_t 类型，数值范�??-2000~ 2000�??
// 对应实际转矩电流范围-32A~32A（母线电流和电机的实际扭矩因不同电机而异）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA1
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 转矩电流控制值低字节 DATA[4] = *(uint8_t *)(&iqControl)
// DATA[5] 转矩电流控制值高字节 DATA[5] = *((uint8_t *)(&iqControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA1
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

// 关节1�??
// 关节2�??4：额定扭矩（35 N.m�??; 额定电流�??6.9 A�??
// 关节3�??5：额定扭矩（6  N.m�??; 额定电流�??3.5 A�??
// 关节6：额定扭矩（1  N.m�??; 额定电流�??1.6 A�??

void torque_close_loop(uint8_t id, float torqueControl){
    uint8_t buf[LEN] = {0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		int16_t iqControl = 0;
		
		switch(id)
		{
			case 6: iqControl = (int16_t)(torqueControl/1*1.6/32*2000); break;
			case 5: iqControl = (int16_t)(torqueControl/6*3.5/32*2000); break;
			case 4: iqControl = (int16_t)(torqueControl/35*6.9/32*2000); break;
			case 3: iqControl = (int16_t)(torqueControl/6*3.5/32*2000); break;
			case 2: iqControl = (int16_t)(torqueControl/35*6.9/32*2000); break;
			default:printf("id error\n"); break;
    }
    buf[4] = *(uint8_t *)(& iqControl);
    buf[5] = *((uint8_t *)(& iqControl)+1);
    can_send(buf, id);

    Delay_ms(command_interval_time);
}


//(21)
//  速度闭环控制命令 0xA2
// 主机发送该命令以控制电机的速度，控制�? speedControl �?? int32_t 类型，对应实际转速为 0.01dps/LSB�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA2
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 速度控制低字�?? DATA[4] = *(uint8_t *)(&speedControl)
// DATA[5] 速度控制 DATA[5] = *((uint8_t *)(&speedControl)+1)
// DATA[6] 速度控制 DATA[6] = *((uint8_t *)(&speedControl)+2)
// DATA[7] 速度控制高字�?? DATA[7] = *((uint8_t *)(&speedControl)+3)

// 1. 该命令下电机的最大转矩电流由上位�??(LK-Motor Tool)中的 Max Torque Current 值限制�?
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制�?

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA2
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void speed_close_loop(uint8_t id, int32_t speedControl){
    uint8_t buf[LEN] = {0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& speedControl);
    buf[5] = *((uint8_t *)(& speedControl)+1);
    buf[6] = *((uint8_t *)(& speedControl)+2);
    buf[7] = *((uint8_t *)(& speedControl)+3);
    can_send(buf, id);
}



//(22)
//  位置闭环控制命令 1 0xA3
// 主机发送该命令以控制电机的位置（多圈角度）�?? 控制�?? angleControl �?? int32_t 类型，对应实际位
// 置为 0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA3
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字�?? DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字�?? DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注�??
// 1. 该命令下的控制�? angleControl 受上位机(LK-Motor Tool)中的 Max Angle 值限制�?
// 2. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制�?
// 3. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制�?
// 4. 该控制模式下，电机的最大转矩电流由上位�??(LK-Motor Tool)中的 Max Torque Current 值限
// �??

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA3
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop(uint8_t id, int32_t angleControl){
    uint8_t buf[LEN] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    buf[6] = *((uint8_t *)(& angleControl)+2);
    buf[7] = *((uint8_t *)(& angleControl)+3);
    can_send(buf, id);
    Delay_ms(command_interval_time);
}


//(23)
//  位置闭环控制命令 2 0xA4
// 主机发送该命令以控制电机的位置（多圈角度）�?? 控制�?? angleControl �?? int32_t 类型，对应实际位
// 置为 0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定�?
// 控制�?? maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转�?? 1dps/LSB�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA4
// DATA[1] NULL 0x00
// DATA[2] 速度限制低字�?? DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字�?? DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字�?? DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字�?? DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注�??
// 1. 该命令下的控制�? angleControl 受上位机(LK-Motor Tool)中的 Max Angle 值限制�?
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制�?
// 3. 该控制模式下，电机的最大转矩电流由上位�??(LK-Motor Tool)中的 Max Torque Current 值限
// 制�?

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA4
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)


void angle_close_loop_with_speed(uint8_t id, float angleControl, float maxSpeed){
    uint8_t buf[LEN] = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		int32_t angleControl_int32;
		int16_t maxSpeed_int16;
		switch(id)
		{
			case 6: angleControl_int32 = (int32_t)(angleControl*1000.0f); maxSpeed_int16 = (uint16_t)maxSpeed*10.0; break;
			case 5: angleControl_int32 = (int32_t)(angleControl*800.0f); maxSpeed_int16 = (uint16_t)maxSpeed*8.0; break;
			case 4: angleControl_int32 = (int32_t)(angleControl*3600.0f); maxSpeed_int16 = (uint16_t)maxSpeed*36.0; break;
			case 3: angleControl_int32 = (int32_t)(angleControl*800.0f); maxSpeed_int16 = (uint16_t)maxSpeed*8.0; break;
			case 2: angleControl_int32 = (int32_t)(angleControl*3600.0f); maxSpeed_int16 = (uint16_t)maxSpeed*36.0; break;
			default:printf("id error\n"); break;
    }
    buf[2] = *(uint8_t *)(&maxSpeed_int16);
    buf[3] = *((uint8_t *)(&maxSpeed_int16)+1);
    buf[4] = *(uint8_t *)(&angleControl_int32);
    buf[5] = *((uint8_t *)(&angleControl_int32)+1);
    buf[6] = *((uint8_t *)(&angleControl_int32)+2);
    buf[7] = *((uint8_t *)(&angleControl_int32)+3);
    can_send(buf, id);
		Delay_us(600);
//    Delay_ms(command_interval_time);
}

//(24)
//  位置闭环控制命令 3 0xA5
// 主机发送该命令以控制电机的位置（单圈角度）�?? 控制�?? angleControl �?? uint16_t 类型，数值范�??
// 0~35999，对应实际位置为 0.01degree/LSB，即实际角度范围 0°~359.99°�??
// 控制�?? spinDirection 设置电机转动的方向，�?? uint8_t 类型�??0x00 代表顺时针，0x01 代表逆时针�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA5
// DATA[1] 转动方向字节 DATA[1] = spinDirection
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字�?? DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制高字�?? DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 备注�??
// 1. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制�?
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制�?
// 3. 该控制模式下，电机的最大转矩电流由上位�??(LK-Motor Tool)中的 Max Torque Current 值限
// 制�?

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA5
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction(uint8_t id, int32_t angleControl, uint8_t spinDirection){
    uint8_t buf[LEN] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[1] = spinDirection;
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    can_send(buf, id);
}


//(25)
//  位置闭环控制命令 4 0xA6
// 主机发送该命令以控制电机的位置（单圈角度）�??
// 1. 角度控制�?? angleControl �?? uint16_t 类型，数值范�?? 0~35999，对应实际位置为 0.01degree/LSB�??
// 即实际角度范�?? 0°~359.99°�??
// 2. spinDirection 设置电机转动的方向，�?? uint8_t 类型�??0x00 代表顺时针，0x01 代表逆时针�?
// 3. maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转�?? 1dps/LSB�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA6
// DATA[1] 转动方向字节 DATA[1] = spinDirection
// DATA[2] 速度限制低字�?? DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字�?? DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字�?? DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制高字�?? DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 备注�??
// 1. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制�?
// 2. 该控制模式下，电机的最大转矩电流由上位�??(LK-Motor Tool)中的 Max Torque Current 值限
// 制�?

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA6
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction_and_speed(uint8_t id, int32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed){
    uint8_t buf[LEN] = {0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[1] = spinDirection;
    buf[2] = *(uint8_t *)(& maxSpeed);
    buf[3] = *((uint8_t *)(& maxSpeed)+1);
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    can_send(buf, id);
		Delay_ms(1);
}


//(26)
//  位置闭环控制命令 5 0xA7
// 主机发送该命令以控制电机的位置增量�?? 控制�?? angleControl �?? int32_t 类型，对应实际位置为
// 0.01degree/LSB。电机的转动方向由控制量的符号确定�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA7
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字�?? DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字�?? DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注�??
// 1. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制�?
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制�?
// 3. 该控制模式下，电机的最大转矩电流由上位�??(LK-Motor Tool)中的 Max Torque Current 值限
// 制�?

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA7
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop4(uint8_t id, int32_t angleControl){
    uint8_t buf[LEN] = {0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    buf[6] = *((uint8_t *)(& angleControl)+2);
    buf[7] = *((uint8_t *)(& angleControl)+3);
    can_send(buf, id);
}




//(27)
//  位置闭环控制命令 6 0xA8
// 主机发送该命令以控制电机的位置增量�?? 控制�?? angleControl �?? int32_t 类型，对应实际位置为
// 0.01degree/LSB。电机的转动方向由控制量的符号确定�?
// 控制�?? maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转�?? 1dps/LSB�??
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA8
// DATA[1] NULL 0x00
// DATA[2] 速度限制低字�?? DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字�?? DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字�?? DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字�?? DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注�??
// 1. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制�?
// 2. 该控制模式下，电机的最大转矩电流由上位�??(LK-Motor Tool)中的 Max Torque Current 值限
// 制�?

// 驱动回复�??1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数�?
// 1. 电机温度 temperature（int8_t 类型�??1�??/LSB）�?
// 2. 电机的转矩电流�? iq（int16_t 类型，范�??-2048~2048，对应实际转矩电流范�??-33A~33A）�?
// 3. 电机转�? speed（int16_t 类型�??1dps/LSB）�?
// 4. 编码器位置�? encoder（uint16_t 类型�??14bit 编码器的数值范�?? 0~16383）�?
// 数据�?? 说明 数据
// DATA[0] 命令字节 0xA8
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字�?? DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字�?? DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字�?? DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字�?? DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction_and_angle_and_max_speed(uint8_t id, int32_t angleControl, uint16_t maxSpeed){
    uint8_t buf[LEN] = {0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[2] = *(uint8_t *)(& maxSpeed);
    buf[3] = *((uint8_t *)(& maxSpeed)+1);
    buf[4] = *(uint8_t *)(& angleControl);
    buf[5] = *((uint8_t *)(& angleControl)+1);
    buf[6] = *((uint8_t *)(& angleControl)+2);
    buf[7] = *((uint8_t *)(& angleControl)+3);
    can_send(buf, id);
	Delay_ms(10);
}

//motor1:角度需要乘上减速比�??360°=36000*10=360000
//motor2:角度需要乘上减速比�??360°=36000*8=288000
//motor3:角度需要乘上减速比�??360°=36000*36=1296000
//motor4:角度需要乘上减速比�??360°=36000*8=288000
//motor5:角度需要乘上减速比�??360°=36000*36=1296000

//机械臂零点位�??
void ske_base_position(void)
{
	
////写入当前位置到ROM作为零点(多次写入会影响芯片寿命，不建议频繁使�??)
// 	HAL_Delay(1000);
// 	for(int i=1;i<=6;i++)
// 	{
// 		write_current_position_to_rom(i);
// 		HAL_Delay(1000);
// 	}
	
////读取编码器位置，并将编码器零偏值写入ROM作为电机零点
//	for(int i=2;i<=6;i++)
//	{
//		read_encoder(i);
//		uint16_t encoderOffset = 0;
//		*(uint8_t *)(&encoderOffset) = CAN_motor_data[6];
//		*((uint8_t *)(&encoderOffset)+1) = CAN_motor_data[7];
//		write_encoder_offset(i, encoderOffset);
//	}
//	printf("\nset motors zero point Success!!\r\n");
	
	//所有电机及电缸回到初始位置
	for(int i=1; i<=6; i++)
	{
		if(i == 1)
			LinearActuator_startRun_maxspeed_position(i, 0, 30);
		
		else if(i == 4)
			angle_close_loop_with_speed(i, 0, 30);
		else if(i == 5)
			angle_close_loop_with_speed(i, 0, 30);
		
		else
			angle_close_loop_with_speed(i, 0, 30);
	}
	Delay_ms(5000);
}

//读取五个关节扭矩电流
void read_5torque_current(void)
{
	for(int i=0;i<6;i++)
	{
		read_status2(i);
		Delay_ms(1000);
	}
	printf("\nGet Agroup of turque current Message Success!!");
}

//读取六个关节角度、角速度、扭矩电流等信息，在控制中断外读取各关节角度
void read_all_joint_message(void)
{
	for(int r_id=1; r_id<=6; r_id++)
	{
		if(r_id == 1)
		{
			LinearActuator_read_position(r_id);
			LinearActuator_read_CurrentandSpeed(r_id);
			pressure_SensorReading();
		}
		else
		{
			read_status2(r_id);
			read_angle(r_id);
		}
	}
//	uint8_t start_flag[5] = {'s', 't', 'a', 'r', 't'};
//	HAL_UART_Transmit(&huart1, start_flag, 5, 5);

//	for(int j=0; j<19; j++)
//	{
//		uint8_t LinAcr_position_int8[4];
//		LinAcr_position_int8[0]=(uint8_t)((uint8_t)LinAcr_position_float>>24);
//		LinAcr_position_int8[1]=(uint8_t)((uint8_t)LinAcr_position_float>>18&0xFF);
//		LinAcr_position_int8[2]=(uint8_t)((uint8_t)LinAcr_position_float>>12&0xFF);
//		LinAcr_position_int8[3]=(uint8_t)((uint8_t)LinAcr_position_float&0xFF);
//		HAL_UART_Transmit(&huart1,LinAcr_position_int8, 4, 5);
//	}
//	
//	uint8_t stop_flag[4] = {'s', 't', 'o', 'p'};
//	HAL_UART_Transmit(&huart1, stop_flag, 4, 5);
	
	printf("LA_pos:%.3fmm, LA_c:%.3fA, LA_s:%.3frpm, LA_Prs:%.3fV\t\r\n  \
	        m_c2:%.3fA , m_s2:%.3fdps, m_a2:%.3fdu\t\r\n               \
	        m_c3:%.3fA , m_s3:%.3fdps, m_a3:%.3fdu\t\r\n               \
	        m_c4:%.3fA , m_s4:%.3fdps, m_a4:%.3fdu\t\r\n               \
	        m_c5:%.3fA , m_s5:%.3fdps, m_a5:%.3fdu\t\r\n               \
	        m_c6:%.3fA , m_s6:%.3fdps, m_a6:%.3fdu\t\r\n"              
	, LinAcr_position_float, LinAcr_current_float, LinAcr_speed_float, ADC_Pressure_Value, motor_current_float[2], motor_speed_float[2], motorAngle_float[2]
	                                                                                     , motor_current_float[3], motor_speed_float[3], motorAngle_float[3]
	                                                                                     , motor_current_float[4], motor_speed_float[4], motorAngle_float[4]
	                                                                                     , motor_current_float[5], motor_speed_float[5], motorAngle_float[5]
	                                                                                     , motor_current_float[6], motor_speed_float[6], motorAngle_float[6]);
}


//�����ظ���ȡ�ؽڽ��ٶȡ�Ť�ص�������Ϣ���ڿ����ж��ж�ȡ���ؽ�
void driv_ans_singlejoint(int id)
{
	if(id == 1)
	{
		LinearActuator_read_position(id);
		LinearActuator_read_CurrentandSpeed(id);
		pressure_SensorReading();
	}
	else
	{
		*(uint8_t *)(&motor_current) = CAN_motor_PosCurntSpd_ans[2];
		*((uint8_t *)(&motor_current)+1) = CAN_motor_PosCurntSpd_ans[3];
		motor_current_float[id] = (float)((int16_t)motor_current/2048.0*33.0);
		
		*(uint8_t *)(&motor_speed) = CAN_motor_PosCurntSpd_ans[4];
		*((uint8_t *)(&motor_speed)+1) = CAN_motor_PosCurntSpd_ans[5];
//		motor_speed_float[id] = (int16_t)motor_speed;
		switch(id)
		{
			case 6: motor_speed_float[id] = (float)((int16_t)motor_speed/10.0); break;
			case 5: motor_speed_float[id] = (float)((int16_t)motor_speed/8.0); break;
			case 4: motor_speed_float[id] = (float)((int16_t)motor_speed/36.0); break;
			case 3: motor_speed_float[id] = (float)((int16_t)motor_speed/8.0); break;
			case 2: motor_speed_float[id] = (float)((int16_t)motor_speed/36.0); break;
			default:printf("id error\n"); break;
		}
    
		read_angle(id);		
//		read_acc(id);
	}
}
