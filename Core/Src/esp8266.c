#include "esp8266.h"
#include "tim.h"
#include "usart.h"


/*

向ESP8266发送命令
cmd:发送的命令字符串
ack:期待的应答结果,如果为空,则表示不需要等待应答
waittime:等待时间(单位:10ms)
返回值:0,发送成功(得到了期待的应答结果)
       1,发送失败

*/

uint8_t esp8266_send_cmd(uint8_t *cmd, uint8_t *ack, uint16_t waittime)
{
	uint8_t res = 0;
	USART3_RX_STA = 0;
	memset(USART3_RX_BUF,0,USART3_MAX_RECV_LEN);
	u3_printf("%s\r\n",cmd);
	if(waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			Delay_ms(10);
			if(USART3_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(esp8266_check_cmd(ack))
				{
					u1_printf("ack:%s\r\n",(uint8_t*)ack);
					break;//得到有效数据 
				}
				USART3_RX_STA=0;
			}
		}
		if(waittime==0)
			res=1; 
	}
	return res;
}


/*

向ESP8266发送指定数据
data:发送的数据(不需要添加回车了)
ack:期待的应答结果,如果为空,则表示不需要等待应答
waittime:等待时间(单位:10ms)
返回值:0,发送成功(得到了期待的应答结果)
	
*/

uint8_t atk_8266_send_data(uint8_t *data, uint8_t *ack, uint16_t waittime)
{
	uint8_t res=0; 
	USART3_RX_STA=0;
	u3_printf("%s",data);	//发送数据
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			Delay_ms(10);
			if(USART3_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(esp8266_check_cmd(ack))break;//得到有效数据 
				USART3_RX_STA=0;
			} 
		}
		if(waittime==0)
			res=1; 
	}
	return res;
}


/*

ATK-ESP8266发送命令后,检测接收到的应答
str:期待的应答结果
返回值:0,没有得到期待的应答结果
    其他,期待应答结果的位置(str的位置)
	
*/

uint8_t* esp8266_check_cmd(uint8_t *str)
{
	
	char *strx=0;
	if(USART3_RX_STA&0X8000)		//接收到一次数据了
	{ 
		u1_printf("%s\r\n",(char*)USART3_RX_BUF);
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//添加结束符
		
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	}
	return (uint8_t*)strx;
}


/*

ATK-ESP8266退出透传模式
返回值:0,退出成功;
       1,退出失败	

*/

uint8_t esp8266_quit_trans(void)
{
	while((USART3->SR&0X40)==0);	//等待发送空
	USART3->DR='+';      
	HAL_Delay(15);					//大于串口组帧时间(10ms)
	while((USART3->SR&0X40)==0);	//等待发送空
	USART3->DR='+';      
	HAL_Delay(15);					//大于串口组帧时间(10ms)
	while((USART3->SR&0X40)==0);	//等待发送空
	USART3->DR='+';      
	HAL_Delay(500);					//等待500ms
	return esp8266_send_cmd("AT","OK",20);//退出透传判断.
}

uint8_t esp8266_Connect_IOTServer(void)
{
	u1_printf("准备配置模块\r\n");
	HAL_Delay(100);
	
	
	esp8266_send_cmd("AT","OK",50);
	u1_printf("准备退出透传模式\n");
	if(esp8266_quit_trans())
	{
		u1_printf("退出透传模式失败，准备重启\r\n");
		return 6;
	}else u1_printf("退出透传模式成功\r\n");
	
	
	
	u1_printf("准备关闭回显\r\n");
	if(esp8266_send_cmd("ATE0","OK",50))
	{
		u1_printf("关闭回显失败准备重启\r\n");
		return 1;
	}else u1_printf("关闭回显成功\r\n");
	
	u1_printf("查询模块是否在线\r\n");
	if(esp8266_send_cmd("AT","OK",50))
	{
		u1_printf("模块不在线准备重启\r\n");
		return 1;
	}else u1_printf("设置查询在线成功\r\n");
	
	
	
	
	u1_printf("准备设置STA模式\r\n");
	if(esp8266_send_cmd("AT+CWMODE=1","OK",50))
	{
		u1_printf("设置STA模式失败准备重启\r\n");
		return 1;
	}else u1_printf("设置STA模式成功\r\n");
	
	u1_printf("准备重启\r\n");
	if(esp8266_send_cmd("AT+RST","OK",50))
	{
		u1_printf("重启失败，准备重启\r\n");
		return 2;
	}else u1_printf("重启成功，等待三秒\r\n");
	
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	
	
	u1_printf("准备取消自动连接\r\n");
	if(esp8266_send_cmd("AT+CWAUTOCONN=0","OK",50))
	{
		u1_printf("取消自动连接失败，准备重启\r\n");
		return 3;
	}else u1_printf("取消自动连接成功\r\n");
	
	u1_printf("准备链接路由器\r\n");
	if(esp8266_Connect_AP())
	{
		u1_printf("连接路由器失败，等待重启\r\n");
		return 4;
	}else u1_printf("连接路由器成功\r\n");
	
	HAL_Delay(1000);
	HAL_Delay(1000);
	
	
	u1_printf("准备开启DHCP\r\n");
	if(esp8266_send_cmd("AT+CWDHCP=1,1","OK",100))
	{
		u1_printf("开启DHCP失败，准备重启\r\n");
		return 7;
	}else u1_printf("设置DHCP成功\r\n");
	
	
	u1_printf("设置为关闭多路连接\r\n");
	if(esp8266_send_cmd("AT+CIPMUX=0","OK",100))
	{
		u1_printf("关闭多路连接失败，准备重启\r\n");
		return 7;
	}else u1_printf("设置关闭多路连接成功\r\n");
	
	
	u1_printf("准备链接服务器\r\n");
	if(esp8266_Connect_Server())
	{
		u1_printf("连接服务器失败，等待重启\r\n");
		return 8;
	}else u1_printf("连接服务器成功\r\n");
	
	u1_printf("准备退出透传模式\n");
	if(esp8266_quit_trans())
	{
		u1_printf("退出透传模式失败，准备重启\r\n");
		return 6;
	}else u1_printf("退出透传模式成功\r\n");
	
	u1_printf("设置为透传模式\r\n");
	if(esp8266_send_cmd("AT+CIPMODE=1","OK",50))
	{
		u1_printf("设置透传失败，准备重启\r\n");
		return 6;
	}else u1_printf("设置透传成功\r\n");
	
	u1_printf("设置开启透传模式\r\n");
	if(esp8266_send_cmd("AT+CIPSEND","OK",1000))
	{
		u1_printf("开启透传失败，准备重启\r\n");
		return 9;
	}else u1_printf("开启透传成功\r\n");
	
	
	
	while(1){
		HAL_Delay(1000);
		//esp8266_send_cmd("AT+CIPSEND","OK",200);  //发送指定长度的数据
		u3_printf("This is a message\r\n");
	}
	return 0;
}

/*

客户端配置代码
返回值:0,连接成功;
       其他,连接失败	

*/

uint8_t esp8266_client_config(void)
{
	u1_printf("准备配置esp8266客户端模块\r\n");
	HAL_Delay(100);
	
	esp8266_send_cmd("AT","OK",50); //检查WIFI模块是否在线
	
	u1_printf("准备设置STA模式\r\n");
	if(esp8266_send_cmd("AT+CWMODE=1","OK",100)) //1.Station模式 2.Ap模式 3.兼容1.2两个模式
	{
		u1_printf("设置STA模式失败准备重启\r\n");
		return 1;
	}else u1_printf("设置STA模式成功\r\n");
	
	u1_printf("准备重启\r\n");
	if(esp8266_send_cmd("AT+RST","OK",50)) //重启复位
	{
		u1_printf("重启失败，准备重启\r\n");
		return 2;
	}else u1_printf("重启成功，等待四秒\r\n");
	Delay_ms(4000);
	
	u1_printf("准备链接WIFI\r\n");
	if(esp8266_Connect_AP()) //连接WIFI
	{
		u1_printf("连接WIFI失败，等待重启\r\n");
		return 3;
	}else u1_printf("连接WIFI成功\r\n");
	Delay_ms(2000);
	
	u1_printf("准备链接服务器\r\n");
	if(esp8266_Connect_Server()) //连接服务器
	{
		u1_printf("连接服务器失败，等待重启\r\n");
		return 4;
	}else u1_printf("连接服务器成功\r\n");
	
	return 0;
}


/*

服务器配置代码
返回值:0,连接成功;
       其他,连接失败	

*/

uint8_t esp8266_server_config(void)
{
	u1_printf("准备配置esp8266服务器模块\r\n");
	HAL_Delay(100);
	
	esp8266_send_cmd("AT","OK",50); //检查WIFI模块是否在线
	
	u1_printf("准备设置AP模式\r\n");
	if(esp8266_send_cmd("AT+CWMODE=2","OK",50)) //1.Station模式2.Ap模式3.兼容1.2两个模式
	{
		u1_printf("设置AP模式失败准备重启\r\n");
		return 1;
	}else u1_printf("设置AP模式成功\r\n");
	
	u1_printf("准备重启\r\n");
	if(esp8266_send_cmd("AT+RST","OK",50)) //重启复位，模式生效
	{
		u1_printf("重启失败，准备重启\r\n");
		return 2;
	}else u1_printf("重启成功，等待四秒\r\n");
	Delay_ms(4000);
	
	u1_printf("准备启动多连接\r\n");
	if(esp8266_send_cmd("AT+CIPMUX=1","OK",50)) //0：单连接，1：多连接
	{
		u1_printf("启动多连接失败，等待重启\r\n");
		return 3;
	}else u1_printf("启动多连接成功\r\n");
	
	u1_printf("准备建立服务器\r\n");
	if(esp8266_Set_PORTNUM()) //建立服务器server指令	
	{
		u1_printf("建立服务器失败，等待重启\r\n");
		return 4;
	}else u1_printf("建立服务器成功\r\n");

	
	return 0;
}


/*

设置服务器端口
返回值:0,连接成功;
       1,连接失败	

*/

uint8_t esp8266_Set_PORTNUM()
{
	uint8_t i=10;
	char *p = (char*)malloc(50);
	sprintf((char*)p,"AT+CIPSERVER=1,%s",PORTNUM);
	while(esp8266_send_cmd(p,"OK",300)&i)
	{
		u1_printf("设置服务器端口失败，尝试重新设置\r\n");
		i--;
	}
	free(p);
	if(i)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


/*

连接WIFI
返回值:0,连接成功;
       1,连接失败	

*/

uint8_t esp8266_Connect_AP()
{
	uint8_t i=10;
	char *p = (char*)malloc(50);
	sprintf((char*)p,"AT+CWJAP=\"%s\",\"%s\"",SSID,PASS);
	while(esp8266_send_cmd(p,"WIFI GOT IP",300)&i)
	{
		u1_printf("链接AP失败，尝试重新连接\r\n");
		i--;
	}
	free(p);
	if(i)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}



/*

连接服务器的IP地址和端口
返回值:0,连接成功;
       1,连接失败	

*/

uint8_t esp8266_Connect_Server()
{
	uint8_t i=10;
	char *p = (char*)malloc(50);
	sprintf((char*)p,"AT+CIPSTART=\"TCP\",\"%s\",%s",IPBUF,PORTNUM);
	while(esp8266_send_cmd(p,"CONNECT",300)&i)
	{
		u1_printf("链接服务器失败，尝试重新连接\r\n");
		i--;
	}
	free(p);
	if(i)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


