#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdlib.h"
#include "usart.h"

extern uint8_t wifisendbuf[6][24];

#define SSID "JJCWIFI" //WIFIÃû
#define PASS "password6666"   //WIFIÃÜÂë

#define IPBUF "192.168.4.1"   //IPµØÖ·
#define PORTNUM "8086"        //¶Ë¿ÚºÅ

void read_handskeleton();

uint8_t esp8266_send_cmd(uint8_t *cmd, uint8_t *ack, uint16_t waittime);
uint8_t atk_8266_send_data(uint8_t *data, uint8_t *ack, uint16_t waittime);
uint8_t* esp8266_check_cmd(uint8_t *str);
uint8_t esp8266_Connect_IOTServer(void);
uint8_t esp8266_client_config(void);
uint8_t esp8266_Set_PORTNUM(void);
uint8_t esp8266_Connect_AP(void);
uint8_t esp8266_Connect_Server(void);
uint8_t esp8266_server_config(void);
uint8_t esp8266_quit_trans(void);


#endif


