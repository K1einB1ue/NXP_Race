#pragma once

#include"../Dependency.h"

void ESP8266_init();
cJSON* ESP8266_update();
void ESP8266_reset();
void ESP8266_connect(const char* IPaddr,uint32 port);
void ESP8266_tryExitPassThrough();
uint8 ESP8266_ping(const char* IPaddr);
void ESP8266_Socket_start();
void ESP8266_Connect_wifi(const char* SSID,const char* Password);
#ifdef ESP8266_ResetPin
void ESP8266_hardware_reset();
#endif