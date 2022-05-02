#pragma once
#include"../Dependency.h"


void BlueTooth_init();
void BlueTooth_send(char* str,uint16 size);
uint32 BlueTooth_printf(const char *format, ...);
void BlueTooth_sendStr(char* str);
void BlueTooth_update();
extern void BlueTooth_decode(const char* str,uint8 size);