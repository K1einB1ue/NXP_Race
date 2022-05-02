#pragma once
#include"../Dependency.h"

void OpenArt_init();
void OpenArt_update();
void OpenArt_send(char* str,uint16 size);
extern void OpenArt_decode(const char* str,uint8 size);