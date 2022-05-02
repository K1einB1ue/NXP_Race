#pragma once

#include "./BlueTooth/BlueTooth.h"
#include "./Tracer/Tracer.h"
#include "./HWT101/HWT101.h"
#include "./OpenArt/OpenArt.h"
#include "./Motor/Motor.h"
#include "./SYN6288/SYN6288.h"
#include "./ESP8266/ESP8266.h"
extern void (*InitCallback)(void);
void Init();
