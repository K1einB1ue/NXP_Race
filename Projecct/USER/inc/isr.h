#pragma once

#include"./Application/Application.h"
extern int32 stateNum;
extern volatile int32 keyNum;
extern float floatBuffer[];
extern int floatCnt;
extern SliceManager Manager;
extern SliceManager Manager0;
extern SliceManager Manager1;
extern SliceUpdater Updater_Motor_PID;
extern SliceUpdater Updater_All_PID;

extern uint8 MapEnable;
extern uint8 __MOTION__;