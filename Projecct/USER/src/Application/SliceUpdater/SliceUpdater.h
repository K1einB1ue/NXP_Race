#pragma once

#include"../../HardWare/HardWare.h"
//���д�С,���Ը�����Ҫ��չ
#define SliceUpdaterSize 64
//��������˳�򶯻���Ƭ
typedef struct {
    uint32 order;     //��ǰ��״̬��
    uint32 size;      //״̬������
    uint32 slice_buffer[SliceUpdaterSize];           //��������ָ��
}SliceUpdater;

//���һ������,��һ������ queue�ṹ.
//this_:
void SliceUpdater_push(SliceUpdater* this_, void (*action)(void*));
//SliceManager�Ĺ��캯��
void SliceUpdater_init(SliceUpdater* this_);
//SliceManager�ĸ��º���
void SliceUpdater_update(SliceUpdater* this_);
