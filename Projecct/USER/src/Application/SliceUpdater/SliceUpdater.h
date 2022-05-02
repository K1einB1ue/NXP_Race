#pragma once

#include"../../HardWare/HardWare.h"
//队列大小,可以根据需要拓展
#define SliceUpdaterSize 64
//面向对象的顺序动画切片
typedef struct {
    uint32 order;     //当前的状态机
    uint32 size;      //状态机总数
    uint32 slice_buffer[SliceUpdaterSize];           //动画函数指针
}SliceUpdater;

//添加一个动画,是一个队列 queue结构.
//this_:
void SliceUpdater_push(SliceUpdater* this_, void (*action)(void*));
//SliceManager的构造函数
void SliceUpdater_init(SliceUpdater* this_);
//SliceManager的更新函数
void SliceUpdater_update(SliceUpdater* this_);
