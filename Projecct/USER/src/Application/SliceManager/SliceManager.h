#pragma once

#include"../../HardWare/HardWare.h"

/*使用例:
  SliceManager test;

  //其它的头文件或源文件文件中
    uint8 myflag = false;
    void run() {
        static int cnt = 0;
        cnt++;
        if (cnt > 100) {
            myflag = true;
        }
        printf("test%d\n",cnt);
    }

  int main(){
    SliceManager_init(&test);
    SliceManager_push(&test,run,&myflag);
    while(!test.is_finish){
      SliceManager_update(&test);
    }
    return 0;
  }
*/

//队列大小,可以根据需要拓展
#define SliceManagerSize 64
//面向对象的顺序动画切片
typedef struct {
    uint32 order;     //当前的状态机
    uint32 size;      //状态机总数
    union
    {
        struct
        {
            uint8 is_loop : 1;        //是否循环播放
            uint8 is_reversed1 : 1;
            uint8 is_reversed2 : 1;
            uint8 is_reversed3 : 1;
            uint8 is_reversed4 : 1;
            uint8 is_reversed5 : 1;
            uint8 is_reversed6 : 1;
            uint8 is_finish : 1;      //全部结束标志位(如果循环播放则不会起效)
        };
        uint8 flagbuffer;
    };
    uint8 first_invoke_flag;
    uint32 action_buffer[SliceManagerSize];           //动画函数指针
    uint32 flag_buffer[SliceManagerSize];             //动画结束标志函数指针
    void* args_buffer[SliceManagerSize];
}SliceManager;

//添加一个动画,是一个队列 queue结构.
//this_:
void SliceManager_push(SliceManager* this_, void (*action)(void*),void* arg,uint8 (*finish)(void));
void SliceManager_push_act(SliceManager* this_, void (*action)(void),uint8 (*finish)(void));
void SliceManager_push_action(SliceManager* this_, void (*action)(void));
//SliceManager的构造函数
void SliceManager_init(SliceManager* this_);
//SliceManager的更新函数
void SliceManager_update(SliceManager* this_);
void SliceManager_prepare(SliceManager* this_);





  