#include"./SliceManager.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      时间片管理器压入一个时间片
//  @param      this_       管理器对象指针
//  @param      action      时间片函数(你需要做的事)
//  @param      arg         传入action的参数(如果没有则action函数不要使用void* arg),可以在action中递归调用.
//  @param      finish      结束标志函数(这样可以使得其返回值惰性求值) 
//  @return     void
//  @since      v1.0
//  Sample usage:           SliceManager_push(&Manager,ToPosition,Target+0,GetFlag);
//  @note                   ****
//-------------------------------------------------------------------------------------------------------------------
void SliceManager_push(SliceManager* this_, void (*action)(void*),void* arg,uint8 (*finish)(void)){
  this_->action_buffer[this_->size] = (uint32)action;
  this_->flag_buffer[this_->size] = (uint32)finish;
  this_->args_buffer[this_->size] = arg;
  this_->size++;
}

void SliceManager_push_act(SliceManager* this_, void (*action)(void),uint8 (*finish)(void)){
  this_->action_buffer[this_->size] = (uint32)action;
  this_->flag_buffer[this_->size] = (uint32)finish;
  this_->args_buffer[this_->size] = 0;
  this_->size++;
}

void SliceManager_push_action(SliceManager* this_, void (*action)(void)){
  this_->action_buffer[this_->size] = (uint32)action;
  this_->flag_buffer[this_->size] = 0;
  this_->args_buffer[this_->size] = 0;
  this_->size++;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      时间片管理器初始化函数
//  @param      this_       管理器对象指针
//  @return     void
//  @since      v1.0
//  Sample usage:           SliceManager_init(&Manager);
//  @note                   ****
//-------------------------------------------------------------------------------------------------------------------
void SliceManager_init(SliceManager* this_) {
  this_->order = 0;
  this_->size = 0;
  this_->is_loop = 0;
  this_->is_finish = 0;
  this_->first_invoke_flag = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      时间片管理器更新接口
//  @param      this_       管理器对象指针
//  @return     void
//  @since      v1.0
//  Sample usage:           SliceManager_update(&Manager);
//  @note                   ****
//-------------------------------------------------------------------------------------------------------------------
uint8 TryCallNext(SliceManager* this_){
  if (this_->order < this_->size - 1) {
    this_->order++;
  }else {
    if (this_->is_loop) {
      this_->order = 0;
    }else {
      this_->is_finish = 1;
      return 1;
    }
  }
  return 0;
}

void SliceManager_update(SliceManager* this_) {
  if (this_->size == 0) {
    this_->is_finish = 1;
    return;
  }
  //如果这个动画帧是否具有结束标志函数
  if(this_->flag_buffer[this_->order]){
    if(!this_->first_invoke_flag){
      //开启动画
      this_->first_invoke_flag = 1;
      if(this_->args_buffer[this_->order]){
        ((void(*)(void*))this_->action_buffer[this_->order])(this_->args_buffer[this_->order]);
      }else{
        ((void(*)(void))this_->action_buffer[this_->order])();
      }
    }
    //查看动画是否结束
    if (((uint8(*)(void))this_->flag_buffer[this_->order])()) {
      this_->first_invoke_flag = 0;
      if(TryCallNext(this_)){
        return;
      }
    }
  }else{        //没有则说明是一个action
    ((void(*)(void))this_->action_buffer[this_->order])();
    if(TryCallNext(this_)){
      return;
    }
  }
}

void SliceManager_prepare(SliceManager* this_){
  this_->is_finish = 0;
}