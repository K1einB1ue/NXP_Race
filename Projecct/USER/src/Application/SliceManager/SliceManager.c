#include"./SliceManager.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ʱ��Ƭ������ѹ��һ��ʱ��Ƭ
//  @param      this_       ����������ָ��
//  @param      action      ʱ��Ƭ����(����Ҫ������)
//  @param      arg         ����action�Ĳ���(���û����action������Ҫʹ��void* arg),������action�еݹ����.
//  @param      finish      ������־����(��������ʹ���䷵��ֵ������ֵ) 
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
//  @brief      ʱ��Ƭ��������ʼ������
//  @param      this_       ����������ָ��
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
//  @brief      ʱ��Ƭ���������½ӿ�
//  @param      this_       ����������ָ��
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
  //����������֡�Ƿ���н�����־����
  if(this_->flag_buffer[this_->order]){
    if(!this_->first_invoke_flag){
      //��������
      this_->first_invoke_flag = 1;
      if(this_->args_buffer[this_->order]){
        ((void(*)(void*))this_->action_buffer[this_->order])(this_->args_buffer[this_->order]);
      }else{
        ((void(*)(void))this_->action_buffer[this_->order])();
      }
    }
    //�鿴�����Ƿ����
    if (((uint8(*)(void))this_->flag_buffer[this_->order])()) {
      this_->first_invoke_flag = 0;
      if(TryCallNext(this_)){
        return;
      }
    }
  }else{        //û����˵����һ��action
    ((void(*)(void))this_->action_buffer[this_->order])();
    if(TryCallNext(this_)){
      return;
    }
  }
}

void SliceManager_prepare(SliceManager* this_){
  this_->is_finish = 0;
}