#include"./SliceUpdater.h"

void SliceUpdater_push(SliceUpdater* this_, void (*action)(void*)){
  this_->slice_buffer[this_->size++]=(uint32)action;
}

void SliceUpdater_init(SliceUpdater* this_){
  this_->order = 0;
  this_->size = 0;
}

void SliceUpdater_update(SliceUpdater* this_){
  if(this_->order == this_->size){
    this_->order = 0;
  }
  ((void(*)(void))this_->slice_buffer[this_->order])();
   this_->order++;
}