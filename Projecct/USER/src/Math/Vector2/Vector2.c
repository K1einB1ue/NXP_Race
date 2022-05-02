#include"./Vector2.h"


void normalize(Vector2* this_){
  float rate = sqrtf(this_->x*this_->x+this_->y*this_->y);
  this_->x/=rate;
  this_->y/=rate;
}

void mul_vec(Vector2* this_,Vector2* that_){
  this_->x*=that_->x;
  this_->y*=that_->y;
}

void mul_val(Vector2* this_,float val){
  this_->x*=val;
  this_->y*=val;
}