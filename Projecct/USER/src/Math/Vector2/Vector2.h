#pragma once

#include<math.h>
typedef struct _Vector2_{
  float x;
  float y;
}Vector2;

void normalize(Vector2* this_);
void mul_vec(Vector2* this_,Vector2* that_);
void mul_val(Vector2* this_,float val);
