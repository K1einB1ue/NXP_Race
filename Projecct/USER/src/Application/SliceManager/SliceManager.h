#pragma once

#include"../../HardWare/HardWare.h"

/*ʹ����:
  SliceManager test;

  //������ͷ�ļ���Դ�ļ��ļ���
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

//���д�С,���Ը�����Ҫ��չ
#define SliceManagerSize 64
//��������˳�򶯻���Ƭ
typedef struct {
    uint32 order;     //��ǰ��״̬��
    uint32 size;      //״̬������
    union
    {
        struct
        {
            uint8 is_loop : 1;        //�Ƿ�ѭ������
            uint8 is_reversed1 : 1;
            uint8 is_reversed2 : 1;
            uint8 is_reversed3 : 1;
            uint8 is_reversed4 : 1;
            uint8 is_reversed5 : 1;
            uint8 is_reversed6 : 1;
            uint8 is_finish : 1;      //ȫ��������־λ(���ѭ�������򲻻���Ч)
        };
        uint8 flagbuffer;
    };
    uint8 first_invoke_flag;
    uint32 action_buffer[SliceManagerSize];           //��������ָ��
    uint32 flag_buffer[SliceManagerSize];             //����������־����ָ��
    void* args_buffer[SliceManagerSize];
}SliceManager;

//���һ������,��һ������ queue�ṹ.
//this_:
void SliceManager_push(SliceManager* this_, void (*action)(void*),void* arg,uint8 (*finish)(void));
void SliceManager_push_act(SliceManager* this_, void (*action)(void),uint8 (*finish)(void));
void SliceManager_push_action(SliceManager* this_, void (*action)(void));
//SliceManager�Ĺ��캯��
void SliceManager_init(SliceManager* this_);
//SliceManager�ĸ��º���
void SliceManager_update(SliceManager* this_);
void SliceManager_prepare(SliceManager* this_);





  