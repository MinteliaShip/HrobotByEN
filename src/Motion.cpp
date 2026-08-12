#include "Motion.h"

void motion::walk::walk1(){
    Serial.printf("walk!\n");
    FrameLimiter framelim;
    framelim.setInterval(int(1000/30));//30fps設定

    for(int i = 0;i < 90;i++){
        Serial.printf("c:%d\n",i);
        taskManager(0);//移行許可は出さない。
    }
    return;//処理終わり
}

void motion::posture::battle::attack1(){
}

void motion::posture::battle::attack2(){
}

void motion::posture::nop(){
    Serial.printf("nop!\n");
    delay(10);
}

