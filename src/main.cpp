#include <Arduino.h>
#include <servoICS.h>

#include "FootIK.h"
#include "PS4Controller_support.h"
#include "controller_to_command.h"
#include "esp_timer.h"



#include <math.h>


struct {
  char J1;
  char J2;
  char J3;
  char J4;
  char J5;
}typedef footServoID;



/*宣言・初期化・定数*/
PS4Controller_support Dualshock4;
ControllerApp::CommandConverter OpeCom(&Dualshock4.data_support);
const char ControllerMac[18] = "06:02:01:02:05:10";
const long bpsPC = 115200;
//サーボとの通信設定
HardwareSerial* ServoSerial = &Serial1;
const char enPin = 23;
const char txPin = 33;
const char rxPin = 19;
const long bpsServo = 115200;
//サーボの設定。
footServoID footRight = {0,1,2,3,4};





float tread_y(float h,float T,float Duty,float ts_){
    float A  = 2*h / (1-Duty);
    float AngV = PI*4/((1-Duty)*T);
    float t_1 = Duty*T/2;
    float t_2 = T/2;
    float t_3 = T-Duty*T/2;

    float y_ = 0.0;

    if (ts_ < t_1) {
        y_ = 0;
    }else if (ts_ < t_2) {
        y_ = -A / (2 * AngV) * sin(AngV * (ts_ - t_1)) + A / 2 * (ts_ - t_1);
    }else if(ts_ < t_3) {
        y_ = A / (2 * AngV) * sin(AngV * (ts_ - t_2)) - A / 2 * (ts_ - t_2) + (1.0 - Duty) * T * A / 4.0;
    }else {
        y_ = 0;
    }
    return y_;
}


float tread_x(float Wd,float T,float Duty,float ts_){

    float A  = 2*Wd / (1-Duty);
    float AngV = PI*2/((1-Duty)*T);
    float t_1 = Duty*T/2;
    float t_2 = T-Duty*T/2;

    float x_ = 0.0;

    if (ts_ < t_1) {
        x_ = -Wd * ts_;
    } 
    else if (ts_ < t_2) {
        float dt = ts_ - t_1;
        x_ = -A / (2.0f * AngV) * sinf(AngV * dt) + (A * 0.5f - Wd) * dt - Wd * t_1;
    } 
    else {
        x_ = -Wd * (ts_ - t_2) + Wd * t_1;
    }
    return x_;
}
 

int executionCycil(float cycilTime, float offsetTime, float fps, void (*p_func)(float)) {
    int64_t start_time = esp_timer_get_time();
    int64_t total_duration_us = (int64_t)(cycilTime * 1000000.0f);
    int64_t intervalTime = (int64_t)(1000000.0f / fps);
    int64_t nextTime = start_time + intervalTime;

    while (true) {
        int64_t now = esp_timer_get_time();
        int64_t elapsed_us = now - start_time;

        // 1. ループの先頭で終了判定を行う（確実）
        if (elapsed_us >= total_duration_us) {
            Serial.println("Cycle Finished.");
            break;
        }

        // 2. 関数の実行
        p_func((float)elapsed_us * 0.000001f);

        // 4. 次のフレームまで待機
        while ((esp_timer_get_time() - start_time) < (nextTime - start_time)) {
            delay(1);
        }
        nextTime += intervalTime;
    }
    return 0;
}


/*
namespace walk{
  float x,y;

  bool isForward;
  bool isBackward;
  bool isMoveRight;
  bool isMoveLeft;


  
  void update(){
    x = (float)map(DS4->LStickX(),-128,127,-1000,1000) * 0.1;
    y = (float)map(DS4->LStickY(),-128,127,-1000,1000) * 0.1;

    if(x > 50){
      isForward = true;
      isBackward = false;
    }else if(x < -50){
      isForward = false;
      isBackward = true;
    }else{
      isForward = false;
      isBackward = false;
    }

    if(y > 50){
      isMoveRight = true;
      isMoveLeft = false;
    }else if(y < -50){
      isMoveLeft = true;
      isMoveRight = false;
    }else{
      isMoveRight = false;
      isMoveLeft = false;
    }
  }

  float T;
  float fps;
  float h;
  float Wd;
  float DutyX;
  float DutyY;



  void treadxy(float t){
    float y = tread_y(h,T,DutyY,t);
    float x = tread_x(Wd,T,DutyX,t);

    //グラフ作成[*,+]
    for(int i=-50;i<int(y);i++){
        Serial.printf("*");
    }
    Serial.printf("\n");
    for(int i=-50;i<int(x);i++){
        Serial.printf("+");
    }
    Serial.printf("\n");
  }

  void walk_play(float T_,float fps_, float h_, float Wd_,float DutyX_,float DutyY_){
    T = T_;
    fps = fps_;
    h = h_;
    Wd = Wd_;
    DutyX = DutyX_;
    DutyY = DutyY_;

    executionCycil(T,0,fps,treadxy);
  } 
}
*/


void WaitCom(){
  if (Dualshock4.isConnected()) {
    Dualshock4.update();
    OpeCom.update();
    ControllerApp::Commands cmd = OpeCom.getCommands();

    char str[1024];

    sprintf(str,
    "[controller]\n"
    "[X]:%.2f [Y]:%.2f\n"
    "[isAttack1]:%d [isAttack2]:%d [isAttack3]:%d [isAttack4]:%d\n"
    "[isGetup]:%d [isSquat]:%d\n",
    cmd.moveSpeed.x, cmd.moveSpeed.y,
    cmd.isAttack1, cmd.isAttack2, cmd.isAttack3, cmd.isAttack4,
    cmd.isGetup, cmd.isSquat
    );

    Serial.printf(str);

    if(cmd.moveSpeed.y != 0 || cmd.moveSpeed.x != 0){
      //並行移動
    }else if(cmd.moveAngle != 0){
      //水平視点移動
    }else if(cmd.isAttack1){
      //攻撃１
    }else if(cmd.isAttack2){
      //攻撃２
    }else if(cmd.isAttack3){
      //攻撃３
    }else if(cmd.isAttack4){
      //攻撃４  
    }else if(cmd.isGetup){
      //起き上がり
    }else if(cmd.isSquat){
      //しゃがみ
    }

    
  }

}


//左足サーボ
servoICS::Servo leftFoot_J1(&Serial,0,1);
servoICS::Servo leftFoot_J2(&Serial,0,2);
servoICS::Servo leftFoot_J3(&Serial,0,3);
servoICS::Servo leftFoot_J4(&Serial,0,4);
servoICS::Servo leftFoot_J5(&Serial,0,5);
//右足サーボ
servoICS::Servo rightFoot_J1(&Serial,0,11);
servoICS::Servo rightFoot_J2(&Serial,0,12);
servoICS::Servo rightFoot_J3(&Serial,0,13);
servoICS::Servo rightFoot_J4(&Serial,0,14);
servoICS::Servo rightFoot_J5(&Serial,0,15);



void setup() {


  Serial.begin(bpsPC,SERIAL_8E1);
  
  //FootIK::leng8 lengs8={60,100,40,100,100,40,100,60};
  FootIK::leng8 lengs8={18.75,49,20.96,150.04,150.04,20.96,49,18.75};
  
  while(1){
    for(float i=-90;i<90;i++){
      FootIK::Pose poses_={
          260,50,i,
          0,0,(float)servoICS::fromDeg_toRad(i)
      };
      FootIK::footJoint5 joint = FootIK::IK(poses_, lengs8,0);

      auto log1 = leftFoot_J1.setPosRad(joint.J1);
      auto log2 = leftFoot_J2.setPosRad(joint.J2);
      auto log3 = leftFoot_J3.setPosRad(joint.J3);
      auto log4 =leftFoot_J4.setPosRad(joint.J4);
      auto log5 = leftFoot_J5.setPosRad(joint.J5);

      Serial.printf("[i:%d] J1:%0.2f,J2:%0.2f,J3:%0.2f,J4:%0.2f,J5:%0.2f\n",(int)i,servoICS::fromRad_toDeg(joint.J1),servoICS::fromRad_toDeg(joint.J2),servoICS::fromRad_toDeg(joint.J3),servoICS::fromRad_toDeg(joint.J4),servoICS::fromRad_toDeg(joint.J5));
      Serial.printf("\nlog1:%s,log2:%s,\nlog3:%s,log4:%s,\nlog5:%s\n",log1.returnStatus().error_msg,log2.returnStatus().error_msg,log3.returnStatus().error_msg,log4.returnStatus().error_msg,log5.returnStatus().error_msg);

      delay(10);
    }
    delay(2000);

    for(float i=-90;i<90;i++){
      FootIK::Pose poses_={
          260,50,i,
          0,0,(float)servoICS::fromDeg_toRad(i)
      };
      FootIK::footJoint5 joint = FootIK::IK(poses_, lengs8,0);

      auto log1 = leftFoot_J1.setPosRad(joint.J1);
      auto log2 = leftFoot_J2.setPosRad(joint.J2);
      auto log3 = leftFoot_J3.setPosRad(joint.J3);
      auto log4 =leftFoot_J4.setPosRad(joint.J4);
      auto log5 = leftFoot_J5.setPosRad(joint.J5);
      delay(20);
    }
    delay(2000);

    for(float i=0;i<=100;i++){
      FootIK::Pose poses_={
          440,0,0,
          0,0,0
      };
      FootIK::footJoint5 joint = FootIK::IK(poses_, lengs8,0);

      auto log1 = leftFoot_J1.setPosRad(joint.J1);
      auto log2 = leftFoot_J2.setPosRad(joint.J2);
      auto log3 = leftFoot_J3.setPosRad(joint.J3);
      auto log4 =leftFoot_J4.setPosRad(joint.J4);
      auto log5 = leftFoot_J5.setPosRad(joint.J5);
      delay(20);
    }
    delay(2000);

  }




  ServoSerial->begin(bpsServo,SERIAL_8E1,rxPin,txPin);  //SERIAL_8E1がICS規格で使用されている。

  Dualshock4.begin(ControllerMac);
  Dualshock4.update();





}

int64_t setTime;



void loop() {
  WaitCom();
  delay(500);
}