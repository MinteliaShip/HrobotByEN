#include <Arduino.h>
#include <servoICS.h>

#include <WiFi.h>

#include <PS4Controller.h>
#include <esp_gap_bt_api.h>

#include "controller_to_command.h"

#include "FootController.h"

#include <math.h>


/*宣言・初期化・定数*/
PS4Controller Dualshock4;
ControllerApp::CommandConverter OpeCom(&Dualshock4.data);
const char ControllerMac[18] = "06:02:01:02:05:10";
const long bpsPC = 115200;
//サーボとの通信設定
HardwareSerial* ServoSerial = &Serial1;
const char enPin = 23;
const char txPin = 33;
const char rxPin = 19;
const long bpsServo = 115200;

//足寸法
FootController::leng8 lengs8={18.75,49,20.96,150.04,150.04,20.96,49,18.75};
FootController::IcsServoConfig leftfootConfig{
  9,
  12,
  13,
  14,
  15,
  enPin,
  &Serial
};
FootController::IcsServoConfig rightfootConfig{
  10,
  16,
  17,
  18,
  19,
  enPin,
  &Serial
};
//足コントロール
FootController leftFoot(leftfootConfig,lengs8);
FootController rightFoot(rightfootConfig,lengs8);
//歩行関数定数
float MV_X_T = 0.8;
float MV_X_fps = 40;
float MV_X_h = 20;
float MV_X_Wd = 150;
float MV_X_DutyX = 0.2;
float MV_X_DutyY = 0.18;

//アイドル時の設定
float IDLE_fps = 5;

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
//0 49 50 99
//50 99 0 49

long phaseShift(long inStep,long phaseShift){
  long result = 0;
  if(phaseShift>inStep){
    result = inStep + phaseShift;
  }else{
    result = inStep - phaseShift;
  }
  return result;
}

void bondReset(){
  // 1. 保存されているデバイスの数を確認
  int dev_num = esp_bt_gap_get_bond_device_num();

  if (dev_num > 0) {
    esp_bd_addr_t dev_list[dev_num];
    // 2. デバイスリストを取得
    esp_bt_gap_get_bond_device_list(&dev_num, dev_list);
    
    // 3. 全てのデバイス情報を削除（リセット）
    for (int i = 0; i < dev_num; i++) {
      esp_bt_gap_remove_bond_device(dev_list[i]);
    }
  }
}



bool MV_X_F(long motionTime){

  float T = MV_X_T;
  float h = MV_X_h;
  float Wd = MV_X_Wd;
  float DutyX = MV_X_DutyX;
  float DutyY = MV_X_DutyY;

  ControllerApp::Commands cmd = OpeCom.getCommands();

  //左足計算式
  float ly = tread_y(h,T,DutyY,motionTime*0.001);
  float lx = tread_x(Wd*(-cmd.moveSpeed.y*0.01),T,DutyX,motionTime*0.001);

  FootController::Pose leftPose={
    420-ly,0,lx,
    0,0,0
  };
  leftFoot.setTargetPose(leftPose);

  float ry = tread_y(h,T,DutyY,phaseShift(motionTime,(long)(T*500))*0.001);
  float rx = tread_x(Wd*(-cmd.moveSpeed.y*0.01),T,DutyX,phaseShift(motionTime,(long)(T*500))*0.001);
  FootController::Pose rightPos={
    420-ry,0,rx,
    0,0,0
  };
  rightFoot.setTargetPose(rightPos);

  if((long)(T*1000) > motionTime){
    return false;
  }else{
    return true;
  }
}

void IDLE_F(){
  //直立
  FootController::Pose Pose={
    420,0,0,
    0,0,0
  };
  leftFoot.setTargetPose(Pose);
  rightFoot.setTargetPose(Pose);
}

enum RobotState {
  IDLE,
  MV_X,
  MV_Y,
  MV_TURN,
  ACT_ATTACK1,
  ACT_ATTACK2,
  ACT_ATTACK3,
  ACT_ATTACK4,
  ACT_GETUP
};

RobotState robotState = IDLE;

int lastRecvTime=0;
bool stateUpdate(){
  static RobotState robotStateLast;
  if (Dualshock4.isConnected()) {
    OpeCom.update();
    ControllerApp::Commands cmd = OpeCom.getCommands();

    if(cmd.moveSpeed.y != 0){
      //前進後退
      robotState = MV_X;
    }else if(cmd.moveSpeed.x != 0){
      //左右移動
      robotState = MV_Y;
    }else if(cmd.moveAngle != 0){
      //方向変更
      robotState = MV_TURN;
    }else if(cmd.isAttack1){
      //攻撃１
      robotState = ACT_ATTACK1;
    }else if(cmd.isAttack2){
      //攻撃２
      robotState = ACT_ATTACK2;
    }else if(cmd.isAttack3){
      //攻撃３
      robotState = ACT_ATTACK3;
    }else if(cmd.isAttack4){
      //攻撃４  
      robotState = ACT_ATTACK4;
    }else if(cmd.isGetup){
      //起き上がり
      robotState = ACT_GETUP;
    }else{
      //IDLE
      robotState = IDLE;
    }
    lastRecvTime = millis();
  }else{
    if(lastRecvTime+1000 < millis()){//最終接続から1秒経過しても接続されない場合。
      robotState = IDLE;
    }
  }

  if(robotStateLast != robotState){//変化があれば1 なければ0
    robotStateLast = robotState;
    return 1;
  }else{
    robotStateLast = robotState;
    return 0;
  }

}


// モーションの進捗管理用
int motionStep = 0;
long motionTimeOrigin = 0;
void motorTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(long(1000.0 / MV_X_fps)); 

  while(1){
    if (stateUpdate()) {
      motionStep = 0; 
      motionTimeOrigin = millis(); 
    }
    long motionTime = millis() - motionTimeOrigin;


    switch (robotState) {
      case MV_X:
        if(MV_X_F(motionTime)){
          motionTimeOrigin = millis(); 
          motionStep = 0; 
        }
        break;

      case IDLE:
        IDLE_F();
        TickType_t xFrequency = pdMS_TO_TICKS(long(1000.0 / MV_X_fps)); 
        break;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(bpsPC,SERIAL_8E1);
  ServoSerial->begin(bpsServo,SERIAL_8E1,rxPin,txPin);  //SERIAL_8E1がICS規格で使用されている。
  Dualshock4.begin(ControllerMac);
  bondReset();


  xTaskCreateUniversal(
    motorTask,      // 関数名
    "motorTask",    // タスク名
    4096,           // スタックサイズ
    NULL,           // パラメータ
    1,              // 優先度
    NULL,           // タスクハンドル
    1               // 実行するコア (0 or 1)
  );


}


void loop() {
  delay(1000);
}