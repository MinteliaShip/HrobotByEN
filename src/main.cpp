#include <Arduino.h>
#include <servoICS.h>
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>

#include "controller_to_command.h"
#include "FootController.h"
#include "Config.h"
#include "ConfigDef.h"
#include "Vector.h"

/*宣言・初期化・定数*/
PS4Controller Dualshock4;
ControllerApp::CommandConverter OpeCom(&Dualshock4.data,Config::mapping);
FootController leftFoot(Config::leftfootConfig,Config::lengs8);
FootController rightFoot(Config::rightfootConfig,Config::lengs8);



/*-------------------------------------*/
//歩行軌道生成
float tread_y(float h,float T,float Duty,float ts_){
  float A  = 2.0*h / (1.0-Duty);
  float AngV = PI*4.0/((1.0-Duty)*T);
  float t_1 = Duty*T/2.0;
  float t_2 = T/2.0;
  float t_3 = T-Duty*T/2.0;

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

  float A  = 2.0*Wd / (1.0-Duty);
  float AngV = PI*2.0/((1.0-Duty)*T);
  float t_1 = Duty*T/2.0;
  float t_2 = T-Duty*T/2.0;
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

Vector2 tread(float h,float Wd,float DutyX,float DutyY,float T,float ts){
  Vector2 result;
  result.x = tread_x(Wd,T,DutyX,ts);
  result.y = tread_y(h,T,DutyY,ts);
  return result;
}
/*-------------------------------------*/
//歩行重心移動
float tread_z(float p,float T,float Duty,float ts_){
  float T_ = (2*Duty-1)*T;
  float av = 2*PI/T_;

  float section2 = T*(1-Duty)/2;
  float section3 = T/2*Duty;
  float section4 = T*(2-Duty)/2;
  float section5 = T*(1+Duty)/2;

  float z_ = 0.0;

  if (ts_ < section2) {//1
    z_ = p;
  } else if(ts_ < section3){//2
    z_ = p*cos(av*(ts_-section2));
  } else if(ts_ < section4){//3
    z_ = -p;
  } else if(ts_ < section5){//4
    z_ = -p*cos(av*(ts_-section4));
  } else{//5
    z_ = p;
  }

  return z_;
}
/*-------------------------------------*/
//サポート関数
//時間ずらし
long phaseShift(long inStep,long phaseShift){
  long result = 0;
  if(phaseShift>inStep){
    result = inStep + phaseShift;
  }else{
    result = inStep - phaseShift;
  }
  return result;
}

//ボード履歴削除
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

/*-------------------------------------*/
//動作関数
//前後方向移動
bool MV_X_F(long motionTime,GaitParameters GaitParameters_){
  leftFoot.setJointSkip(true);
  rightFoot.setJointSkip(true);

  #ifdef DEBUG
  Serial.printf("MV_X_F\n");
  #endif

  float T = GaitParameters_.T;
  float h = GaitParameters_.h;
  float Wd = GaitParameters_.Wd;
  float DutyX = GaitParameters_.DutyX;
  float DutyY = GaitParameters_.DutyY;

  ControllerApp::Commands cmd = OpeCom.getCommands();
  float z = tread_z(30,T,DutyY+0.1,motionTime*0.001);

  auto leftPos_ = tread(h,Wd*-cmd.move.y,DutyX,DutyY,T,motionTime*0.001);
  FootController::Pose leftPose={
    420-leftPos_.y,-Config::MV_X_SPAC+z,leftPos_.x,
    0, 0, 0
  };
  leftFoot.setTargetPose(leftPose);

  auto rightPos_ = tread(h,Wd*-cmd.move.y,DutyX,DutyY,T,phaseShift(motionTime,(long)(T*500))*0.001);
  FootController::Pose rightPos={
    420-rightPos_.y,+Config::MV_X_SPAC+z,rightPos_.x,
    0, 0, 0
  };
  rightFoot.setTargetPose(rightPos);

  if((long)(T*1000) > motionTime){
    return false;
  }else{
    return true;
  }
}

//左右方向移動
bool MV_Y_F(long motionTime,GaitParameters GaitParameters_){
  leftFoot.setJointSkip(true);
  rightFoot.setJointSkip(true);

  #ifdef DEBUG
  Serial.printf("MV_Y_F\n");
  #endif

  float T = GaitParameters_.T;
  float h = GaitParameters_.h;
  float Wd = GaitParameters_.Wd;
  float DutyX = GaitParameters_.DutyX;
  float DutyY = GaitParameters_.DutyY;

  ControllerApp::Commands cmd = OpeCom.getCommands();


  auto leftPos_ = tread(h,Wd*-cmd.move.x,DutyX,DutyY,T,motionTime*0.001);
  FootController::Pose leftPose={
    420-leftPos_.y,leftPos_.x-Config::MV_Y_SPAC,0,
    0, 0, 0
  };
  leftFoot.setTargetPose(leftPose);

  auto rightPos_ = tread(h,Wd*-cmd.move.x,DutyX,DutyY,T,phaseShift(motionTime,(long)(T*500))*0.001);
  FootController::Pose rightPos={
    420-rightPos_.y,rightPos_.x+Config::MV_Y_SPAC,0,
    0, 0, 0
  };
  rightFoot.setTargetPose(rightPos);

  if((long)(T*1000) > motionTime){
    return false;
  }else{
    return true;
  }
}

//平行移動
bool MV_FREE_F(long motionTime,GaitParameters GaitParameters_){
  leftFoot.setJointSkip(true);
  rightFoot.setJointSkip(true);

  #ifdef DEBUG
  Serial.printf("MV_FREE_F\n");
  #endif

  float T = GaitParameters_.T;
  float h = GaitParameters_.h;
  float Wd = GaitParameters_.Wd;
  float DutyX = GaitParameters_.DutyX;
  float DutyY = GaitParameters_.DutyY;

  ControllerApp::Commands cmd = OpeCom.getCommands();

  float footLeftUp = tread_y(h,T,DutyY,motionTime*0.001);
  float Left_x = tread_x(Wd*-cmd.move.x,T,DutyX,motionTime*0.001);
  float Left_z = tread_x(Wd*-cmd.move.y,T,DutyX,motionTime*0.001);
  FootController::Pose leftPose={
    420-footLeftUp,Left_x-Config::MV_FREE_SPAC,Left_z,
    0, 0, 0
  };
  leftFoot.setTargetPose(leftPose);

  float footRightUp = tread_y(h,T,DutyY,phaseShift(motionTime,(long)(T*500))*0.001);
  float Right_x = tread_x(Wd*-cmd.move.x,T,DutyX,phaseShift(motionTime,(long)(T*500))*0.001);
  float Right_z = tread_x(Wd*-cmd.move.y,T,DutyY,phaseShift(motionTime,(long)(T*500))*0.001);
  FootController::Pose rightPos={
    420-footRightUp,Right_x+Config::MV_FREE_SPAC,Right_z,
    0, 0, 0
  };
  rightFoot.setTargetPose(rightPos);

  if((long)(T*1000) > motionTime){
    return false;
  }else{
    return true;
  }
}

//アイドル状態（待機）
void IDLE_F(){
  leftFoot.setJointSkip(false);
  rightFoot.setJointSkip(false);
  
  #ifdef DEBUG
  Serial.printf("IDLE_F\n");
  #endif
  //足曲げ立ち
  FootController::Pose Pose={
    420,-Config::IDLE_SPAC,0,
    0,0,0
  };
  leftFoot.setTargetPose(Pose);
  Pose.Y = +Config::IDLE_SPAC;
  rightFoot.setTargetPose(Pose);
}

//攻撃1
void ACT_ATTACK1_F(){
  Serial.println("[Demo]");
}

//足サーボ零点移動
void ZERO_F(){
  //直立
  leftFoot.setJointSkip(false);
  rightFoot.setJointSkip(false);

  leftFoot.setJointAngles(7500,7500,7500,7500,7500);
  rightFoot.setJointAngles(7500,7500,7500,7500,7500);
  Serial.println("[zero]");
}

/*-------------------------------------*/
//ステート管理
enum RobotState {
  IDLE,
  MV_X,
  MV_Y,
  MV_TURN,
  ACT_ATTACK1,
  ACT_ATTACK2,
  ACT_ATTACK3,
  ACT_ATTACK4,
  SP1,
  SP2,
  ACT_GETUP
};

RobotState robotState = IDLE;
//無操作時間の記録のため
int lastRecvTime=0;
bool stateUpdate(){
  static RobotState robotStateLast;
  if (Dualshock4.isConnected()) {
    OpeCom.update();
    ControllerApp::Commands cmd = OpeCom.getCommands();

    #ifdef DEBUG
    Serial.printf("UnitY:%f UnitX:%f LookX:%f\n robotState:%d\n",cmd.move.y,cmd.move.x, cmd.look,robotState);
    #endif

    if(cmd.isSp1){
      //SP1
      robotState = SP1;
    }else if(cmd.look > 0.5){
      //方向変更
      robotState = MV_TURN;
    }else if(cmd.isSp2){
      //
      robotState = SP2;
    }else if(cmd.isAttack1){
      //攻撃１(零点)
      robotState = ACT_ATTACK1;
    }else if(cmd.isAttack2){
      //攻撃２(自由方向移動)
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
    }else if((abs(cmd.move.x)+abs(cmd.move.y)) > 0.1){
      if(abs(cmd.move.x) < abs(cmd.move.y)){
        robotState = MV_X;
      }else{
        robotState = MV_Y;
      }
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
/*-------------------------------------*/
// モーションの進捗管理用
int motionStep = 0;
long motionTimeOrigin = 0;
void motorTask(void *pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(long(1000.0 / Config::IDEL_FPS)); 

  while(1){
    if (stateUpdate()) {
      motionStep = 0; 
      motionTimeOrigin = millis(); 
    }
    long motionTime = millis() - motionTimeOrigin;


    switch (robotState) {
      case MV_X:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / Config::MV_X_FPS)); 
        if(MV_X_F(motionTime,Config::MV_X_PARAM)){
          motionTimeOrigin = millis(); 
          motionStep = 0; 
        }
        break;

      case MV_Y:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / Config::MV_Y_FPS)); 
        if(MV_Y_F(motionTime,Config::MV_Y_PARAM)){
          motionTimeOrigin = millis(); 
          motionStep = 0; 
        }
        break;

      case SP1:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / Config::MV_FREE_FPS)); 
        if(MV_FREE_F(motionTime,Config::MV_FREE_PARAM)){
          motionTimeOrigin = millis(); 
          motionStep = 0; 
        }
        break;

      case SP2:
        ZERO_F();
        xFrequency = pdMS_TO_TICKS(long(1000.0 / Config::IDEL_FPS));
        break;

      case IDLE:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / Config::IDEL_FPS)); 
        IDLE_F();
        break;

      case ACT_ATTACK1:
        ACT_ATTACK1_F();
        break;

    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
/*-------------------------------------*/
//準備関数（setup）
servoICS::Servo servoDEMOS(Config::ServoSerial,Config::enPin,5);
void setup() {
  Serial.begin(Config::bpsPC);
  if(Config::ServoSerial != &Serial)Config::ServoSerial->begin(Config::bpsServo,SERIAL_8E1,Config::rxPin,Config::txPin);
  //SERIAL_8E1がICS規格で使用されている。


  
  Dualshock4.begin(Config::ControllerMac);
  bondReset();

  Serial.printf("Hello World!\n");
  #ifndef SIMULATION
  leftFoot.setOffset(7726,7466,7378,7429,7576);
  rightFoot.setOffset(7620,7509,7452,7168,7638);
  #endif

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