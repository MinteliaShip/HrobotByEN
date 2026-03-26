#include <Arduino.h>
#include <servoICS.h>


#include <PS4Controller.h>
#include <esp_gap_bt_api.h>

#include "controller_to_command.h"

#include "FootController.h"

struct Vector2 {
  float x;
  float y;
  Vector2(float _x = 0.0f, float _y = 0.0f) : x(_x), y(_y) {}
};

/*宣言・初期化・定数*/
PS4Controller Dualshock4;
ControllerApp::CommandConverter OpeCom(&Dualshock4.data);
const char ControllerMac[18] = "06:02:01:02:05:10";
const long bpsPC = 115200;
//サーボとの通信設定
HardwareSerial* ServoSerial = &Serial1;
const char enPin = 23;
const char txPin = 19;
const char rxPin = 22;
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
  ServoSerial
};

FootController::IcsServoConfig rightfootConfig{
  10,
  16,
  17,
  18,
  19,
  enPin,
  ServoSerial
};
//足コントロール
FootController leftFoot(leftfootConfig,lengs8);
FootController rightFoot(rightfootConfig,lengs8);
//歩行関数定数
float MV_X_T = 0.6;
float MV_X_fps = 40;
float MV_X_h = 20;
float MV_X_Wd = 120;
float MV_X_DutyX = 0.6;
float MV_X_DutyY = 0.8;

//平行移動
float MV_FREE_T = 0.6;
float MV_FREE_fps = 40;
float MV_FREE_h = 20;
float MV_FREE_Wd = 120;
float MV_FREE_DutyX = 0.6;
float MV_FREE_DutyY = 0.8;

//歩行関数定数
float MV_Y_T = 0.5;
float MV_Y_fps = 40;
float MV_Y_h = 5;
float MV_Y_Wd = 80;
float MV_Y_DutyX = 0.8;
float MV_Y_DutyY = 0.8;

//アイドル時の設定
float IDLE_fps = 5;

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

  Serial.printf("A:%f, AngV:%f, t_1:%f, t_2:%f\n",A,AngV,t_1,t_2);
  Serial.printf("ts_:%f\n",ts_);

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
//2次元ベクトルの正方化
Vector2 Normalized(Vector2 input) {
    // ベクトルの大きさ（長さ）を計算: L = sqrt(x^2 + y^2)
    float magnitude = sqrt(input.x * input.x + input.y * input.y);

    // 0除算を防ぐため、一定以上の入力がある場合のみ計算
    if (magnitude > 0.001f) {
        return Vector2(input.x / magnitude, input.y / magnitude);
    }

    // 入力がデッドゾーン内などで極めて小さい場合はゼロベクトルを返す
    return Vector2(0.0f, 0.0f);
}
/*-------------------------------------*/
//動作関数
//前後方向移動
bool MV_X_F(long motionTime){

  float T = MV_X_T;
  float h = MV_X_h;
  float Wd = MV_X_Wd;
  float DutyX = MV_X_DutyX;
  float DutyY = MV_X_DutyY;

  ControllerApp::Commands cmd = OpeCom.getCommands();

  auto leftPos_ = tread(h,Wd*(-cmd.moveSpeed.y * 0.01),DutyX,DutyY,T,motionTime*0.001);
  FootController::Pose leftPose={
    420-leftPos_.y,20,leftPos_.x,
    0, 0, 0
  };
  leftFoot.setTargetPose(leftPose);

  auto rightPos_ = tread(h,Wd*(-cmd.moveSpeed.y * 0.01),DutyX,DutyY,T,phaseShift(motionTime,(long)(T*500))*0.001);
  FootController::Pose rightPos={
    420-rightPos_.y,-20,rightPos_.x,
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
bool MV_Y_F(long motionTime){
  float T = MV_Y_T;
  float h = MV_Y_h;
  float Wd = MV_Y_Wd;
  float DutyX = MV_Y_DutyX;
  float DutyY = MV_Y_DutyY;

  ControllerApp::Commands cmd = OpeCom.getCommands();

  auto leftPos_ = tread(h,Wd*(-cmd.moveSpeed.x * 0.01),DutyX,DutyY,T,motionTime*0.001);
  FootController::Pose leftPose={
    420-leftPos_.y,leftPos_.x+20,0,
    0, 0, 0
  };
  leftFoot.setTargetPose(leftPose);

  auto rightPos_ = tread(h,Wd*(-cmd.moveSpeed.x * 0.01),DutyX,DutyY,T,phaseShift(motionTime,(long)(T*500))*0.001);
  FootController::Pose rightPos={
    420-rightPos_.y,rightPos_.x-20,0,
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
bool MV_FREE_F(long motionTime){
  float T = MV_X_T;
  float h = MV_X_h;
  float Wd = MV_X_Wd;
  float DutyX = MV_X_DutyX;
  float DutyY = MV_X_DutyY;

  ControllerApp::Commands cmd = OpeCom.getCommands();

  auto moveAngle = Normalized((Vector2){cmd.moveSpeed.y,cmd.moveSpeed.x});

  float footLeftUp = tread_y(h,T,DutyY,motionTime*0.001);
  float Left_x = tread_x(Wd*-moveAngle.x,T,DutyX,motionTime*0.001);
  float Left_z = tread_x(Wd*-moveAngle.y,T,DutyX,motionTime*0.001);
  FootController::Pose leftPose={
    420-footLeftUp,Left_z+20,Left_x,
    0, 0, 0
  };
  leftFoot.setTargetPose(leftPose);

  float footRightUp = tread_y(h,T,DutyY,phaseShift(motionTime,(long)(T*500))*0.001);
  float Right_x = tread_x(Wd*-moveAngle.x,T,DutyX,phaseShift(motionTime,(long)(T*500))*0.001);
  float Right_z = tread_x(Wd*-moveAngle.y,T,DutyX,phaseShift(motionTime,(long)(T*500))*0.001);
  FootController::Pose rightPos={
    420-footRightUp,Right_z-20,Right_x,
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
  //直立
  FootController::Pose Pose={
    420,20,0,
    0,0,0
  };
  leftFoot.setTargetPose(Pose);
  Pose.Y = -20;
  rightFoot.setTargetPose(Pose);
}

//攻撃1
void ACT_ATTACK1_F(){
  Serial.println("[Demo:left]");
  leftFoot.DemoPos();
  //Serial.println("[Demo:right]");
  //rightFoot.DemoPos();
}

//足サーボ零点移動
void ZERO_F(){
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

    if(cmd.isSp1){
      robotState = SP1;
    }else if(cmd.moveSpeed.y != 0){
      //前進後退
      robotState = MV_X;
    }else if(cmd.moveSpeed.x != 0){
      //左右移動
      robotState = MV_Y;
    }else if(cmd.moveAngle != 0){
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
  TickType_t xFrequency = pdMS_TO_TICKS(long(1000.0 / MV_X_fps)); 

  while(1){
    if (stateUpdate()) {
      motionStep = 0; 
      motionTimeOrigin = millis(); 
    }
    long motionTime = millis() - motionTimeOrigin;


    switch (robotState) {
      case MV_X:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / MV_X_fps)); 
        if(MV_X_F(motionTime)){
          motionTimeOrigin = millis(); 
          motionStep = 0; 
        }
        break;

      case MV_Y:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / MV_X_fps)); 
        if(MV_Y_F(motionTime)){
          motionTimeOrigin = millis(); 
          motionStep = 0; 
        }
        break;

      case SP1:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / MV_X_fps)); 
        if(MV_FREE_F(motionTime)){
          motionTimeOrigin = millis(); 
          motionStep = 0; 
        }
        break;

      case SP2:
        ZERO_F();
        xFrequency = pdMS_TO_TICKS(long(1000.0 / MV_X_fps));
        break;

      case IDLE:
        xFrequency = pdMS_TO_TICKS(long(1000.0 / IDLE_fps)); 
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
servoICS::Servo servoDEMOS(ServoSerial,enPin,5);
void setup() {

  Serial.begin(bpsPC);
  ServoSerial->begin(bpsServo,SERIAL_8E1,rxPin,txPin);  //SERIAL_8E1がICS規格で使用されている。
  
  Dualshock4.begin(ControllerMac);
  bondReset();

  leftFoot.setOffset(7620,7500,7452,7168,7514);
  rightFoot.setOffset(7726,7376,7378,7429,7586);

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