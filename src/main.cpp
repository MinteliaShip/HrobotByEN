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
//ステート管理
enum RobotState {
  IDLE,
  MV_X,
  MV_Y,
  MV_FREE,
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

float phaseShift_f(float inStep,float phaseShift){
  float result = 0;
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
void MV_X_F(GaitParameters Para = Config::MV_X_PARAM){
  #ifdef DEBUG
  Serial.printf("MV_X_F\n");
  #endif

  leftFoot.setJointSkip(true);
  rightFoot.setJointSkip(true);

  TickType_t lastTimeTicks = xTaskGetTickCount();
  TickType_t beginTimeTicks = lastTimeTicks;
  TickType_t cycleTimeTicks = pdMS_TO_TICKS(long(1000.0 / Para.Fps)); 
  TickType_t actionTimeTicks = pdMS_TO_TICKS(long(Para.T*1000));

  while(lastTimeTicks-beginTimeTicks <= actionTimeTicks){
    ControllerApp::Commands cmd = OpeCom.getCommands();
    float t = pdTICKS_TO_MS(lastTimeTicks-beginTimeTicks)*0.001;

    #ifdef DEBUG
    Serial.printf("MV_X_F:T[%fs]\n",t);
    #endif

    Vector2 leftPosXY = leftFoot.tread(Para.h,Para.Wd*-cmd.move.y,Para.DutyX,Para.DutyY,Para.T,t);
    Vector2 rightPosXY = rightFoot.tread(Para.h,Para.Wd*-cmd.move.y,Para.DutyX,Para.DutyY,Para.T,phaseShift_f(t,Para.T/2.0));

    FootController::Pose leftPos={
      420-leftPosXY.y,-Para.Spac,leftPosXY.x,
      0, 0, 0
    };
    FootController::Pose rightPos={
      420-rightPosXY.y,+Para.Spac,rightPosXY.x,
      0, 0, 0
    };
    leftFoot.setTargetPose(leftPos);
    rightFoot.setTargetPose(rightPos);

    if(robotState != MV_X)break;
    vTaskDelayUntil(&lastTimeTicks, cycleTimeTicks);
  }
}

//横移動
void MV_Y_F(GaitParameters Para = Config::MV_Y_PARAM){
  #ifdef DEBUG
  Serial.printf("MV_Y_F\n");
  #endif

  leftFoot.setJointSkip(true);
  rightFoot.setJointSkip(true);

  TickType_t lastTimeTicks = xTaskGetTickCount();
  TickType_t beginTimeTicks = lastTimeTicks;
  TickType_t cycleTimeTicks = pdMS_TO_TICKS(long(1000.0 / Para.Fps)); 
  TickType_t actionTimeTicks = pdMS_TO_TICKS(long(Para.T*1000));

  while(lastTimeTicks-beginTimeTicks <= actionTimeTicks){
    ControllerApp::Commands cmd = OpeCom.getCommands();
    float t = pdTICKS_TO_MS(lastTimeTicks-beginTimeTicks)*0.001;

    #ifdef DEBUG
    Serial.printf("MV_Y_F:T[%fs]\n",t);
    #endif

    Vector2 leftPosXY = leftFoot.tread(Para.h,Para.Wd*-cmd.move.x,Para.DutyX,Para.DutyY,Para.T,t);
    Vector2 rightPosXY = rightFoot.tread(Para.h,Para.Wd*-cmd.move.x,Para.DutyX,Para.DutyY,Para.T,phaseShift_f(t,Para.T/2.0));

    FootController::Pose leftPos={
      420-leftPosXY.y,leftPosXY.x-Para.Spac,0,
      0, 0, 0
    };
    FootController::Pose rightPos={
      420-rightPosXY.y,rightPosXY.x+Para.Spac,0,
      0, 0, 0
    };
    leftFoot.setTargetPose(leftPos);
    rightFoot.setTargetPose(rightPos);

    if(robotState != MV_Y)break;
    vTaskDelayUntil(&lastTimeTicks, cycleTimeTicks);
  }
}


//平行移動
void MV_FREE_F(GaitParameters Para = Config::MV_Y_PARAM){
  #ifdef DEBUG
  Serial.printf("MV_FREE_F\n");
  #endif

  leftFoot.setJointSkip(true);
  rightFoot.setJointSkip(true);

  TickType_t lastTimeTicks = xTaskGetTickCount();
  TickType_t beginTimeTicks = lastTimeTicks;
  TickType_t cycleTimeTicks = pdMS_TO_TICKS(long(1000.0 / Para.Fps)); 
  TickType_t actionTimeTicks = pdMS_TO_TICKS(long(Para.T*1000));

  while(lastTimeTicks-beginTimeTicks <= actionTimeTicks){
    ControllerApp::Commands cmd = OpeCom.getCommands();
    float t = pdTICKS_TO_MS(lastTimeTicks-beginTimeTicks)*0.001;

    #ifdef DEBUG
    Serial.printf("MV_FREE_F:T[%fs]\n",t);
    #endif
    
    float footLeftUp = leftFoot.tread_y(Para.h,Para.T,Para.DutyY,t);
    float Left_x = leftFoot.tread_x(Para.Wd*-cmd.move.x,Para.T,Para.DutyX,t);
    float Left_z = leftFoot.tread_x(Para.Wd*-cmd.move.y,Para.T,Para.DutyX,t);
    FootController::Pose leftPos={
      420-footLeftUp,Left_x-Para.Spac,Left_z,
      0, 0, 0
    };

    float footRightUp = rightFoot.tread_y(Para.h,Para.T,Para.DutyY,phaseShift_f(t,Para.T/2.0));
    float Right_x = rightFoot.tread_x(Para.Wd*-cmd.move.x,Para.T,Para.DutyX,phaseShift_f(t,Para.T/2.0));
    float Right_z = rightFoot.tread_x(Para.Wd*-cmd.move.y,Para.T,Para.DutyX,phaseShift_f(t,Para.T/2.0));
    FootController::Pose rightPos={
      420-footRightUp,Right_x+Para.Spac,Right_z,
      0, 0, 0
    };
    
    leftFoot.setTargetPose(leftPos);
    rightFoot.setTargetPose(rightPos);
    
    if(robotState != MV_FREE)break;
    vTaskDelayUntil(&lastTimeTicks, cycleTimeTicks);
  }
}


//アイドル状態（待機）
void IDLE_F(){
  TickType_t lastTimeTicks = xTaskGetTickCount();
  TickType_t cycleTimeTicks = pdMS_TO_TICKS(long(1000.0 / Config::IDEL_FPS)); 
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
  vTaskDelayUntil(&lastTimeTicks, cycleTimeTicks);
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
      robotState = MV_FREE;
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

  while(1){
    #ifdef DEBUG
    static long LastTime=0;
    long NowTime = millis();
    //Serial.printf("T%d\n",NowTime-LastTime);
    LastTime = NowTime;
    #endif

    switch (robotState) {
      case MV_X:
        MV_X_F();
        break;

      
      case MV_Y:
        MV_Y_F();
        break;

      case MV_FREE:
        MV_FREE_F();
        break;

      case SP2:
        ZERO_F();
        break;
      
      case ACT_ATTACK1:
        ACT_ATTACK1_F();
        break;

      case IDLE:
        IDLE_F();
        break;
    }
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
  stateUpdate();
  delay(100);
}