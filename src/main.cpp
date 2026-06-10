#include <Arduino.h>
#include <servoICS.h>
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>

#include "controller_to_command.h"
#include "FootController.h"
#include "Config.h"
#include "ConfigDef.h"
#include "Vector.h"
#include "Motion.h"
#include "frameData.h"

/*宣言・初期化・定数*/
PS4Controller Dualshock4;
ControllerApp::CommandConverter OpeCom(&Dualshock4.data,Config::mapping);
FootController leftFoot(Config::leftfootConfig,Config::lengs8);
FootController rightFoot(Config::rightfootConfig,Config::lengs8);
servoICS::Servo hipServo(Config::ServoSerial,Config::enPin,Config::hipServoID);

servoICS::Servo leftArmJ1(Config::ServoSerial,Config::enPin,Config::leftArmJ1ID);
servoICS::Servo leftArmJ2(Config::ServoSerial,Config::enPin,Config::leftArmJ2ID);
servoICS::Servo leftArmJ3(Config::ServoSerial,Config::enPin,Config::leftArmJ3ID);
servoICS::Servo leftArmJ4(Config::ServoSerial,Config::enPin,Config::leftArmJ4ID);
servoICS::Servo *leftArm[4]{
  &leftArmJ1,
  &leftArmJ2,
  &leftArmJ3,
  &leftArmJ4
};

servoICS::Servo rightArmJ1(Config::ServoSerial,Config::enPin,Config::rightArmJ1ID);
servoICS::Servo rightArmJ2(Config::ServoSerial,Config::enPin,Config::rightArmJ2ID);
servoICS::Servo rightArmJ3(Config::ServoSerial,Config::enPin,Config::rightArmJ3ID);
servoICS::Servo rightArmJ4(Config::ServoSerial,Config::enPin,Config::rightArmJ4ID);
servoICS::Servo *rightArm[4]{
  &rightArmJ1,
  &rightArmJ2,
  &rightArmJ3,
  &rightArmJ4
};

servoICS::Servo *arm[8]{
  &leftArmJ1,
  &leftArmJ2,
  &leftArmJ3,
  &leftArmJ4,
  &rightArmJ1,
  &rightArmJ2,
  &rightArmJ3,
  &rightArmJ4
};
Motion::MotionController MotionDemo(motionCaptured,500,arm);


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
  SP3,
  SP4,
  ACT_GETUP,
  TAUNT
};
RobotState robotState = IDLE;

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

void ACT_ATTACK1_F(){
  Serial.printf("腕薙ぎ払いモーション!\n");
  delay(2000);
  Serial.printf("[end]\n");
}


void ACT_ATTACK3_F(){
  Serial.printf("パンチモーション!\n");
  delay(2000);
  Serial.printf("[end]\n");
}

void ArmIni(){
  //初期位置
  leftArmJ1.setPosDeg(0);
  leftArmJ2.setPosDeg(+70);
  leftArmJ3.setPosDeg(0);
  leftArmJ4.setPosDeg(0);//-方向が手前。↓も同じ

  rightArmJ1.setPosDeg(0);
  rightArmJ2.setPosDeg(+70);
  rightArmJ3.setPosDeg(0);
  rightArmJ4.setPosDeg(0);

  hipServo.setMSMaxDeg(0);
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
  FootController::Pose Pose={//420
    420,-Config::IDLE_SPAC,0,
    0,0,0
  };

  ArmIni();//初期位置

  leftFoot.setTargetPose(Pose);
  Pose.Y = +Config::IDLE_SPAC;
  rightFoot.setTargetPose(Pose);
  vTaskDelayUntil(&lastTimeTicks, cycleTimeTicks);

  //100以上になったら、通信をやめる。
  static int count = 0;
  if(count > 100)while(1){}
  count++;

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

    if(cmd.isTaunt){//TARNT
      robotState = TAUNT;
    }else if(cmd.isSp1){//SP1
      //robotState = MV_FREE;
    }else if(cmd.isSp2){//SP2
      //robotState = SP2;
    }else if(cmd.isSp3){//SP3
      //robotState = SP3;
    }else if(cmd.isSp4){//SP4
      //robotState = SP4;
    }else if(cmd.isAttack1){//ACT1
      robotState = ACT_ATTACK1;
    }else if(cmd.isAttack2){//ACT2
      //robotState = ACT_ATTACK2;
    }else if(cmd.isAttack3){//ACT3
      robotState = ACT_ATTACK3;
    }else if(cmd.isAttack4){//ACT4
      //robotState = ACT_ATTACK4;
    }else if(cmd.isGetup){//get up
      //robotState = ACT_GETUP;
    }

    //以降はスティック系
    else if(cmd.look > 0.5){//turn
      //方向変更
      //robotState = MV_TURN;
    }else if((abs(cmd.move.x)+abs(cmd.move.y)) > 0.1){
      if(abs(cmd.move.x) < abs(cmd.move.y)){
        //robotState = MV_X;
      }else{
        //robotState = MV_Y;
      }
    }else{
      //IDLE
      robotState = TAUNT;
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
void motorTask(void *pvParameters) {

  while(1){
    switch (robotState) {
      case MV_X:
        //MV_X_F();
        break;
      
      case MV_Y:
        //MV_Y_F();
        break;

      case MV_FREE:
        //MV_FREE_F();
        break;

      case TAUNT:
        //TAUNT_F();
        break;

      case SP3:
        //readArm();
        break;

      case SP2:
        //ZERO_F();
        break;
      
      case ACT_ATTACK1:
        ACT_ATTACK1_F();
        break;

      case ACT_ATTACK2:
        //ACT_ATTACK2_F();
        break;

      case ACT_ATTACK3:
        ACT_ATTACK3_F();
        break;

      case IDLE:
        IDLE_F();
        break;
    }
    delay(1);
  }
}



/*-------------------------------------*/
//準備関数（setup）
servoICS::Servo servoDEMOS(Config::ServoSerial,Config::enPin,5);
void setup() {

  delay(1000);

  Serial.begin(Config::bpsPC);
  if(Config::ServoSerial != &Serial)Config::ServoSerial->begin(Config::bpsServo,SERIAL_8E1,Config::rxPin,Config::txPin);
  //SERIAL_8E1がICS規格で使用されている。
  
  Dualshock4.begin(Config::ControllerMac);
  bondReset();

  #ifndef SIMULATION
  leftFoot.setOffset(7726,7466,7378,7429,7576);
  rightFoot.setOffset(7620,7509,7452,7168,7638);
  #endif

  xTaskCreateUniversal(
    motorTask,      // 関数名
    "motorTask",    // タスク名
    8192,           // スタックサイズ
    NULL,           // パラメータ
    configMAX_PRIORITIES - 1,              // 優先度
    NULL,           // タスクハンドル
    1               // 実行するコア (0 or 1)
  );

}


void loop() {
  stateUpdate();
  delay(100);
}