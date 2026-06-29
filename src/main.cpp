#include <Arduino.h>
#include <servoICS.h>
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>

//#include "controller_to_command.h"
//#include "FootController.h"
#include <PS4Controller.h>
#include "Config.h"
#include "ConfigDef.h"
#include "Vector.h"
#include "Motion.h"
#include "frameData.h"


/*宣言・初期化・定数*/
PS4Controller Dualshock4;
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

enum RobotButtonBit : uint32_t {
    B_RIGHT     = 1 << 0,   // right
    B_DOWN      = 1 << 1,   // down
    B_UP        = 1 << 2,   // up
    B_LEFT      = 1 << 3,   // left

    B_SQUARE    = 1 << 4,   // square
    B_CROSS     = 1 << 5,   // cross
    B_CIRCLE    = 1 << 6,   // circle
    B_TRIANGLE  = 1 << 7,   // triangle

    B_UPRIGHT   = 1 << 8,   // upright
    B_DOWNRIGHT = 1 << 9,   // downright
    B_UPLEFT    = 1 << 10,  // upleft
    B_DOWNLEFT  = 1 << 11,  // downleft

    B_L1        = 1 << 12,  // l1
    B_R1        = 1 << 13,  // r1
    B_L2        = 1 << 14,  // l2
    B_R2        = 1 << 15,  // r2

    B_SHARE     = 1 << 16,  // share
    B_OPTIONS   = 1 << 17,  // options
    B_L3        = 1 << 18,  // l3
    B_R3        = 1 << 19,  // r3

    B_PS        = 1 << 20,  // ps
    B_TOUCHPAD  = 1 << 21   // touchpad
};

void State(){
  uint32_t buttonState=0;
  memcpy(&buttonState, &(Dualshock4.data.button), sizeof(Dualshock4.data.button));

  switch (buttonState) {
    /*------------------------*/  
    //

    /*------------------------*/  
    //特殊
    case B_SHARE|B_OPTIONS://特殊
      Serial.printf("B_SHARE|B_OPTIONS\n");
      break;
    case B_PS://特殊
      Serial.printf("B_PS\n");
      break;
    /*------------------------*/  
    case B_L1://攻撃1
      Serial.printf("B_L1\n");
      break;

    case B_L2://攻撃2
      Serial.printf("B_L2\n");
      break;

    case B_R1://攻撃3
      Serial.printf("B_R1\n");
      break;
    case B_R2://攻撃4
      Serial.printf("B_R2\n");
      break;
    /*------------------------*/  
    default:
      Serial.printf("elif\n");
      break;
  }



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
//準備関数（setup）
servoICS::Servo servoDEMOS(Config::ServoSerial,Config::enPin,5);
void setup() {
  Serial.begin(Config::bpsPC);
  if(Config::ServoSerial != &Serial)Config::ServoSerial->begin(Config::bpsServo,SERIAL_8E1,Config::rxPin,Config::txPin);
  //SERIAL_8E1がICS規格で使用されている。
  
  Dualshock4.begin(Config::ControllerMac);
  bondReset();

  #ifndef SIMULATION
  leftFoot.setOffset(7726,7466,7378,7429,7576);
  rightFoot.setOffset(7620,7509,7452,7168,7638);
  #endif

  delay(500);
  // data.button の構造体メモリをそのまま uint32_t にキャストして二進数文字列に変換
  uint32_t buttonRaw = 0;
  memcpy(&buttonRaw, &(Dualshock4.data.button), sizeof(Dualshock4.data.button));

  Serial.printf("%s\n\n", String(buttonRaw, BIN).c_str());


  while(1){
    State();
  }




}


void loop() {
  delay(100);
}