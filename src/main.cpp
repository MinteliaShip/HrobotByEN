#include "Declaration.h"
#include "AssistiveProgram.h"   //補助プログラム集

#include "Config.h"
#include "ConfigDef.h"
#include "Vector.h"
#include "Motion.h"
#include "frameData.h"


/**********************************************/
//基本設定
int serialPC_bps = 115200;
int serialServo_bps = 115200;
int txPin = 0;
int rxPin = 0;
int enPin = 0;

/**********************************************/

/*宣言・初期化・定数*/
FootController leftFoot(Config::leftfootConfig,Config::lengs8);
FootController rightFoot(Config::rightfootConfig,Config::lengs8);
servoICS::Servo hipServo(&Serial1,enPin,Config::hipServoID);

servoICS::Servo leftFoot1(&Serial1,enPin,Config::leftFootJ1ID);

servoICS::Servo leftArmJ1(&Serial1,enPin,Config::leftArmJ1ID);
servoICS::Servo leftArmJ2(&Serial1,enPin,Config::leftArmJ2ID);
servoICS::Servo leftArmJ3(&Serial1,enPin,Config::leftArmJ3ID);
servoICS::Servo leftArmJ4(&Serial1,enPin,Config::leftArmJ4ID);

servoICS::Servo rightArmJ1(&Serial1,enPin,Config::rightArmJ1ID);
servoICS::Servo rightArmJ2(&Serial1,enPin,Config::rightArmJ2ID);
servoICS::Servo rightArmJ3(&Serial1,enPin,Config::rightArmJ3ID);
servoICS::Servo rightArmJ4(&Serial1,enPin,Config::rightArmJ4ID);

/**/

void taskManager(bool canDelegateTask){//タスク管理。

    //次に実行するタスクを選択する。
    void (*task)()=nullptr;

    do{
        if(canDelegateTask){//処理移行の許可アリ
        task = motion::posture::nop;//なにもない場合は、nopになる。

        //歩行モーション
        /*************************/
        int stick_lx = map_controller(Dualshock4.data.analog.stick.lx,-127,128,20,-10,10);
        if(stick_lx > 0){
            task = motion::walk::walk1;
            break;
        }
        /*************************/
        //姿勢モーション
        if(Dualshock4.data.button.r2){
            task = motion::posture::battle::attack1;
            break;
        }

        }else{//処理移行の許可ナシ 至急実行用
        //起き上がりモーション

        }
    }while(false);

    //タスク実行
    if(task!=nullptr) task();

}


/*-------------------------------------*/
//準備関数（setup）
servoICS::Servo servoDEMOS(&Serial1,enPin,5);
void setup() {
  Serial.begin(serialPC_bps);
  Serial1.begin(serialServo_bps,SERIAL_8E1,rxPin,txPin);
  
  Dualshock4.begin(Config::ControllerMac);
  bondReset();

  #ifndef SIMULATION
  leftFoot.setOffset(7726,7466,7378,7429,7576);
  rightFoot.setOffset(7620,7509,7452,7168,7638);
  #endif

}


void loop() {
  taskManager(1);//移行許可を与えて。
}