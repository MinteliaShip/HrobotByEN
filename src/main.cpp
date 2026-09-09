#include "Declaration.h"
#include "AssistiveProgram.h"   //補助プログラム集

#include "Config.h"
#include "ConfigDef.h"
#include "Vector.h"
#include "Motion.h"
#include "frameData.h"

/*　*/
//デバッグ用関数　コマンド
//毎ループ推奨
//コマンド内容
//・task　：タスク関連
//オプション
//task      ：実行中タスクを表示
//task -s   ：次回タスクの設定
//task -s -n：緊急ですぐに実行するタスクを設定


//(検討)

/*  */
//using NextTaskType = void (*)();
//引数  ：canDelegateTask＝通常タスクの判定をスキップする。しかし、緊急タスクの判定はある。
//返り値：次に実行すべきタスクを帰す。
//
//移行許可ありで判定なし（操作なしや緊急動作なし）の場合、なにもしないnopを帰す。   
//移行許可なしで判定なしの場合は、nullptrを帰す。
NextTaskType taskManager(bool canDelegateTask){//タスク管理。

    //次に実行するタスクを選択する。
    NextTaskType nextTask_ = nullptr;

    //以下に判定内容と関数ポインタの指定。
    do{
        if(canDelegateTask){//処理移行の許可アリ
            nextTask_ = motion::posture::nop;//なにもない場合は、nopになる。

            //歩行モーション
            /*************************/
            int stick_ly = map_controller(Dualshock4.data.analog.stick.ly,20,-128,127,-10,10);
            Serial.printf("stick_lx:%d",stick_ly);
            if(stick_ly > 0){
                nextTask_ = motion::walk::walk1;
                break;
            }


            /*************************/
            //姿勢モーション

            //合わせボタン実行
            if(Dualshock4.data.button.r3){
                if(Dualshock4.data.button.r1){
                    nextTask_ = motion::posture::battle::attack_Heavy_2;
                    break;
                }
            }

            //単体ボタン実行
            if(Dualshock4.data.button.r2){
                nextTask_ = motion::posture::battle::attack_Light_1;
                break;
            }

            if(Dualshock4.data.button.l2){
                nextTask_ = motion::posture::battle::attack_Light_2;
                break;
            }

            if(Dualshock4.data.button.r1){
                nextTask_ = motion::posture::battle::attack_Medium_1;
                break;
            }

            if(Dualshock4.data.button.l1){
                nextTask_ = motion::posture::battle::attack_Medium_2;
                break;
            }

            if(Dualshock4.data.button.options){
                nextTask_ = motion::posture::taunt;
                break;
            }

            if(Dualshock4.data.button.share){
                if(Dualshock4.data.button.square){
                    nextTask_ = motion::posture::pose;  //腕適度な位置
                }else if(Dualshock4.data.button.circle){
                    //nextTask_ = motion::posture::chair; //椅子に座らせたい時
                }else if(Dualshock4.data.button.cross){
                    //nextTask_ = motion::posture::kneeling; //膝立ち
                }

                break;
            }
        }

        if(Dualshock4.data.button.ps){

            nextTask_ = motion::posture::DebugMode;
            while(Dualshock4.data.button.ps==0);
            break;
        }

        
    }while(false);

    //タスクを返す。
    return nextTask_;

}


void setup() {
  Serial.begin(serialPC_bps);
  Serial1.begin(serialServo_bps,SERIAL_8E1,rxPin,txPin);
  
  Dualshock4.begin(Config::ControllerMac);
  bondReset();

  LittleFS_ini();//初期化
  listFiles();//保存データを一覧表示
  

  #ifndef SIMULATION
  const float offsetDeg[10]={1.79,3.98,-2.13,-2.09,4.83,14.48,3.21,0.00,-7.56,4.96};
  for(int i=0;i<10;i++){
    ServoArray[i+9]->setOffsetDeg(offsetDeg[i]);
  }
  #endif

    while(Dualshock4.isConnected()==0){
        delay(1000);
        Serial.println("Connect...");
    }
    Serial.println("Connected");

    nextTask = taskManager(1);
}


void loop() {
    NextTaskType runTask = nextTask;
    runTask();
}