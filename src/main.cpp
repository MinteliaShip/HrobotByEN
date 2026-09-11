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

void taskManager();

void loop2(void *p);

void loop2_begin(){
    xTaskCreatePinnedToCore(
        loop2,           // タスク関数名
        "Loop2Task",     // タスク名（デバッグ用文字列）
        4096,            // スタックサイズ（バイト単位。必要に応じて調整）
        NULL,            // タスク引数
        1,               // 優先度（1〜24、数値が大きいほど優先）
        NULL,            // タスクハンドル
        0                // 割り当てるコア番号（0 または 1）
    );
}

const uint32_t RIGHT_BIT     = (1UL << 0);
const uint32_t DOWN_BIT      = (1UL << 1);
const uint32_t UP_BIT        = (1UL << 2);
const uint32_t LEFT_BIT      = (1UL << 3);

const uint32_t SQUARE_BIT    = (1UL << 4);
const uint32_t CROSS_BIT     = (1UL << 5);
const uint32_t CIRCLE_BIT    = (1UL << 6);
const uint32_t TRIANGLE_BIT  = (1UL << 7);

const uint32_t UPRIGHT_BIT   = (1UL << 8);
const uint32_t DOWNRIGHT_BIT = (1UL << 9);
const uint32_t UPLEFT_BIT    = (1UL << 10);
const uint32_t DOWNLEFT_BIT  = (1UL << 11);

const uint32_t L1_BIT        = (1UL << 12);
const uint32_t R1_BIT        = (1UL << 13);
const uint32_t L2_BIT        = (1UL << 14);
const uint32_t R2_BIT        = (1UL << 15);

const uint32_t SHARE_BIT     = (1UL << 16);
const uint32_t OPTIONS_BIT   = (1UL << 17);
const uint32_t L3_BIT        = (1UL << 18);
const uint32_t R3_BIT        = (1UL << 19);

const uint32_t PS_BIT        = (1UL << 20);
const uint32_t TOUCHPAD_BIT  = (1UL << 21);

namespace activeMotion{//アクティブなモーションはtrueに。
    namespace walk{
        bool walk1;
        bool walkY;
        bool turn;
    }

    namespace posture{
        namespace battle{
            bool attack_Light_left;
            bool attack_Light_right;
            bool attack_Medium_left;
            bool attack_Medium_right;
            bool attack_Heavy_1;
            bool attack_Heavy_2;
        }

        bool taunt;
        bool nop;
        bool LOCK_DebugMode;
        bool pose;
        bool chair;   //椅子に座る
        bool kneeling;   //膝立ち
        bool getUp;

        bool hip;//腰回転
    }
}

char busyPartsBit;
//コントローラ判定
//0 hip
//1 leftArm
//2 rightArm
//3 leftFoot
//4 rightFoot
//5 Non
//6 Non
//7 Non

const char HIP_BIT        = 0b00000001;
const char LEFT_ARM_BIT   = 0b00000010;
const char RIGHT_ARM_BIT  = 0b00000100;
const char LEFT_FOOT_BIT  = 0b00001000;
const char RIGHT_FOOT_BIT = 0b00010000;

bool runExclusiveTask(bool startFlag,char targetBit, bool &activeFlag, bool (*motionFunc)()) {
    if (activeFlag) {
        // 実行中の場合：モーションを継続し、終了したら解放
        if (!motionFunc()) {
            activeFlag = false;
            busyPartsBit &= ~targetBit;
        }
        return true;
    } else if (startFlag && ((busyPartsBit & targetBit) == 0)) {
        // 停止中で、リソースが空いている場合：起動条件を満たしていれば開始
        // ※必要に応じて外部の起動トリガー条件を引数に追加可能
        activeFlag = true;
        busyPartsBit |= targetBit;
        
        // 初回実行
        if (!motionFunc()) {
            activeFlag = false;
            busyPartsBit &= ~targetBit;
        }
        return true;
    }
    return false; // 他のタスクが占有中のため実行不可
}

void taskManager(){//タスク管理。
    bool active=0;

    uint32_t button_bits = 0;
    memcpy(&button_bits, &Dualshock4.data.button, sizeof(Dualshock4.data.button));
    ps4_button_t &ps4Button = Dualshock4.data.button;
    Serial.println(busyPartsBit, BIN);

    //起き上がり
    runExclusiveTask(button_bits==TOUCHPAD_BIT,HIP_BIT | LEFT_ARM_BIT | RIGHT_ARM_BIT | LEFT_FOOT_BIT | RIGHT_FOOT_BIT,activeMotion::posture::getUp,motion::posture::getUp);

    //腰回転
    runExclusiveTask(true,HIP_BIT,activeMotion::posture::hip,motion::posture::hip);

    //歩行モーション
    int stick_ly = map_controller(Dualshock4.data.analog.stick.ly,20,-128,127,-10,10);

    //通常歩行
    runExclusiveTask((stick_ly > 0) && !(ps4Button.l3),LEFT_FOOT_BIT | RIGHT_FOOT_BIT,activeMotion::walk::walk1,motion::walk::walk1);

    //横歩行
    runExclusiveTask(ps4Button.right || ps4Button.left || ps4Button.up || ps4Button.down ,LEFT_FOOT_BIT | RIGHT_FOOT_BIT,activeMotion::walk::walkY,motion::walk::walkY);

    //回転
    runExclusiveTask(button_bits == L3_BIT,LEFT_FOOT_BIT | RIGHT_FOOT_BIT,activeMotion::walk::turn,motion::walk::turn);

    //単押しの攻撃モーション
    active += runExclusiveTask(button_bits == L1_BIT || (button_bits == (L1_BIT | R1_BIT)) || (button_bits == (L1_BIT | R2_BIT)),LEFT_ARM_BIT,activeMotion::posture::battle::attack_Light_left,motion::posture::battle::attack_Light_left);
    active += runExclusiveTask(button_bits == L2_BIT || (button_bits == (L2_BIT | R1_BIT)) || (button_bits == (L2_BIT | R2_BIT)),LEFT_ARM_BIT,activeMotion::posture::battle::attack_Medium_left,motion::posture::battle::attack_Medium_left);

    active += runExclusiveTask(button_bits == R1_BIT || (button_bits == (L1_BIT | R1_BIT))|| (button_bits == (L2_BIT | R1_BIT)),RIGHT_ARM_BIT,activeMotion::posture::battle::attack_Light_right,motion::posture::battle::attack_Light_right);
    active += runExclusiveTask(button_bits == R2_BIT || (button_bits == (L1_BIT | R2_BIT))|| (button_bits == (L2_BIT | R2_BIT)),RIGHT_ARM_BIT,activeMotion::posture::battle::attack_Medium_right,motion::posture::battle::attack_Medium_right);

    //攻撃モーションがない時に実行される。実質LEDを白色に戻す担当者
    runExclusiveTask(!active,0,activeMotion::posture::nop,motion::posture::nop);

    //姿勢を正す。 強制移行可能
    runExclusiveTask(button_bits == SHARE_BIT,0,activeMotion::posture::pose,motion::posture::pose);
    runExclusiveTask(button_bits == OPTIONS_BIT,0,activeMotion::posture::taunt,motion::posture::taunt);
    //デバッグモード 強制移行可能
    runExclusiveTask(button_bits == PS_BIT,0,activeMotion::posture::LOCK_DebugMode,motion::posture::LOCK_DebugMode);
}

FrameLimiter framelim;

void setup() {
    Serial.begin(serialPC_bps);
    Serial1.begin(serialServo_bps,SERIAL_8E1,rxPin,txPin);

    loop2_begin();
    delay(500);

    Dualshock4.begin(Config::ControllerMac);
    bondReset();

    while(1){
        if(Dualshock4.isConnected()){
            Dualshock4.setLed(255, 255, 255);
            Dualshock4.setRumble(0, 255);
            Dualshock4.sendToController();
            resetRumble(500);   //500ms後に停止。
            break;
        }
    }
    // 設定の送信
    Dualshock4.sendToController();
    Serial.println("Connected");

    LittleFS_ini();//初期化
    listFiles();//保存データを一覧表示

    #ifndef SIMULATION
        const float offsetDeg[10]={1.79,3.98,-2.13,-2.09,4.83,14.48,3.21,0.00,-7.56,4.96};
        for(int i=0;i<10;i++){
            ServoArray[i+9]->setOffsetDeg(offsetDeg[i]);
            delay(5);
        }
        for(int i=0;i<19;i++){
            ServoArray[i]->setStretch(stretch);
            delay(5);
        }
        ServoArray[13]->setStretch(1);
        delay(5);
        ServoArray[18]->setStretch(1);
        delay(5);
        ServoArray[0]->setSkip(true);
    #endif

    Dualshock4.setLed(255, 0, 0);
    Dualshock4.sendToController();

    framelim.setInterval(1000 / 40);


}

void loop() {
    taskManager();
    framelim.sync();
}

namespace loop2_task
{
    // resetRumble
    volatile unsigned long resetRumble_time=0;
    volatile unsigned long startTime = 0;
    volatile bool resetRumble_flag=false;
}

void resetRumble(long time_ms) {
    loop2_task::startTime = millis();
    loop2_task::resetRumble_time = time_ms;
    loop2_task::resetRumble_flag = true;
}

void loop2(void *p){
    while (true) {
        //[resetRumble]タスク
        // フラグが立っており、指定された時間が経過したか判定
        if (loop2_task::resetRumble_flag) {
            if (millis() - loop2_task::startTime >= loop2_task::resetRumble_time) {
                // 1. フラグを降ろす（2重実行防止）
                loop2_task::resetRumble_flag = false;

                // 2. コントローラーの振動をリセットして送信
                Dualshock4.setRumble(0, 0);
                Dualshock4.sendToController();
            }
        }

        delay(1);
    }
}