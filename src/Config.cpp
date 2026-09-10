#include "Config.h"

/**********************************************/
//基本設定
int serialPC_bps = 115200;
int serialServo_bps = 115200;
int txPin = 19;
int rxPin = 22;
int enPin = 23;
/**********************************************/



//コントローラのMACアドレス
const char Config::ControllerMac[18] = "06:02:01:02:05:10";

//PCとの通信のボートレート
const long Config::bpsPC = 115200;

//サーボとの通信設定
#ifdef SIMULATION
HardwareSerial* Config::ServoSerial = &Serial;
#else
HardwareSerial* Config::ServoSerial = &Serial1;
#endif
const char Config::enPin = 23;
const char Config::txPin = 19;
const char Config::rxPin = 22;
const long Config::bpsServo = 115200;

const char Config::hipServoID = 11;

//足寸法
//float Config::lengs8[8]={18.75,49,20.96,150.04,150.04,20.96,49,18.75};
float Config::lengs8[8]={9.4,29.219,25.4,120,120.0,25.4,25,11.5};
//歩行軌道のパラメータ
GaitParameters Config::MV_X_PARAM{
    0.7,    //T
    50,     //h
    150,    //Wd
    0.8,    //DutyX
    0.73,    //DutyY
    40,     //Fps
    40.0,    //Spac
    323.0f,  // offsetZ_left
    -15.0f,  // offsetX_left
    320.0f,  // offsetZ_right
    -15.0f,  // offsetX_right
    -6.0f,   // kickX_left
    -5.0f,   // kickY_left
    -6.0f,   // kickX_right
    -5.0f,   // kickY_right
    0.3f     // kickTime
};

GaitParameters Config::MV_X_PARAM_2{
    0.9,     // T
    52,      // h
    70,      // Wd
    0.95,    // DutyX
    0.90,    // DutyY
    30,      // Fps
    30,      // Spac
    314.0f,  // offsetZ_left
    -20.0f,  // offsetX_left
    310.0f,  // offsetZ_right
    -20.0f,  // offsetX_right
    -0.0f,   // kickX_left
    -0.7f,   // kickY_left
    -0.0f,   // kickX_right
    -0.7f,   // kickY_right
    0.4f     // kickTime
};


GaitParameters Config::MV_X_PARAM_3{
    1.0,     // T
    50,      // h
    80,      // Wd
    0.95,    // DutyX
    0.85,    // DutyY
    30,      // Fps
    30,      // Spac
    312.0f,  // offsetZ_left
    -20.0f,  // offsetX_left
    310.0f,  // offsetZ_right
    -20.0f,  // offsetX_right
    -0.0f,   // kickX_left
    -1.0f,   // kickY_left
    -0.0f,   // kickX_right
    -1.0f,   // kickY_right
    0.5f     // kickTime
};



GaitParameters Config::MV_X_PARAM_TEST{
    2.0,    //T
    50,     //h
    100,    //Wd
    0.8,    //DutyX
    0.8,    //DutyY
    10,     //Fps
    50.0,    //Spac
    323.0f,  // offsetZ_left
    -15.0f,  // offsetX_left
    320.0f,  // offsetZ_right
    -15.0f,  // offsetX_right
    -0.0f,   // kickX_left
    -4.0f,   // kickY_left
    -0.0f,   // kickX_right
    -4.0f,   // kickY_right
    0.3f     // kickTime
};

int Config::IDEL_FPS = 10;
float Config::IDLE_SPAC = 50.0;

const int Config::MotionFPS = 10;

char Config::leftFootJ1ID = 10;
char Config::leftFootJ2ID = 16;
char Config::leftFootJ3ID = 17;
char Config::leftFootJ4ID = 18;
char Config::leftFootJ5ID = 19;

char Config::rightFootJ1ID = 9;
char Config::rightFootJ2ID = 12;
char Config::rightFootJ3ID = 14;
char Config::rightFootJ4ID = 13;
char Config::rightFootJ5ID = 15;

char Config::leftArmJ1ID = 1;
char Config::leftArmJ2ID = 3;
char Config::leftArmJ3ID = 4;
char Config::leftArmJ4ID = 5;

char Config::rightArmJ1ID = 2;
char Config::rightArmJ2ID = 6;
char Config::rightArmJ3ID = 7;
char Config::rightArmJ4ID = 8;

const int Config::TAUNT_FPS = 30;
const float Config::TAUNT_SPAC = 50.0;
