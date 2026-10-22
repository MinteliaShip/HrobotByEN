#include "Config.h"

/**********************************************/
// 基本設定の実体定義
const int serialPC_bps = 115200;
const int serialServo_bps = 115200;
const int txPin = 19;
const int rxPin = 22;
const int enPin = 23;
/**********************************************/

const char stretch = 120;
int G_fps = 40;

float NeutralPoseY = 250;
float NeutralPoseX = -0;
float NeutralSpac = 120;
//左右の足の差をここで吸収させる。Def = 差
float footPoseDefLeft=0;
float footPoseDefRight=20;

// グローバル変数の実体定義
bool g_isFaceUp = true;
float g_lpfAcc = 1.0f;

// コントローラのMACアドレス
const char Config::ControllerMac[18] = "06:02:01:02:05:10";

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

// 足寸法
float Config::lengs8[8] = {9.4, 29.219, 25.4, 120, 120.0, 25.4, 25, 11.5};

GaitParameters Config::MV_X_PARAM{
    0.7,         // T
    50,          // h
    150,         // Wd
    0.8,         // DutyX
    0.73,        // DutyY
    G_fps,       // Fps
    NeutralSpac, // Spac
    NeutralPoseY + 3.0f, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + 0.0f, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -6.0f,       // kickX_left
    -5.0f,       // kickY_left
    -6.0f,       // kickX_right
    -5.0f,       // kickY_right
    0.3f         // kickTime
};

GaitParameters Config::MV_X_PARAM_1Y{
    1.0,         // T
    40,          // h
    -40,         // Wd
    0.90,        // DutyX
    0.85,        // DutyY
    G_fps,       // Fps
    NeutralSpac, // Spac
    NeutralPoseY + footPoseDefLeft, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + footPoseDefRight, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -8.0f,       // kickX_left
    -10.0f,      // kickY_left
    -8.0f,       // kickX_right
    -10.0f,      // kickY_right
    0.15f        // kickTime
};

//double

GaitParameters Config::MV_Y_DOUBLE{
    0.3,         // T
    100,          // h
    -400,         // Wd
    0.90,        // DutyX
    0.85,        // DutyY
    G_fps,       // Fps
    NeutralSpac-40, // Spac
    NeutralPoseY + footPoseDefLeft, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + footPoseDefRight, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    0.0f,      // kickX_left
    -10.0f,      // kickY_left
    0.0f,      // kickX_right
    -10.0f,      // kickY_right
    0.2f         // kickTime
};

GaitParameters Config::MV_Y_SINGLE{//そこまで早くない
    0.3,         // T
    80,          // h
    -400,         // Wd
    0.90,        // DutyX
    0.85,        // DutyY
    G_fps,       // Fps
    NeutralSpac-40, // Spac
    NeutralPoseY + footPoseDefLeft, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + footPoseDefRight, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    0.0f,      // kickX_left
    -10.0f,      // kickY_left
    0.0f,      // kickX_right
    -10.0f,      // kickY_right
    0.1f         // kickTime
};

GaitParameters Config::MV_X_PARAM_TURN{
    0.3,         // T
    60,          // h
    180,         // Wd
    0.95,        // DutyX
    0.90,        // DutyY
    G_fps,       // Fps
    NeutralSpac, // Spac
    NeutralPoseY + 3.0f, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + 0.0f, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -3.0f,       // kickX_left
    -9.0f,       // kickY_left
    -0.0f,       // kickX_right
    -8.0f,       // kickY_right
    0.15f        // kickTime
};

GaitParameters Config::MV_X_PARAM_2{
    0.9,         // T
    52,          // h
    70,          // Wd
    0.95,        // DutyX
    0.90,        // DutyY
    G_fps,       // Fps
    NeutralSpac, // Spac
    NeutralPoseY + 3.0f, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + 0.0f, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -0.0f,       // kickX_left
    -0.7f,       // kickY_left
    -0.0f,       // kickX_right
    -0.7f,       // kickY_right
    0.4f         // kickTime
};

GaitParameters Config::MV_X_PARAM_3{
    1.0,         // T
    30,          // h
    50,          // Wd
    0.90,        // DutyX
    0.88,        // DutyY
    G_fps,       // Fps
    NeutralSpac, // Spac
    NeutralPoseY + 3.0f, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + 0.0f, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -0.0f,       // kickX_left
    -10.0f,      // kickY_left
    -0.0f,       // kickX_right
    -10.0f,      // kickY_right
    0.15f        // kickTime
};

GaitParameters Config::MV_X_PARAM_BACK{
    1.0,         // T
    30,          // h
    -50,         // Wd
    0.90,        // DutyX
    0.88,        // DutyY
    G_fps,       // Fps
    NeutralSpac, // Spac
    NeutralPoseY + 3.0f, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + 0.0f, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -0.0f,       // kickX_left
    -10.0f,      // kickY_left
    -0.0f,       // kickX_right
    -10.0f,      // kickY_right
    0.15f        // kickTime
};

GaitParameters Config::MV_X_PARAM_TEST{
    2.0,         // T
    50,          // h
    100,         // Wd
    0.8,         // DutyX
    0.8,         // DutyY
    G_fps,       // Fps
    NeutralSpac, // Spac
    NeutralPoseY + 3.0f, // offsetZ_left
    NeutralPoseX,        // offsetX_left
    NeutralPoseY + 0.0f, // offsetZ_right
    NeutralPoseX,        // offsetX_right
    -0.0f,       // kickX_left
    -4.0f,       // kickY_left
    -0.0f,       // kickX_right
    -4.0f,       // kickY_right
    0.3f         // kickTime
};

int Config::IDEL_FPS = 10;
float Config::IDLE_SPAC = 50.0;

const int Config::MotionFPS = 10;

const char Config::leftFootJ1ID = 10;
const char Config::leftFootJ2ID = 16;
const char Config::leftFootJ3ID = 17;
const char Config::leftFootJ4ID = 18;
const char Config::leftFootJ5ID = 19;

const char Config::rightFootJ1ID = 9;
const char Config::rightFootJ2ID = 12;
const char Config::rightFootJ3ID = 14;
const char Config::rightFootJ4ID = 13;
const char Config::rightFootJ5ID = 15;

const char Config::leftArmJ1ID = 1;
const char Config::leftArmJ2ID = 3;
const char Config::leftArmJ3ID = 4;
const char Config::leftArmJ4ID = 5;

const char Config::rightArmJ1ID = 2;
const char Config::rightArmJ2ID = 6;
const char Config::rightArmJ3ID = 7;
const char Config::rightArmJ4ID = 8;

const int Config::TAUNT_FPS = 30;
const float Config::TAUNT_SPAC = 50.0;