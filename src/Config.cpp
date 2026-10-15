#include "Config.h"

/**********************************************/
// 基本設定の実体定義
const int serialPC_bps = 115200;
const int serialServo_bps = 115200;
const int txPin = 19;
const int rxPin = 22;
const int enPin = 23;
/**********************************************/

const int stretch = 105;
int G_fps = 40;

float NeutralPoseY = 110;
float NeutralPoseX = -20;
float NeutralSpac = 100;

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
    0.7, 50, 150, 0.8, 0.73, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -6.0f, -5.0f, -6.0f, -5.0f, 0.3f
};

GaitParameters Config::MV_X_PARAM_1Y{
    1.0, 40, -40, 0.90, 0.85, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -8.0f, -10.0f, -8.0f, -10.0f, 0.15f
};

GaitParameters Config::MV_X_PARAM_FAST_Y{
    0.3, 25, -80, 0.90, 0.85, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -15.0f, -10.0f, -15.0f, -10.0f, 0.1f
};

GaitParameters Config::MV_X_PARAM_TURN{
    0.3, 60, 180, 0.95, 0.90, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -3.0f, -9.0f, -0.0f, -8.0f, 0.15f
};

GaitParameters Config::MV_X_PARAM_2{
    0.9, 52, 70, 0.95, 0.90, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -0.0f, -0.7f, -0.0f, -0.7f, 0.4f
};

GaitParameters Config::MV_X_PARAM_3{
    1.0, 30, 50, 0.90, 0.88, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -0.0f, -10.0f, -0.0f, -10.0f, 0.15f
};

GaitParameters Config::MV_X_PARAM_BACK{
    1.0, 30, -50, 0.90, 0.88, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -0.0f, -10.0f, -0.0f, -10.0f, 0.15f
};

GaitParameters Config::MV_X_PARAM_TEST{
    2.0, 50, 100, 0.8, 0.8, G_fps, NeutralSpac,
    NeutralPoseY + 3.0f, NeutralPoseX, NeutralPoseY + 0.0f, NeutralPoseX,
    -0.0f, -4.0f, -0.0f, -4.0f, 0.3f
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