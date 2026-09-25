#ifndef CONFIG_H
#define CONFIG_H

#include "ConfigDef.h"
#include <Arduino.h>
/**********************************************/
//基本設定
extern const int serialPC_bps;
extern const int serialServo_bps;
extern const int txPin;
extern const int rxPin;
extern const int enPin;
/**********************************************/
extern const int stretch;
extern int G_fps;

struct GaitParameters {
  float T;
  float h;
  float Wd;
  float DutyX;
  float DutyY;
  int Fps;
  float Spac;
  float offsetZ_left;
  float offsetX_left;
  float offsetZ_right;
  float offsetX_right;
  float kickX_left;
  float kickY_left;
  float kickX_right;
  float kickY_right;
  float kickTime;
};


namespace Config {
    //コントローラのMACアドレス
    extern const char ControllerMac[18];

    //PCとの通信のボートレート
    extern const long bpsPC;

    //サーボとの通信設定
    extern HardwareSerial* ServoSerial;

    extern const char enPin;
    extern const char txPin;
    extern const char rxPin;
    extern const long bpsServo;

    extern const char hipServoID;

    //足寸法
    extern float lengs8[8];

    //歩行軌道のパラメータ
    extern GaitParameters MV_X_PARAM;
    extern GaitParameters MV_X_PARAM_2;
    extern GaitParameters MV_X_PARAM_3;
    extern GaitParameters MV_X_PARAM_TEST;
    extern GaitParameters MV_X_PARAM_TURN;
    extern GaitParameters MV_X_PARAM_BACK;

    extern GaitParameters MV_X_PARAM_1Y;
    extern GaitParameters MV_X_PARAM_FAST_Y;

    extern GaitParameters MV_FREE_PARAM;

    extern GaitParameters MV_Y_PARAM;

    extern int IDEL_FPS;
    extern float IDLE_SPAC;

    extern const int MotionFPS;

    extern const char leftFootJ1ID;
    extern const char leftFootJ2ID;
    extern const char leftFootJ3ID;
    extern const char leftFootJ4ID;
    extern const char leftFootJ5ID;

    extern const char rightFootJ1ID;
    extern const char rightFootJ2ID;
    extern const char rightFootJ3ID;
    extern const char rightFootJ4ID;
    extern const char rightFootJ5ID;

    extern const char leftArmJ1ID;
    extern const char leftArmJ2ID;
    extern const char leftArmJ3ID;
    extern const char leftArmJ4ID;

    extern const char rightArmJ1ID;
    extern const char rightArmJ2ID;
    extern const char rightArmJ3ID;
    extern const char rightArmJ4ID;

    extern const int TAUNT_FPS;
    extern const float TAUNT_SPAC;

}

#endif