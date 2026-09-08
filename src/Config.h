#ifndef CONFIG_H
#define CONFIG_H

#include "ConfigDef.h"
#include <Arduino.h>
/**********************************************/
//基本設定
extern int serialPC_bps;
extern int serialServo_bps;
extern int txPin;
extern int rxPin;
extern int enPin;
/**********************************************/


struct GaitParameters {
  float T;
  float h;
  float Wd;
  float DutyX;
  float DutyY;
  int Fps;
  float Spac;
  GaitParameters(float T_, float h_, float Wd_, float DutyX_, float DutyY_, int Fps_, float Spac_) : T(T_), h(h_), Wd(Wd_), DutyX(DutyX_), DutyY(DutyY_), Fps(Fps_), Spac(Spac_) {}
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
    extern GaitParameters MV_X_PARAM_TEST;

    extern GaitParameters MV_FREE_PARAM;

    extern GaitParameters MV_Y_PARAM;

    extern int IDEL_FPS;
    extern float IDLE_SPAC;

    extern const int MotionFPS;

    extern char leftFootJ1ID;
    extern char leftFootJ2ID;
    extern char leftFootJ3ID;
    extern char leftFootJ4ID;
    extern char leftFootJ5ID;

    extern char rightFootJ1ID;
    extern char rightFootJ2ID;
    extern char rightFootJ3ID;
    extern char rightFootJ4ID;
    extern char rightFootJ5ID;

    extern char leftArmJ1ID;
    extern char leftArmJ2ID;
    extern char leftArmJ3ID;
    extern char leftArmJ4ID;

    extern char rightArmJ1ID;
    extern char rightArmJ2ID;
    extern char rightArmJ3ID;
    extern char rightArmJ4ID;

    extern const int TAUNT_FPS;
    extern const float TAUNT_SPAC;

}

#endif