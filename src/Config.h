#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "FootController.h"



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
    const char ControllerMac[18] = "06:02:01:02:05:10";

    //PCとの通信のボートレート
    const long bpsPC = 115200;

    //サーボとの通信設定
    #ifdef SIMULATION
    HardwareSerial* ServoSerial = &Serial;
    #else
    HardwareSerial* ServoSerial = &Serial1;
    #endif
    const char enPin = 23;
    const char txPin = 19;
    const char rxPin = 22;
    const long bpsServo = 115200;

    const char hipServoID = 11;

    //足寸法
    FootController::leng8 lengs8={18.75,49,20.96,150.04,150.04,20.96,49,18.75};
    FootController::IcsServoConfig rightfootConfig{
    9,
    12,
    13,
    14,
    15,
    enPin,
    ServoSerial
    };

    FootController::IcsServoConfig leftfootConfig{
    10,
    16,
    17,
    18,
    19,
    enPin,
    ServoSerial
    };

    //歩行軌道のパラメータ
    GaitParameters MV_X_PARAM(
    0.7,    //T
    40,     //h
    120,    //Wd
    0.8,    //DutyX
    0.8,     //DutyY
    40,     //Fps
    50.0    //Spac
    );

    GaitParameters MV_FREE_PARAM(
    1.0,    //T
    60,     //h
    100,    //Wd
    0.6,    //DutyX
    0.65,     //DutyY
    30,     //Fps
    20.0    //Spac
    );

    GaitParameters MV_Y_PARAM(
    0.7,    //T
    40,     //h
    120,    //Wd
    0.8,    //DutyX
    0.8,     //DutyY
    40,     //Fps
    50.0    //Spac
    );

    int IDEL_FPS = 10;
    float IDLE_SPAC = 50.0;

    const int MotionFPS = 10;

    char leftFootJ1ID = 10;
    char leftFootJ2ID = 16;
    char leftFootJ3ID = 17;
    char leftFootJ4ID = 18;
    char leftFootJ5ID = 19;

    char rightFootJ1ID = 9;
    char rightFootJ2ID = 12;
    char rightFootJ3ID = 13;
    char rightFootJ4ID = 14;
    char rightFootJ5ID = 15;

    char leftArmJ1ID = 1;
    char leftArmJ2ID = 3;
    char leftArmJ3ID = 4;
    char leftArmJ4ID = 5;

    char rightArmJ1ID = 2;
    char rightArmJ2ID = 6;
    char rightArmJ3ID = 7;
    char rightArmJ4ID = 8;

    const int TAUNT_FPS = 30;
    const float TAUNT_SPAC = 50.0;

}

#endif