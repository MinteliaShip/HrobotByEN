#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "FootController.h"
#include "controller_to_command.h"



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

    char leftArmJ1ID = 1;
    char leftArmJ2ID = 3;
    char leftArmJ3ID = 4;
    char leftArmJ4ID = 5;

    char rightArmJ1ID = 2;
    char rightArmJ2ID = 6;
    char rightArmJ3ID = 7;
    char rightArmJ4ID = 8;

    ControllerApp::controllerMapping mapping(
        ControllerApp::BTN_L1,          //ATC1
        ControllerApp::BTN_L2,          //ATC2
        ControllerApp::BTN_R1,          //ATC3
        ControllerApp::BTN_R2,          //ATC4

        ControllerApp::BTN_SHARE,       //SP1
        ControllerApp::BTN_OPTIONS,     //SP2

        ControllerApp::BTN_UP,          //Getup
        ControllerApp::BTN_DOWN,        //Squat
        ControllerApp::BTN_PS,    //Taunt

        ControllerApp::BTN_TOUCHPAD,

        ControllerApp::ANALOG_LX,       //MoveX
        ControllerApp::ANALOG_LY,       //MoveY

        ControllerApp::ANALOG_RX        //LookX
    );


}

#endif