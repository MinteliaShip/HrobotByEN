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
  GaitParameters(float T_, float h_, float Wd_, float DutyX_, float DutyY_) : T(T_), h(h_), Wd(Wd_), DutyX(DutyX_), DutyY(DutyY_){}
};

namespace Config {
    //コントローラのMACアドレス
    const char ControllerMac[18] = "06:02:01:02:05:10";


    //PCとの通信のボートレート
    const long bpsPC = 115200;

    //サーボとの通信設定
    HardwareSerial* ServoSerial = &Serial1;
    const char enPin = 23;
    const char txPin = 19;
    const char rxPin = 22;
    const long bpsServo = 115200;

    //足寸法
    FootController::leng8 lengs8={18.75,49,20.96,150.04,150.04,20.96,49,18.75};
    FootController::IcsServoConfig leftfootConfig{
    9,
    12,
    13,
    14,
    15,
    enPin,
    ServoSerial
    };

    FootController::IcsServoConfig rightfootConfig{
    10,
    16,
    17,
    18,
    19,
    enPin,
    ServoSerial
    };

    GaitParameters MV_X_PARAM(
    0.6,    //T
    20,     //h
    120,    //Wd
    0.6,    //DutyX
    0.8     //DutyY
    );
    int MV_X_FPS = 30; 

    GaitParameters MV_FREE_PARAM(
    0.6,    //T
    20,     //h
    120,    //Wd
    0.6,    //DutyX
    0.8     //DutyY
    );
    int MV_Y_FPS = 30; 

    GaitParameters MV_Y_PARAM(
    0.5,    //T
    5,      //h
    80,     //Wd
    0.8,    //DutyX
    0.8     //DutyY
    );
    int MV_FREE_FPS = 30; 

    int IDEL_FPS = 30; 

    ControllerApp::controllerMapping mapping(
        ControllerApp::BTN_L1,          //ATC1
        ControllerApp::BTN_L2,          //ATC2
        ControllerApp::BTN_R1,          //ATC3
        ControllerApp::BTN_R2,          //ATC4

        ControllerApp::BTN_SHARE,       //SP1
        ControllerApp::BTN_OPTIONS,     //SP2

        ControllerApp::BTN_UP,          //Getup
        ControllerApp::BTN_DOWN,        //Squat
        ControllerApp::BTN_TOUCHPAD,    //Taunt

        ControllerApp::ANALOG_LX,       //MoveX
        ControllerApp::ANALOG_LY,       //MoveY

        ControllerApp::ANALOG_RX        //LookX
    );


}

#endif