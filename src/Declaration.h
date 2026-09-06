#include <Arduino.h>
#include <servoICS.h>
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>
#include <LittleFS.h>
#include "Config.h"

#ifndef DECL
#define DECL

extern PS4Controller Dualshock4;
using NextTaskType = void (*)();
NextTaskType taskManager(bool canDelegateTask);
extern NextTaskType nextTask;


/*宣言・初期化・定数*/
extern FootController leftFoot;
extern FootController rightFoot;
extern servoICS::Servo hipServo;

extern servoICS::Servo leftFoot1;

extern servoICS::Servo leftArmJ1;
extern servoICS::Servo leftArmJ2;
extern servoICS::Servo leftArmJ3;
extern servoICS::Servo leftArmJ4;

extern servoICS::Servo rightArmJ1;
extern servoICS::Servo rightArmJ2;
extern servoICS::Servo rightArmJ3;
extern servoICS::Servo rightArmJ4;

extern servoICS::Servo* ServoArray[10];
extern const char* ServoArray_name[10];

#endif