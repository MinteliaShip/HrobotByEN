#include <Arduino.h>
#include <servoICS.h>
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>

#ifndef DECL
#define DECL

PS4Controller Dualshock4;
void taskManager(bool canDelegateTask);

#endif