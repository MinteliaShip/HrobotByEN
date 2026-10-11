#include <Arduino.h>
#include <servoICS.h>
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>
#include <LittleFS.h>
#include "Config.h"
#include "ConfigDef.h"
#include "FootController.h"

#ifndef DECL
#define DECL

extern bool g_isFaceUp;
// フィルタ処理後の重力加速度 [g]
extern float g_lpfAcc;

extern PS4Controller Dualshock4;

/*宣言・初期化・定数*/
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

extern servoICS::Servo* ServoArray[19];
extern const char* ServoArray_name[19];

extern const int leftArm_index_num;
extern const int rightArm_index_num;
extern const int leftFoot_index_num;
extern const int rightFoot_index_num;
extern const int arm_index_num;
extern const int foot_index_num;

extern FootController leftFoot;
extern FootController rightFoot;

void resetRumble(long time);

enum BusyPartsBit : char {
    HIP_BIT        = 0b00000001,
    LEFT_ARM_BIT   = 0b00000010,
    RIGHT_ARM_BIT  = 0b00000100,
    LEFT_FOOT_BIT  = 0b00001000,
    RIGHT_FOOT_BIT = 0b00010000,
    FOOT_BIT       = 0b00011000,
    ARM_BIT        = 0b00000110,
    ALL_BIT        = HIP_BIT | FOOT_BIT | ARM_BIT
};

enum ButtonBit : uint32_t {
    RIGHT_BIT    = (1UL << 0),
    DOWN_BIT     = (1UL << 1),
    UP_BIT       = (1UL << 2),
    LEFT_BIT     = (1UL << 3),

    SQUARE_BIT   = (1UL << 4),
    CROSS_BIT    = (1UL << 5),
    CIRCLE_BIT   = (1UL << 6),
    TRIANGLE_BIT = (1UL << 7),

    UPRIGHT_BIT   = (1UL << 8),
    DOWNRIGHT_BIT = (1UL << 9),
    UPLEFT_BIT    = (1UL << 10),
    DOWNLEFT_BIT  = (1UL << 11),

    L1_BIT       = (1UL << 12),
    R1_BIT       = (1UL << 13),
    L2_BIT       = (1UL << 14),
    R2_BIT       = (1UL << 15),

    SHARE_BIT    = (1UL << 16),
    OPTIONS_BIT  = (1UL << 17),
    L3_BIT       = (1UL << 18),
    R3_BIT       = (1UL << 19),

    PS_BIT       = (1UL << 20),
    TOUCHPAD_BIT = (1UL << 21)
};

#endif