#include "Declaration.h"

PS4Controller Dualshock4;//実態の定義
NextTaskType nextTask;

FootController leftFoot(Config::leftfootConfig,Config::lengs8);
FootController rightFoot(Config::rightfootConfig,Config::lengs8);
servoICS::Servo hipServo(&Serial1,enPin,Config::hipServoID);

servoICS::Servo leftFoot1(&Serial1,enPin,Config::leftFootJ1ID);

servoICS::Servo leftArmJ1(&Serial1,enPin,Config::leftArmJ1ID);
servoICS::Servo leftArmJ2(&Serial1,enPin,Config::leftArmJ2ID);
servoICS::Servo leftArmJ3(&Serial1,enPin,Config::leftArmJ3ID);
servoICS::Servo leftArmJ4(&Serial1,enPin,Config::leftArmJ4ID);

servoICS::Servo rightArmJ1(&Serial1,enPin,Config::rightArmJ1ID);
servoICS::Servo rightArmJ2(&Serial1,enPin,Config::rightArmJ2ID);
servoICS::Servo rightArmJ3(&Serial1,enPin,Config::rightArmJ3ID);
servoICS::Servo rightArmJ4(&Serial1,enPin,Config::rightArmJ4ID);

servoICS::Servo* ServoArray[10]{
    &hipServo,
    &leftFoot1,
    &leftArmJ1,
    &leftArmJ2,
    &leftArmJ3,
    &leftArmJ4,
    &rightArmJ1,
    &rightArmJ2,
    &rightArmJ3,
    &rightArmJ4
};

const char* ServoArray_name[10]{
    "hipServo",
    "leftFoot1",
    "leftArmJ1",
    "leftArmJ2",
    "leftArmJ3",
    "leftArmJ4",
    "rightArmJ1",
    "rightArmJ2",
    "rightArmJ3",
    "rightArmJ4"
};