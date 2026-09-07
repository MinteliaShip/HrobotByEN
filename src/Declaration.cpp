#include "Declaration.h"

PS4Controller Dualshock4;//実態の定義
NextTaskType nextTask;

FootController leftFoot(Config::leftfootConfig,Config::lengs8);
FootController rightFoot(Config::rightfootConfig,Config::lengs8);
servoICS::Servo hipServo(&Serial1,enPin,Config::hipServoID);

servoICS::Servo leftFoot1(&Serial1,enPin,Config::leftFootJ1ID);
servoICS::Servo leftFoot2(&Serial1,enPin,Config::leftFootJ2ID);
servoICS::Servo leftFoot3(&Serial1,enPin,Config::leftFootJ3ID);
servoICS::Servo leftFoot4(&Serial1,enPin,Config::leftFootJ4ID);
servoICS::Servo leftFoot5(&Serial1,enPin,Config::leftFootJ5ID);

servoICS::Servo rightFoot1(&Serial1,enPin,Config::rightFootJ1ID);
servoICS::Servo rightFoot2(&Serial1,enPin,Config::rightFootJ2ID);
servoICS::Servo rightFoot3(&Serial1,enPin,Config::rightFootJ3ID);
servoICS::Servo rightFoot4(&Serial1,enPin,Config::rightFootJ4ID);
servoICS::Servo rightFoot5(&Serial1,enPin,Config::rightFootJ5ID);

servoICS::Servo leftArmJ1(&Serial1,enPin,Config::leftArmJ1ID);
servoICS::Servo leftArmJ2(&Serial1,enPin,Config::leftArmJ2ID);
servoICS::Servo leftArmJ3(&Serial1,enPin,Config::leftArmJ3ID);
servoICS::Servo leftArmJ4(&Serial1,enPin,Config::leftArmJ4ID);

servoICS::Servo rightArmJ1(&Serial1,enPin,Config::rightArmJ1ID);
servoICS::Servo rightArmJ2(&Serial1,enPin,Config::rightArmJ2ID);
servoICS::Servo rightArmJ3(&Serial1,enPin,Config::rightArmJ3ID);
servoICS::Servo rightArmJ4(&Serial1,enPin,Config::rightArmJ4ID);

servoICS::Servo* ServoArray[19]{
    &hipServo,
    &leftArmJ1,
    &leftArmJ2,
    &leftArmJ3,
    &leftArmJ4,
    &rightArmJ1,
    &rightArmJ2,
    &rightArmJ3,
    &rightArmJ4,
    &leftFoot1,
    &leftFoot2,
    &leftFoot3,
    &leftFoot4,
    &leftFoot5,
    &rightFoot1,
    &rightFoot2,
    &rightFoot3,
    &rightFoot4,
    &rightFoot5
};

const char* ServoArray_name[19]{
    "hipServo",
    "leftArmJ1",
    "leftArmJ2",
    "leftArmJ3",
    "leftArmJ4",
    "rightArmJ1",
    "rightArmJ2",
    "rightArmJ3",
    "rightArmJ4",
    "leftFoot1",
    "leftFoot2",
    "leftFoot3",
    "leftFoot4",
    "leftFoot5",
    "rightFoot1",
    "rightFoot2",
    "rightFoot3",
    "rightFoot4",
    "rightFoot5"
};