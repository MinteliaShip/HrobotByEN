#include "Declaration.h"

PS4Controller Dualshock4;//実態の定義
NextTaskType nextTask;

servoICS::Servo hipServo(Config::ServoSerial,enPin,Config::hipServoID);
servoICS::Servo leftFoot1(Config::ServoSerial,enPin,Config::leftFootJ1ID);
servoICS::Servo leftFoot2(Config::ServoSerial,enPin,Config::leftFootJ2ID);
servoICS::Servo leftFoot3(Config::ServoSerial,enPin,Config::leftFootJ3ID);
servoICS::Servo leftFoot4(Config::ServoSerial,enPin,Config::leftFootJ4ID);
servoICS::Servo leftFoot5(Config::ServoSerial,enPin,Config::leftFootJ5ID);

servoICS::Servo rightFoot1(Config::ServoSerial,enPin,Config::rightFootJ1ID);
servoICS::Servo rightFoot2(Config::ServoSerial,enPin,Config::rightFootJ2ID);
servoICS::Servo rightFoot3(Config::ServoSerial,enPin,Config::rightFootJ3ID);
servoICS::Servo rightFoot4(Config::ServoSerial,enPin,Config::rightFootJ4ID);
servoICS::Servo rightFoot5(Config::ServoSerial,enPin,Config::rightFootJ5ID);

servoICS::Servo leftArmJ1(Config::ServoSerial,enPin,Config::leftArmJ1ID);
servoICS::Servo leftArmJ2(Config::ServoSerial,enPin,Config::leftArmJ2ID);
servoICS::Servo leftArmJ3(Config::ServoSerial,enPin,Config::leftArmJ3ID);
servoICS::Servo leftArmJ4(Config::ServoSerial,enPin,Config::leftArmJ4ID);

servoICS::Servo rightArmJ1(Config::ServoSerial,enPin,Config::rightArmJ1ID);
servoICS::Servo rightArmJ2(Config::ServoSerial,enPin,Config::rightArmJ2ID);
servoICS::Servo rightArmJ3(Config::ServoSerial,enPin,Config::rightArmJ3ID);
servoICS::Servo rightArmJ4(Config::ServoSerial,enPin,Config::rightArmJ4ID);

servoICS::Servo* ServoArray[19]{
    &hipServo,          //0
    &leftArmJ1,         //1
    &leftArmJ2,         //2
    &leftArmJ3,         //3
    &leftArmJ4,         //4
    &rightArmJ1,        //5
    &rightArmJ2,        //6
    &rightArmJ3,        //7
    &rightArmJ4,        //8
    &leftFoot1,         //9
    &leftFoot2,         //10
    &leftFoot3,         //11
    &leftFoot4,         //12
    &leftFoot5,         //13
    &rightFoot1,        //14
    &rightFoot2,        //15
    &rightFoot3,        //16
    &rightFoot4,        //17
    &rightFoot5         //18
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


FootController leftFoot(&ServoArray[9],Config::lengs8);
FootController rightFoot(&ServoArray[14],Config::lengs8);