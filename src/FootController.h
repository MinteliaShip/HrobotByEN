#ifndef FOOTCONTROLLER_H
#define FOOTCONTROLLER_H

#include <servoICS.h>
#include <ArduinoEigen.h>
#include <math.h>

class FootController{
    struct footJoint5
    {
        float J1;
        float J2;
        float J3;
        float J4;
        float J5;
    };

  public:
    struct
    {
        char J1_ID;
        char J2_ID;
        char J3_ID;
        char J4_ID;
        char J5_ID;
        char enPin;
        Stream* stream;
    }typedef IcsServoConfig;

    struct Pose
    {
      float X;
      float Y;
      float Z;
      float Roll;
      float Pitch;
      float Yaw;
    };

    struct leng8
    {
      float L1;
      float L2;
      float L3;
      float L4;
      float L5;
      float L6;
      float L7;
      float L8;
    };

    private:
    leng8 footLeng;
    servoICS::Servo servoJ1;
    servoICS::Servo servoJ2;
    servoICS::Servo servoJ3;
    servoICS::Servo servoJ4;
    servoICS::Servo servoJ5;

    public:
    FootController(IcsServoConfig IcsServoConfig_, leng8 footLeng_);

    void setJointAngles(long J1_,long J2_,long J3_,long J4_,long J5_);
    void setJointAnglesDeg(float J1_,float J2_,float J3_,float J4_,float J5_);
    void setJointAnglesRad(float J1_,float J2_,float J3_,float J4_,float J5_);

    void setTargetPose(const Pose& targetPos,int mode=0);

    private:
    //計算補助関数
    float calcAngle(float LengA, float LengB, float LengC);
    float length(float x, float y, float z = 0);
};

#endif