#ifndef FOOTCONTROLLER_H
#define FOOTCONTROLLER_H

#include <servoICS.h>
#include <ArduinoEigen.h>
#include <math.h>
#include "ConfigDef.h"
#include "Vector.h"


class FootController{
  private:
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

    footJoint5 IK(const Pose& targetPose,int mode);
  public:
    FootController(IcsServoConfig IcsServoConfig_, leng8 footLeng_);

    float tread_y(float h,float T,float Duty,float ts_);
    float tread_x(float Wd,float T,float Duty,float ts_);
    Vector2 tread(float h,float Wd,float DutyX,float DutyY,float T,float ts);
    float tread_kick(float kickAngle,float T1,float T,float Duty,float ts);


    void setJointAngles(long J1_,long J2_,long J3_,long J4_,long J5_);
    void setJointAnglesDeg(float J1_,float J2_,float J3_,float J4_,float J5_);
    void setJointAnglesRad(float J1_,float J2_,float J3_,float J4_,float J5_);

    void setJointStretch(unsigned char stretch);

    void getJointAngles(long J1_,long J2_,long J3_,long J4_,long J5_);

    void setOffset(long J1_,long J2_,long J3_,long J4_,long J5_);
    void DemoPos();

    void setTargetPose(const Pose& targetPos,float kick=0,int mode=0);

    void setJointSkip(bool skip);

  private:
    //計算補助関数
    float calcAngle(float LengA, float LengB, float LengC);
    float length(float x, float y, float z = 0);
};

#endif