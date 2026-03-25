

#include "FootController.h"

FootController::FootController(IcsServoConfig IcsServoConfig_, leng8 footLeng_){
  servoJ1.attach(IcsServoConfig_.stream,IcsServoConfig_.enPin,IcsServoConfig_.J1_ID);
  servoJ2.attach(IcsServoConfig_.stream,IcsServoConfig_.enPin,IcsServoConfig_.J2_ID);
  servoJ3.attach(IcsServoConfig_.stream,IcsServoConfig_.enPin,IcsServoConfig_.J3_ID);
  servoJ4.attach(IcsServoConfig_.stream,IcsServoConfig_.enPin,IcsServoConfig_.J4_ID);
  servoJ5.attach(IcsServoConfig_.stream,IcsServoConfig_.enPin,IcsServoConfig_.J5_ID);
  footLeng = footLeng_;
}

void FootController::setJointAngles(long J1_,long J2_,long J3_,long J4_,long J5_){
  servoJ1.setPos(J1_);
  servoJ2.setPos(J2_);
  servoJ3.setPos(J3_);
  servoJ4.setPos(J4_);
  servoJ5.setPos(J5_);
}
void FootController::setJointAnglesDeg(float J1_,float J2_,float J3_,float J4_,float J5_){
  setJointAngles(servoICS::fromDeg_toIcs(J1_),servoICS::fromDeg_toIcs(J2_),servoICS::fromDeg_toIcs(J3_),servoICS::fromDeg_toIcs(J4_),servoICS::fromDeg_toIcs(J5_)); 
}
void FootController::setJointAnglesRad(float J1_,float J2_,float J3_,float J4_,float J5_){
  setJointAngles(servoICS::fromRad_toIcs(J1_),servoICS::fromRad_toIcs(J2_),servoICS::fromRad_toIcs(J3_),servoICS::fromRad_toIcs(J4_),servoICS::fromRad_toIcs(J5_)); 
}

void FootController::setTargetPose(const Pose& targetPose,int mode){
  footJoint5 joint;

  Eigen::Vector4d Pe = {targetPose.X, targetPose.Y, targetPose.Z, 1};
  Eigen::Matrix4d T2_1;
  T2_1 <<
  1,  0              , 0               ,  -footLeng.L2,
  0,  cos(targetPose.Roll), sin(targetPose.Roll),  0,
  0,  -sin(targetPose.Roll), cos(targetPose.Roll) ,  -footLeng.L1,
  0,  0              , 0               ,  1;
  Eigen::Vector4d P2_e =T2_1 * Pe;
  joint.J1 = targetPose.Roll;

  Eigen::Vector4d P6_e = {cos(targetPose.Yaw)*footLeng.L7, sin(targetPose.Yaw)*footLeng.L7, -footLeng.L8, 0};
  Eigen::Vector4d P2_6 = P2_e - P6_e;

  joint.J2 = atan2(P2_6(1), P2_6(0));
  joint.J5 = targetPose.Yaw - joint.J2;

  Eigen::Matrix4d T3_2;
  T3_2 <<
  cos(joint.J2), sin(joint.J2), 0, -footLeng.L3 ,
  -sin(joint.J2),  cos(joint.J2), 0, 0,
  0            ,0            ,1        ,0,
  0            ,0            ,0        ,1;

  Eigen::Vector4d P3_6 = T3_2 * P2_6;
  Eigen::Vector4d P5_6 = {footLeng.L6,0,0,0};
  Eigen::Vector4d P3_5 = P3_6 - P5_6;

  float LengQ = length(P3_5(0), P3_5(2));

  if(mode == 0){
      joint.J3 = atan2(P3_5(2), P3_5(0)) - calcAngle(footLeng.L5, footLeng.L4, LengQ);
      joint.J4 = PI - calcAngle(LengQ, footLeng.L5, footLeng.L4);
  }else{
      joint.J3 = atan2(P3_5(2), P3_5(0)) + calcAngle(footLeng.L5, footLeng.L4, LengQ);
      joint.J4 = -PI + calcAngle(LengQ, footLeng.L5, footLeng.L4);
  }


  setJointAnglesRad(
    joint.J1,
    joint.J2,
    joint.J3,
    joint.J4 + joint.J3,//平行リンクのため
    joint.J5
  );
}

//計算補助
float FootController::calcAngle(float LengA, float LengB, float LengC)
{
    return acos((-LengA * LengA + LengB * LengB + LengC * LengC) / (2 * LengB * LengC));
}

float FootController::length(float x, float y, float z){
    return sqrt(x*x + y*y + z*z);
}
