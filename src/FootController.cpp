#include "FootController.h"

FootController::footJoint5 FootController::IK(const Pose& targetPose,int mode){
  footJoint5 joint;

  Eigen::Vector4d Pe = {targetPose.X, targetPose.Y, targetPose.Z, 1};
  Eigen::Matrix4d T2_1;
  T2_1 <<
  1,  0              , 0               ,  -footLeng[1],
  0,  cos(targetPose.Roll), sin(targetPose.Roll),  0,
  0,  -sin(targetPose.Roll), cos(targetPose.Roll) ,  -footLeng[0],
  0,  0              , 0               ,  1;
  Eigen::Vector4d P2_e =T2_1 * Pe;
  joint.J1 = targetPose.Roll;

  Eigen::Vector4d P6_e = {cos(targetPose.Yaw)*footLeng[6], sin(targetPose.Yaw)*footLeng[6], -footLeng[7], 0};
  Eigen::Vector4d P2_6 = P2_e - P6_e;

  joint.J2 = atan2(P2_6(1), P2_6(0));
  joint.J5 = targetPose.Yaw - joint.J2;

  Eigen::Matrix4d T3_2;
  T3_2 <<
  cos(joint.J2), sin(joint.J2), 0, -footLeng[2] ,
  -sin(joint.J2),  cos(joint.J2), 0, 0,
  0            ,0            ,1        ,0,
  0            ,0            ,0        ,1;

  Eigen::Vector4d P3_6 = T3_2 * P2_6;
  Eigen::Vector4d P5_6 = {footLeng[5],0,0,0};
  Eigen::Vector4d P3_5 = P3_6 - P5_6;

  float LengQ = length(P3_5(0), P3_5(2));

  if(mode == 0){
      joint.J3 = atan2(P3_5(2), P3_5(0)) - calcAngle(footLeng[4], footLeng[3], LengQ);
      joint.J4 = PI - calcAngle(LengQ, footLeng[4], footLeng[3]);
  }else{
      joint.J3 = atan2(P3_5(2), P3_5(0)) + calcAngle(footLeng[4], footLeng[3], LengQ);
      joint.J4 = -PI + calcAngle(LengQ, footLeng[4], footLeng[3]);
  }

  return joint;
}


/*-------------------------------------*/
//歩行軌道生成
float FootController::tread_y(float h,float T,float Duty,float ts_){
  float A  = 2.0*h / (1.0-Duty);
  float AngV = PI*4.0/((1.0-Duty)*T);
  float t_1 = Duty*T/2.0;
  float t_2 = T/2.0;
  float t_3 = T-Duty*T/2.0;

  float y_ = 0.0;

  if (ts_ < t_1) {
      y_ = 0;
  }else if (ts_ < t_2) {
      y_ = -A / (2 * AngV) * sin(AngV * (ts_ - t_1)) + A / 2 * (ts_ - t_1);
  }else if(ts_ < t_3) {
      y_ = A / (2 * AngV) * sin(AngV * (ts_ - t_2)) - A / 2 * (ts_ - t_2) + (1.0 - Duty) * T * A / 4.0;
  }else {
      y_ = 0;
  }
  return y_;
}

float FootController::tread_x(float Wd,float T,float Duty,float ts_){

  float A  = 2.0*Wd / (1.0-Duty);
  float AngV = PI*2.0/((1.0-Duty)*T);
  float t_1 = Duty*T/2.0;
  float t_2 = T-Duty*T/2.0;
  float x_ = 0.0;

  if (ts_ < t_1) {
      x_ = -Wd * ts_;
  } 
  else if (ts_ < t_2) {
      float dt = ts_ - t_1;
      x_ = -A / (2.0f * AngV) * sinf(AngV * dt) + (A * 0.5f - Wd) * dt - Wd * t_1;
  } 
  else {
      x_ = -Wd * (ts_ - t_2) + Wd * t_1;
  }
  return x_;
}

Vector2 FootController::tread(float h,float Wd,float DutyX,float DutyY,float T,float ts){
  Vector2 result;
  result.x = tread_x(Wd,T,DutyX,ts);
  result.y = tread_y(h,T,DutyY,ts);
  return result;
}

/*-------------------------------------*/
float FootController::tread_kick(float kickAngle,float T1,float T,float Duty,float ts){
  float av = 2*PI/T1;
  
  float kick=0;
  if(ts<(Duty*T/2.0)-T1){
    kick = 0;
  }else if(ts<Duty*T/2.0){
    kick = -kickAngle*cos(av*(ts-(Duty*T/2.0)-T1))+kickAngle;
  }else{
    kick = 0;
  }
  return kick;
}

/*-------------------------------------*/
//歩行重心移動
float tread_z(float p,float T,float Duty,float ts_){
  float T_ = (2*Duty-1)*T;
  float av = 2*PI/T_;

  float section2 = T*(1-Duty)/2;
  float section3 = T/2*Duty;
  float section4 = T*(2-Duty)/2;
  float section5 = T*(1+Duty)/2;

  float z_ = 0.0;

  if (ts_ < section2) {//1
    z_ = p;
  } else if(ts_ < section3){//2
    z_ = p*cos(av*(ts_-section2));
  } else if(ts_ < section4){//3
    z_ = -p;
  } else if(ts_ < section5){//4
    z_ = -p*cos(av*(ts_-section4));
  } else{//5
    z_ = p;
  }

  return z_;
}

/*-------------------------------------*/
void FootController::FootMotorInvert(int J1_,int J2_,int J3_,int J4_,int J5_){
  JointInv[0] = J1_;
  JointInv[1] = J2_;
  JointInv[2] = J3_;
  JointInv[3] = J4_;
  JointInv[4] = J5_;
}


FootController::FootController(servoICS::Servo* ServoArray_[], float footLeng_[8]){
  servoJ1 = ServoArray_[0];
  servoJ2 = ServoArray_[1];
  servoJ3 = ServoArray_[2];
  servoJ4 = ServoArray_[3];
  servoJ5 = ServoArray_[4];
  footLeng = footLeng_;
}

void FootController::setJointAngles(long J1_,long J2_,long J3_,long J4_,long J5_){
  servoJ1->setPos(J1_);
  delay(1);
  servoJ2->setPos(J2_);
  delay(1);
  servoJ3->setPos(J3_);
  delay(1);
  servoJ4->setPos(J4_);
  delay(1);
  servoJ5->setPos(J5_);
  delay(1);
}
void FootController::setJointAnglesDeg(float J1_,float J2_,float J3_,float J4_,float J5_){
  setJointAngles(servoICS::fromDeg_toIcs(J1_),servoICS::fromDeg_toIcs(J2_),servoICS::fromDeg_toIcs(J3_),servoICS::fromDeg_toIcs(J4_),servoICS::fromDeg_toIcs(J5_)); 
}
void FootController::setJointAnglesRad(float J1_,float J2_,float J3_,float J4_,float J5_){
  setJointAngles(servoICS::fromRad_toIcs(J1_),servoICS::fromRad_toIcs(J2_),servoICS::fromRad_toIcs(J3_),servoICS::fromRad_toIcs(J4_),servoICS::fromRad_toIcs(J5_)); 
}


void FootController::setTargetPose(const Pose& targetPose,float kick,int mode){
  footJoint5 joint;
  joint = IK(targetPose,mode);

  #ifdef SIMULATION
  setJointAnglesRad(
    joint.J1,
    joint.J2,
    joint.J3,
    joint.J4 + joint.J3,//平行リンクのため
    joint.J5 + kick
  );
  #else
  setJointAnglesRad(
    joint.J1 * JointInv[0],
    joint.J2 * JointInv[1],
    -joint.J3 * JointInv[2],
    (joint.J4 + joint.J3) * JointInv[3],//平行リンクのため
    (-joint.J5 - kick)*JointInv[4]
  );
  #endif

}

void FootController::setJointStretch(unsigned char stretch){
  servoJ1->setStretch(stretch);
  servoJ2->setStretch(stretch);
  servoJ3->setStretch(stretch);
  servoJ4->setStretch(stretch);
  servoJ5->setStretch(stretch);
}

void FootController::setJointSkip(bool skip){
  servoJ1->setSkip(skip);
  servoJ2->setSkip(skip);
  servoJ3->setSkip(skip);
  servoJ4->setSkip(skip);
  servoJ5->setSkip(skip);
}

void FootController::DemoPos(){
  Serial.printf("J1:%d \n",servoJ1->setPos(0).getPos().value);
  Serial.printf("J2:%d \n",servoJ2->setPos(0).getPos().value);
  Serial.printf("J3:%d \n",servoJ3->setPos(0).getPos().value);
  Serial.printf("J4:%d \n",servoJ4->setPos(0).getPos().value);
  Serial.printf("J5:%d \n",servoJ5->setPos(0).getPos().value);
}

//計算補助
float FootController::calcAngle(float LengA, float LengB, float LengC)
{
    return acos((-LengA * LengA + LengB * LengB + LengC * LengC) / (2 * LengB * LengC));
}

float FootController::length(float x, float y, float z){
    return sqrt(x*x + y*y + z*z);
}
