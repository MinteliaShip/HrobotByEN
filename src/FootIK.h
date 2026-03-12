#ifndef FOOT_IK_H
#define FOOT_IK_H

#include <ArduinoEigen.h>
#include <math.h>


namespace FootIK{
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

    struct footJoint5
    {
        float J1;
        float J2;
        float J3;
        float J4;
        float J5;
    };


    //角度aを求める余弦定理関数 入力はA,B,Cの長さとなっている。
    float calcAngle(float LengA, float LengB, float LengC)
    {
        return acos((-LengA * LengA + LengB * LengB + LengC * LengC) / (2 * LengB * LengC));
    }

    float length(float x, float y, float z = 0){
        return sqrt(x*x + y*y + z*z);
    }

    footJoint5 IK(Pose poses,leng8 lengs,int mode)
    {
        footJoint5 joint;

        Eigen::Vector4d Pe = {poses.X, poses.Y, poses.Z, 1};
        Eigen::Matrix4d T2_1;
        T2_1 <<
        1,  0              , 0               ,  -lengs.L2,
        0,  cos(poses.Roll), sin(poses.Roll),  0,
        0,  -sin(poses.Roll), cos(poses.Roll) ,  -lengs.L1,
        0,  0              , 0               ,  1;
        Eigen::Vector4d P2_e =T2_1 * Pe;
        joint.J1 = poses.Roll;

        Eigen::Vector4d P6_e = {cos(poses.Yaw)*lengs.L7, sin(poses.Yaw)*lengs.L7, -lengs.L8, 0};
        Eigen::Vector4d P2_6 = P2_e - P6_e;

        joint.J2 = atan2(P2_6(1), P2_6(0));
        joint.J5 = poses.Yaw - joint.J2;

        Eigen::Matrix4d T3_2;
        T3_2 <<
        cos(joint.J2), sin(joint.J2), 0, -lengs.L3 ,
        -sin(joint.J2),  cos(joint.J2), 0, 0,
        0            ,0            ,1        ,0,
        0            ,0            ,0        ,1;

        Eigen::Vector4d P3_6 = T3_2 * P2_6;
        Eigen::Vector4d P5_6 = {lengs.L6,0,0,0};
        Eigen::Vector4d P3_5 = P3_6 - P5_6;

        float LengQ = length(P3_5(0), P3_5(2));

        if(mode == 0){
            joint.J3 = atan2(P3_5(2), P3_5(0)) - calcAngle(lengs.L5, lengs.L4, LengQ);
            joint.J4 = PI - calcAngle(LengQ, lengs.L5, lengs.L4);
        }else{
            joint.J3 = atan2(P3_5(2), P3_5(0)) + calcAngle(lengs.L5, lengs.L4, LengQ);
            joint.J4 = PI + calcAngle(LengQ, lengs.L5, lengs.L4);
        }
        return joint;
    }
}
#endif
