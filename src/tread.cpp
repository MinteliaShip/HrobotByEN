#include <math.h>

#define PI 3.14159265358979323846


float tread_y(float h,float T,float Duty,float ts_){
    float A  = 2*h / (1-Duty);
    float AngV = PI*4/((1-Duty)*T);
    float t_1 = Duty*T/2;
    float t_2 = T/2;
    float t_3 = T-Duty*T/2;

    float y_ = 0.0;

    if (ts_ < t_1) {
        y_ = 0;
    }else if (ts_ < t_2) {
        y_ = -A / (2 * AngV) * sin(AngV * (ts_ - t_1)) + A / 2 * (ts_ - t_1);
    }else if (ts_ < t_3) {
        y_ = A / (2 * AngV) * sin(AngV * (ts_ - t_2)) - A / 2 * (ts_ - t_2) + (1.0 - Duty) * T * A / 4.0;
    }else {
        y_ = 0;
    }
    return y_;
}

float tread_x(float Wd,float T,float Duty,float ts_){

    float A  = 2*Wd / (1-Duty);
    float AngV = PI*2/((1-Duty)*T);
    float t_1 = Duty*T/2;
    float t_2 = T-Duty*T/2;

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
