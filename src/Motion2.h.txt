
#ifndef Motion_H
#define Motion_H

#include <servoICS.h> // 近藤科学 ICSサーボ制御ライブラリを使用
#include <HardwareSerial.h>

namespace Motion
{
  #define SERVO_NUM 8

  class MotionController
  {
  private:
    servoICS::Servo *servo[SERVO_NUM];  //servoのポインタ
    const uint16_t (*Motion)[SERVO_NUM]; //保存したモーションをひと固まりとして指し示すポインタ
    int max_frames;
    int frameCount=0;

  public:
    MotionController(const uint16_t (*Motion_)[SERVO_NUM],int max_frames_, servoICS::Servo *servo_[SERVO_NUM]);
    bool beginMotion(); //モーション開始
    bool loopMotion();  //次のモーションへ
    void endMotion(); //片付け

    void readMotion(int fps,int max_frames); 
  };
} // namespace Motion

#endif

