
#include "Motion.h"

namespace Motion
{  
  MotionController::MotionController(const uint16_t (*Motion_)[SERVO_NUM],int max_frames_,servoICS::Servo *servo_[SERVO_NUM]){
    Motion = Motion_;
    
    for(int i=0;i<SERVO_NUM;i++){
      servo[i] = servo_[i];
    }
    max_frames = max_frames_;
  }
  
  bool MotionController::beginMotion(){
    frameCount = 0;

    return true;
  }
  bool MotionController::loopMotion(){
    frameCount++;
    if(frameCount < max_frames){
      for(int i=0;i<SERVO_NUM;i++){
        servo[i]->setPos(Motion[frameCount][i]);
      }
      return true;
    }else{
      return false;//フレーム終了
    }
  }

  void MotionController::endMotion(){
    frameCount = 0;
  }

  void MotionController::readMotion(int fps, int max_frames_to_record) {
    static uint16_t recordBuffer[526][SERVO_NUM]; 
    
    if (max_frames_to_record > 526) max_frames_to_record = 526;
    if (max_frames_to_record <= 0) return; // 安全策

    TickType_t lastTimeTicks = xTaskGetTickCount();
    // 割り算の順序に注意（1000.0で浮動小数点計算させる）
    TickType_t cycleTimeTicks = pdMS_TO_TICKS(1000.0 / fps);

    Serial.println(F(">>> RECORDING START..."));

    for (int count = 0; count < max_frames_to_record; count++) {
      for (int i = 0; i < SERVO_NUM; i++) {
        recordBuffer[count][i] = servo[i]->setPos(0).getPos().value;
        Serial.print("."); 
      }
      
      // ループ内の Serial.printf は削除！
      
      // 進行状況を100フレームごとに表示する程度にする

      // 通信時間を差し引いた残りの時間だけ待機してくれる
      //vTaskDelayUntil(&lastTimeTicks, cycleTimeTicks);
    }

    Serial.println(F("\n>>> RECORDING DONE. Outputting data..."));

    // 出力ループ（ここは一気に出してOK）
    Serial.printf("const uint16_t motionCaptured[%d][%d] = {\n", max_frames_to_record, SERVO_NUM);
    for (int count = 0; count < max_frames_to_record; count++) {
      Serial.print(F("  {"));
      for (int i = 0; i < SERVO_NUM; i++) {
        Serial.print(recordBuffer[count][i]);
        if (i < SERVO_NUM - 1) Serial.print(F(", "));
      }
      Serial.println(count < max_frames_to_record - 1 ? F("},") : F("}"));
      
      // 出力データ量が多い場合、送信バッファ溢れを防ぐため少し待つ
      if (count % 10 == 0) delay(5); 
    }
    Serial.println(F("};"));
  }


} // namespace Motion
