#include <Arduino.h>
#include <servoICS.h>

#include "PS4Controller_support.h"
#include "esp_timer.h"

servoICS::Servo servo1;
PS4Controller_support Dualshock4;


int executionCycil( double cycilTime, double offsetTime, double fps, void (*p_func)(double)){
  int64_t start_time = esp_timer_get_time();
  int64_t intervalTime = int64_t(1000000.0/fps);
  int64_t nextTime = start_time + intervalTime;

  while(true){
    nextTime += intervalTime;
    p_func(double(esp_timer_get_time() - start_time)*0.000001);
    while(esp_timer_get_time() - start_time < nextTime)delay(1);
    if(esp_timer_get_time() - start_time >= int64_t(cycilTime*1000000))break;
  }
}

void setup() {
  // put your setup code here, to run once:
  Dualshock4.begin("12:04:00:00:05:03");
  Dualshock4.update();

  Serial.begin(115200);
}

void loop() {
  if (PS4.isConnected()) {
    Dualshock4.update();
    Serial.printf("Right:%d Down:%d Up:%d Left:%d Square:%d Cross:%d Circle:%d Triangle:%d UpRight:%d DownRight:%d UpLeft:%d DownLeft:%d\n", Dualshock4.Right(), Dualshock4.Down(), Dualshock4.Up(), Dualshock4.Left(), Dualshock4.Square(), Dualshock4.Cross(), Dualshock4.Circle(), Dualshock4.Triangle(), Dualshock4.UpRight(), Dualshock4.DownRight(), Dualshock4.UpLeft(), Dualshock4.DownLeft());
    delay(1);
  }
}
