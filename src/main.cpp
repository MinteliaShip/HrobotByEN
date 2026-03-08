#include <Arduino.h>
#include <servoICS.h>

#include "PS4Controller_support.h"
#include "esp_timer.h"

// put function declarations here:
int myFunction(int, int);

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
  Dualshock4.begin("12:04:00:00:05:05");
  Dualshock4.update();
  
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}