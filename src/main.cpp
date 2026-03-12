#include <Arduino.h>
#include <servoICS.h>

#include "PS4Controller_support.h"
#include "esp_timer.h"

#include "tread.cpp"


servoICS::Servo servo1;
PS4Controller_support Dualshock4;
PS4Controller_support* DS4 = &Dualshock4;

namespace walk{
  float x,y;

  bool isForward;
  bool isBackward;
  bool isMoveRight;
  bool isMoveLeft;


  void update(){
    x = (float)map(DS4->LStickX(),-128,127,-1000,1000) * 0.1;
    y = (float)map(DS4->LStickY(),-128,127,-1000,1000) * 0.1;

    if(x > 50){
      isForward = true;
      isBackward = false;
    }else if(x < -50){
      isForward = false;
      isBackward = true;
    }else{
      isForward = false;
      isBackward = false;
    }

    if(y > 50){
      isMoveRight = true;
      isMoveLeft = false;
    }else if(y < -50){
      isMoveLeft = true;
      isMoveRight = false;
    }else{
      isMoveRight = false;
      isMoveLeft = false;
    }
  }
  
}



void setup() {
  // put your setup code here, to run once:
  Dualshock4.begin("06:02:01:02:05:10");
  Dualshock4.update();

  Serial.begin(115200);
}

void loop() {
  if (PS4.isConnected()) {
    Dualshock4.update();
    //Serial.printf("Right:%d Down:%d Up:%d Left:%d Square:%d Cross:%d Circle:%d Triangle:%d UpRight:%d DownRight:%d UpLeft:%d DownLeft:%d\n", Dualshock4.Right(), Dualshock4.Down(), Dualshock4.Up(), Dualshock4.Left(), Dualshock4.Square(), Dualshock4.Cross(), Dualshock4.Circle(), Dualshock4.Triangle(), Dualshock4.UpRight(), Dualshock4.DownRight(), Dualshock4.UpLeft(), Dualshock4.DownLeft());
    //Serial.printf("LStickX:%d LStickY:%d RStickX:%d RStickY:%d\n", Dualshock4.LStickX(), Dualshock4.LStickY(), Dualshock4.RStickX(), Dualshock4.RStickY());
    float x = (float)map(Dualshock4.RStickX(),-128,127,-1000,1000) * 0.1;
    Serial.printf("x:%f\n",x);

    walk::update();

    if(walk::isForward){
      Serial.printf("forward\n");
      
    
    }


    delay(1);
  }
}


/*
    Serial.printf("forward:%d\n",walk::forward);
    Serial.printf("backward:%d\n",walk::backward);
    Serial.printf("moveRight:%d\n",walk::moveRight);
    Serial.printf("moveLeft:%d\n",walk::moveLeft);
*/