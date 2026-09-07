#include "Motion.h"

void motion::walk::walk1(){
    Serial.printf("walk!\n");
    FrameLimiter framelim;
    framelim.setInterval(int(1000/30));//30fps設定



    for(int i = 0;i < 90;i++){
        Serial.printf("c:%d\n",i);
        nextTask = taskManager(0);//移行許可は出さない。
    }
    nextTask = taskManager(1);
    return;//処理終わり
}


// 1次元の目標角度を計算する関数（全ステップ管理版）
float calculateStepMotion(float start_angle, float target_angle, int current_step, int total_steps) {
    if (current_step >= total_steps) {
        return target_angle; // 終了
    }

    // 進行割合 (0.0 ～ 1.0)
    float t = (float)current_step / (float)total_steps;

    // 3次エルミート曲線（Smoothstep）による加減速
    // スタートとゴールで速度が 0（滑らかに発進・停止）になる
    float smooth_t = t * t * (3.0f - 2.0f * t);

    // 現在のステップにおける目標角度を返す
    return start_angle + (target_angle - start_angle) * smooth_t;
}


void motion::posture::taunt(){//弱攻撃
    Serial.printf("taunt 立ち姿勢!\n");
    float armAngle_zero[8]={0,0,0,0,0,0,0,0};//ServoArray[1]~ServoArray[8]

    for(int i=0;i<4;i++){
        ServoArray[i+1]->setPosDeg(armAngle_zero[i]);
    }

    nextTask = taskManager(1);
}

void motion::posture::battle::attack_Light_1(){//弱攻撃
    Serial.printf("attack_Light_1!\n");
    float armAngle_zero[4]={0,70,33,-130};//rightArmJ1~rightArmJ4 ServoArray[5]~ServoArray[8]
    float armAngle_tar[4]={-14,73,-55.8,-43.6};

    int currentStep=0;
    int totalSteps=15;
    float currentPos[4];

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar[i], currentStep, totalSteps);
            ServoArray[i+5]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(1);
    }

    delay(10);

    currentStep=0;
    totalSteps=25;

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar[i], armAngle_zero[i], currentStep, totalSteps);
            ServoArray[i+5]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(20);
    }
    nextTask = taskManager(1);
}

void motion::posture::battle::attack_Light_2(){
    Serial.printf("attack_Light_2!\n");
    float armAngle_zero[4]={4,80,-16,-130};//leftArmJ1~leftArmJ4 ServoArray[1]~ServoArray[4]
    float armAngle_tar[4]={5,43,75,-14};

    int currentStep=0;
    int totalSteps=20;
    float currentPos[4];

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(5);
    }

    delay(100);

    currentStep=0;

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar[i], armAngle_zero[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(10);
    }
    nextTask = taskManager(1);
}

void motion::posture::battle::attack_Medium_1(){//中攻撃
    Serial.printf("attack2!\n");
    float armAngle_zero[4]={4,80,-16,-130};//leftArmJ1~leftArmJ4 ServoArray[1]~ServoArray[4]
    float armAngle_tar1[4]={-13,41,-26,-71};
    float armAngle_tar2[4]={50,50,76,-18};

    int currentStep=0;
    int totalSteps=20;
    float currentPos[4];

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(5);
    }

    currentStep=0;

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar1[i], armAngle_tar2[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(1);
    }

    delay(10);

    currentStep=0;
    totalSteps=50;

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar2[i], armAngle_zero[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(15);
    }
    nextTask = taskManager(1);
}

void motion::posture::battle::attack_Medium_2(){
    Serial.printf("attack_Medium_2!\n");
    float armAngle_zero[4]={0,70,33,-130};//ServoArray[5]~ServoArray[8]
    float armAngle_tar1[4]={-2.8,90.7,-7.8,-113.9};
    float armAngle_tar2[4]={-19,87,3.4,-100};

    int currentStep=0;
    int totalSteps=20;
    float currentPos[4];

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
            ServoArray[i+5]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(5);
    }

    currentStep=0;

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar1[i], armAngle_tar2[i], currentStep, totalSteps);
            ServoArray[i+5]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(1);
    }

    delay(10);

    currentStep=0;
    totalSteps=50;

    while (currentStep <= totalSteps) {
        for(int i=0;i<4;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar2[i], armAngle_zero[i], currentStep, totalSteps);
            ServoArray[i+5]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(15);
    }
    nextTask = taskManager(1);
}


void motion::posture::battle::attack_Heavy_1(){
    nextTask = taskManager(1);
}

void motion::posture::battle::attack_Heavy_2(){
    Serial.printf("attack_Heavy_2!\n");
    float armAngle_zero[8]={4,80,-16,-130,0,70,33,-130};//ServoArray[1]~ServoArray[8]
    float armAngle_tar1[8]={75,69,4,-100,0,24,-166,-70};
    float armAngle_tar2[8]={-9,94,24,-70,22,31,-72,-43};

    int currentStep=0;
    int totalSteps=20;
    float currentPos[8];

    while (currentStep <= totalSteps) {
        for(int i=0;i<8;i++){
            currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(5);
    }

    currentStep=0;

    while (currentStep <= totalSteps) {
        for(int i=0;i<8;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar1[i], armAngle_tar2[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(1);
    }

    delay(10);

    currentStep=0;
    totalSteps=50;

    while (currentStep <= totalSteps) {
        for(int i=0;i<8;i++){
            currentPos[i] = calculateStepMotion(armAngle_tar2[i], armAngle_zero[i], currentStep, totalSteps);
            ServoArray[i+1]->setPosDeg(currentPos[i]);
        }
        currentStep++;
        delay(10);
    }
    nextTask = taskManager(1);
}




void motion::posture::nop(){
    Serial.printf("nop!\n");
    nextTask = taskManager(1);
    delay(10);
}



void motion::posture::DebugMode(){
    Serial.printf("DebugMode! comand!\n");
    nextTask = taskManager(0);
    float armAngle_set[19];
    bool free_mode = true;

    bool push_circle = Dualshock4.data.button.circle;

    while(nextTask == nullptr){

        if(push_circle==0 && Dualshock4.data.button.circle==1){
            free_mode = !free_mode;
        }
        push_circle=Dualshock4.data.button.circle;


        if(free_mode){
            for(int i = 0;i < 19;i++){
                auto returnData = ServoArray[i]->setPosIcs(0).getPosDeg();
                Serial.printf("[%s]:%f (%s)\n",ServoArray_name[i],returnData.value,returnData.error_msg);
            }
            Serial.print("{");
            for(int i = 0;i < 19;i++){
                auto returnData = ServoArray[i]->setPosIcs(0).getPosDeg();
                if(i!=0)Serial.print(",");
                Serial.print(returnData.value);
                armAngle_set[i]=returnData.value;
            }
            Serial.println("}");

            delay(1500);
            Serial.println("---");
        }else{
            for(int i=0;i<19;i++){
                ServoArray[i]->setPosDeg(armAngle_set[i]);
            }
            Serial.println("{");
            for(int i = 0;i < 19;i++){
                if(i!=0)Serial.print(",");
                Serial.print(armAngle_set[i]);
            }
            Serial.println("}");
            delay(1000);
        }

        if(Dualshock4.data.button.cross == 0){
            nextTask = taskManager(0);
        }else{
            nextTask = taskManager(1);
        }
    }
}