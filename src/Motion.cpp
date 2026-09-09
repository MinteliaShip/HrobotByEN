#include "Motion.h"
#include "ConfigDef.h"

float phaseShift_f(float inStep,float phaseShift){
  float result = 0;
  if(phaseShift>inStep){
    result = inStep + phaseShift;
  }else{
    result = inStep - phaseShift;
  }
  return result;
}

void motion::walk::walk1() {
    Serial.printf("walk!\n");


    FrameLimiter framelim;

    GaitParameters &param_p = Config::MV_X_PARAM;

    // 歩行パラメータで指定されたFPSに設定
    framelim.setInterval(1000 / param_p.Fps);

    float T = param_p.T;
    float h = param_p.h;
    float Wd = param_p.Wd;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    int Spac = 40;

    // 1周期あたりの総フレーム数
    int totalFrames = Fps * T;

    // 前傾・足下ろし位置の高さ初期値（必要に応じて調整）
    float targetZ = -150.0f;

    leftFoot.FootMotorInvert(1,1,1,-1,1);
    rightFoot.FootMotorInvert(1,1,1,1,-1);

    for (int frame = 0; frame < totalFrames; frame++) {
        // 経過時間 ts の計算 (秒)
        float ts_left = (float)frame / Fps;
        // 右足は位相を半周期 (T / 2.0) ずらす
        float ts_right = fmod(ts_left + (T / 2.0f), T);

        // 軌道生成処理 (FootController.cpp の tread 関数を利用)
        Vector2 leftPosXY = leftFoot.tread(h,Wd,DutyX,DutyY,T,ts_left);
        
        Vector2 rightPosXY = rightFoot.tread(h,Wd,DutyX,DutyY,T,phaseShift_f(ts_left,T/2.0));


        // 遊脚直前の蹴り下げオフセット処理 (脚全体での蹴り出し)
        float kick_x_left = 0.0f;
        float kick_y_left = 0.0f;
        float kick_x_right = 0.0f;
        float kick_y_right = 0.0f;

        // 接地期の終盤で後ろ・下へ押し込む
        float push_window = 0.3f; // 蹴り出し時間 (秒)
        float support_end = DutyX * T / 2.0f;

        if (ts_left > (support_end - push_window) && ts_left < support_end) {
            kick_x_left = -6.0f; // 後ろ方向へ押し出し (mm)
            kick_y_left = -5.0f;  // 地面方向へ押し下げ (mm)
        }

        if (ts_right > (support_end - push_window) && ts_right < support_end) {
            kick_x_right = -6.0f;
            kick_y_right = -5.0f;
        }


        FootController::Pose leftPose={
        323-leftPosXY.y - kick_y_left,-Spac,-leftPosXY.x -20 + kick_x_left,
        0, 0, 0
        };

        FootController::Pose rightPose={
        320-rightPosXY.y - kick_y_right,+Spac,-rightPosXY.x -20 + kick_x_right,
        0, 0, 0
        };

        // 逆運動学 (IK) を介して各足のサーボへ指令を出力
        leftFoot.setTargetPose(leftPose);
        rightFoot.setTargetPose(rightPose);

        // 次回タスク判定（タスク移行要求の確認）
        nextTask = taskManager(0);

        // 指定FPS間隔の同期・待機処理
        framelim.sync();
    }

    // 1歩行周期完了後、タスクの移行判定を許可
    nextTask = taskManager(1);
    return;
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
    for(int i=0;i<19;i++){
        ServoArray[i]->setPosDeg(0);
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
        Serial.printf("DebugMode!\n");

        if(push_circle==0 && Dualshock4.data.button.circle==1){
            free_mode = !free_mode;
        }
        push_circle=Dualshock4.data.button.circle;


        if(free_mode){
            for(int i = 0;i < 19;i++){
                auto returnData = ServoArray[i]->setPosFree().getPosDeg();
                Serial.printf("[%s]:%f (%s)\n",ServoArray_name[i],returnData.value,returnData.error_msg);
            }
            Serial.print("{");
            for(int i = 0;i < 19;i++){
                auto returnData = ServoArray[i]->setPosFree().getPosDeg();
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

void motion::posture::pose() {

    float tarPos[9] = {0.00,0.0,60.00,23.59,-109.79,00.00,60.00,7.96,-99.49};
    for(int i=0;i<9;i++){
        ServoArray[i]->setPosDeg(tarPos[i]);
    }

    nextTask = taskManager(1);

}