#include "Motion.h"
#include "ConfigDef.h"

void VIB(){
    Dualshock4.setLed(255, 10, 10);
    Dualshock4.setRumble(255, 0);
    Dualshock4.sendToController();
    resetRumble(300);
}

float phaseShift_f(float inStep,float phaseShift){
    float result = 0;
    if(phaseShift>inStep){
        result = inStep + phaseShift;
    }else{
        result = inStep - phaseShift;
    }
    return result;
}

//    FrameLimiter framelim;
//        framelim.setInterval(1000 / param_p.Fps);
//        // 指定FPS間隔の同期・待機処理
//framelim.sync();
bool startWalking(GaitParameters &param_p){//静止状態から歩行状態への移行
// 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T * 0.90f;
    float h = param_p.h * 0.5f;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    float Spac = param_p.Spac;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = -15.0f;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = -15.0f;

    float kickX_left_val = param_p.kickX_left * 0.0f;
    float kickY_left_val = param_p.kickY_left * 0.0f;
    float kickX_right_val = param_p.kickX_right * 0.0f;
    float kickY_right_val = param_p.kickY_right * 0.0f;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    switch (taskPhase){
        case 0:
        {
            leftFoot.FootMotorInvert(1, 1, 1, -1, 1);
            rightFoot.FootMotorInvert(1, 1, 1, 1, -1);

            taskPhase = 1;
            break;
        }
        case 1:
        {
            // 経過時間 ts の計算 (秒)
            float ts_left = (float)frame / Fps;
            // 右足は位相を半周期 (T / 2.0) ずらす
            float ts_right = phaseShift_f(ts_left,T/2.0);

            float Wd = map(frame,0,totalFrames,0,(int)param_p.Wd);//推移していく。

            // 軌道生成処理 (FootController.cpp の tread 関数を利用)
            Vector2 leftPosXY = leftFoot.tread(h,Wd,DutyX,DutyY,T,ts_left);
            Vector2 rightPosXY = rightFoot.tread(h,Wd,DutyX,DutyY,T,ts_right);

            float kick_x_left = 0.0f;
            float kick_y_left = 0.0f;
            float kick_x_right = 0.0f;
            float kick_y_right = 0.0f;

            // 接地期の終盤で後ろ・下へ押し込む
            float support_end = DutyX * T / 2.0f;

            if (ts_left > (support_end - push_window) && ts_left < support_end) {
                kick_x_left = kickX_left_val;
                kick_y_left = kickY_left_val;
            }

            if (ts_right > (support_end - push_window) && ts_right < support_end) {
                kick_x_right = kickX_right_val;
                kick_y_right = kickY_right_val;
            }


            FootController::Pose leftPose={
            offsetZ_left-leftPosXY.y - kick_y_left,-Spac,-leftPosXY.x + offsetX_left + kick_x_left,
            0, 0, 0
            };

            FootController::Pose rightPose={
            offsetZ_right-rightPosXY.y - kick_y_right,+Spac,-rightPosXY.x + offsetX_right + kick_x_right,
            0, 0, 0
            };

            // 逆運動学 (IK) を介して各足のサーボへ指令を出力
            leftFoot.setTargetPose(leftPose);
            rightFoot.setTargetPose(rightPose);

            
            if(frame < totalFrames){
                frame++;
            }else{
                frame = 0;
                taskPhase = 0;//最初に戻る

                return false;//終了時は、falseを返す
            }
            break;
        }
    }
    return true;//実行中は、trueを返す
}

bool endWalking(GaitParameters &param_p) {//歩行状態から静止状態へ移行
    // 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T * 0.70f;
    float h = param_p.h * 0.5f;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    int Spac = param_p.Spac;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = param_p.offsetX_left;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = param_p.offsetX_right;

    float kickX_left_val = param_p.kickX_left * 0.0f;
    float kickY_left_val = param_p.kickY_left * 0.0f;
    float kickX_right_val = param_p.kickX_right * 0.0f;
    float kickY_right_val = param_p.kickY_right * 0.0f;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    switch (taskPhase) {
        case 0:
        {
            leftFoot.FootMotorInvert(1, 1, 1, -1, 1);
            rightFoot.FootMotorInvert(1, 1, 1, 1, -1);

            taskPhase = 1;
            break;
        }
        case 1:
        {
            // 経過時間 ts の計算 (秒)
            float ts_left = (float)frame / Fps;
            // 右足は位相を半周期 (T / 2.0) ずらす
            float ts_right = phaseShift_f(ts_left, T / 2.0f);

            // 終了へ向かうため、Wdを逆向きに減衰させる
            float Wd = map(totalFrames - frame, 0, totalFrames, 0, (int)param_p.Wd);

            // 軌道生成処理 (FootController.cpp の tread 関数を利用)
            Vector2 leftPosXY = leftFoot.tread(h, Wd, DutyX, DutyY, T, ts_left);
            Vector2 rightPosXY = rightFoot.tread(h, Wd, DutyX, DutyY, T, ts_right);

            float kick_x_left = 0.0f;
            float kick_y_left = 0.0f;
            float kick_x_right = 0.0f;
            float kick_y_right = 0.0f;

            // 接地期の終盤で後ろ・下へ押し込む
            float support_end = DutyX * T / 2.0f;

            if (ts_left > (support_end - push_window) && ts_left < support_end) {
                kick_x_left = kickX_left_val;
                kick_y_left = kickY_left_val;
            }

            if (ts_right > (support_end - push_window) && ts_right < support_end) {
                kick_x_right = kickX_right_val;
                kick_y_right = kickY_right_val;
            }

            FootController::Pose leftPose = {
                offsetZ_left - leftPosXY.y - kick_y_left, -Spac, -leftPosXY.x + offsetX_left + kick_x_left,
                0, 0, 0
            };

            FootController::Pose rightPose = {
                offsetZ_right - rightPosXY.y - kick_y_right, +Spac, -rightPosXY.x + offsetX_right + kick_x_right,
                0, 0, 0
            };

            // 逆運動学 (IK) を介して各足のサーボへ指令を出力
            leftFoot.setTargetPose(leftPose);
            rightFoot.setTargetPose(rightPose);

            if (frame < totalFrames) {
                frame++;
            } else {
                frame = 0;
                taskPhase = 0; // 最初に戻る

                return false; // 終了時は、falseを返す
            }
            break;
        }
    }
    return true; // 実行中は、trueを返す
}


bool motion::walk::walk1() {
    GaitParameters &param_p = Config::MV_X_PARAM_3;

    // 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T * 0.90f;
    float h = param_p.h * 0.5f;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    float Spac = param_p.Spac;
    float Wd = param_p.Wd;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = param_p.offsetX_left;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = param_p.offsetX_right;

    float kickX_left_val = param_p.kickX_left * 0.0f;
    float kickY_left_val = param_p.kickY_left * 0.0f;
    float kickX_right_val = param_p.kickX_right * 0.0f;
    float kickY_right_val = param_p.kickY_right * 0.0f;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    switch (taskPhase){
        case 0:
        {
            leftFoot.FootMotorInvert(1, 1, 1, -1, 1);
            rightFoot.FootMotorInvert(1, 1, 1, 1, -1);

            taskPhase = 1;
            break;
        }
        case 1:
        {
            //歩行準備
            if(!startWalking(param_p))taskPhase = 2;
            break;
        }
        case 2:
        {
            // 経過時間 ts の計算 (秒)
            float ts_left = (float)frame / Fps;
            // 右足は位相を半周期 (T / 2.0) ずらす
            float ts_right = fmod(ts_left + (T / 2.0f), T);

            int stick_lx = map_controller(Dualshock4.data.analog.stick.lx,20,-128,127,-100,100);

            float wd_def = (Wd*0.5) * stick_lx / 100.0;
            float offsetY_def = 0*0.5 * stick_lx / 100.0;
            float angle_def = (15*PI/360.0)*stick_lx / 100.0;

            float wd_left = Wd + wd_def;
            float wd_right = Wd - wd_def;

            // 軌道生成処理 (FootController.cpp の tread 関数を利用)
            Vector2 leftPosXY = leftFoot.tread(h,wd_left,DutyX,DutyY,T,ts_left);
            Vector2 rightPosXY = rightFoot.tread(h,wd_right,DutyX,DutyY,T,phaseShift_f(ts_left,T/2.0));

            float kick_x_left = 0.0f;
            float kick_y_left = 0.0f;
            float angle_left=0;

            float kick_x_right = 0.0f;
            float kick_y_right = 0.0f;
            float angle_right=0;

            // 接地期の終盤で後ろ・下へ押し込む
            float support_end = DutyX * T / 2.0f;

            if (ts_left > (support_end - push_window) && ts_left < support_end) {
                kick_x_left = kickX_left_val - offsetY_def;
                kick_y_left = kickY_left_val + offsetY_def;
                angle_left = angle_def;
            }

            if (ts_right > (support_end - push_window) && ts_right < support_end) {
                kick_x_right = kickX_right_val - offsetY_def;
                kick_y_right = kickY_right_val + offsetY_def;
                angle_right = angle_def;
            }

            FootController::Pose leftPose={
            offsetZ_left-leftPosXY.y - kick_y_left,-Spac,-leftPosXY.x + offsetX_left + kick_x_left,
            angle_left, 0, 0
            };

            FootController::Pose rightPose={
            offsetZ_right-rightPosXY.y - kick_y_right,+Spac,-rightPosXY.x + offsetX_right + kick_x_right,
            angle_right, 0, 0
            };

            // 逆運動学 (IK) を介して各足のサーボへ指令を出力
            leftFoot.setTargetPose(leftPose);
            rightFoot.setTargetPose(rightPose);

            if(frame < totalFrames){
                frame++;
            }else{
                frame = 0;
            }

            int stick_ly = map_controller(Dualshock4.data.analog.stick.ly,20,-128,127,-10,10);
            //コントローラが押されていない時は、次のフェーズへ。
            if(!(Dualshock4.data.button.l3 || (stick_ly > 0))){
                taskPhase = 3;
                frame = 0;
            }
            break;
        }        
        case 3:
        {
            //歩行準備
            if(!endWalking(param_p)){
                taskPhase = 0;
                return false;//終了時は、falseを返す
            }
        }
    }
    return true;//実行中は、trueを返す
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

bool motion::posture::hip(){
    // 現在の角度を保持する静的変数
    static float current_angle = 0.0f;
    // スティック入力から目標角度を計算
    float target_angle = map_controller(Dualshock4.data.analog.stick.rx, 20, -128, 127, -30, 30);
    // 追従係数（0.0 〜 1.0）
    // 値が小さいほど滑らかに遅れて追従し、大きいほど素早く追従する
    float alpha = 0.5f; 
    current_angle += (target_angle - current_angle) * alpha;
    // 補間された角度をサーボに出力
    ServoArray[0]->setPosDeg(current_angle);

    return 0;
}

bool motion::posture::taunt(){
    Serial.printf("taunt 立ち姿勢!\n");
    for(int i=0;i<19;i++){
        ServoArray[i]->setPosDeg(0);
    }
    return 0;
}

bool motion::posture::battle::attack_Light_right(){//弱攻撃

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int currentStep=0;

    float armAngle_zero[4]={0,70,33,-130};//rightArmJ1~rightArmJ4 ServoArray[5]~ServoArray[8]
    float armAngle_tar[4]={-14,73,-55.8,-43.6};


    int totalSteps=15;
    float currentPos[4];

    int index = rightArm_index_num;

    switch (taskPhase){
        case 0:
        {
            Serial.printf("attack_Light_1!\n");
            VIB();//バイブレーション
            taskPhase = 1;
            currentStep=0;
            break;
        }
        case 1:
        {
            int totalSteps=15;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }

            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 2;
                currentStep=0;
            }
            break;
        }
        case 2:
        {
            int totalSteps=20;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_tar[i], armAngle_zero[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }
            
            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 3;
                currentStep=0;
            }
            break;
        }
        case 3:
        {
            taskPhase = 0;
            currentStep=0;
            return false;
        }

    }
    return true;
}


bool motion::posture::battle::attack_Light_left(){//弱攻撃

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int currentStep=0;

    float armAngle_zero[4]={4,80,-16,-130};//leftArmJ1~leftArmJ4 ServoArray[1]~ServoArray[4]
    float armAngle_tar[4]={5,43,75,-14};


    int totalSteps=15;
    float currentPos[4];

    int index = leftArm_index_num;

    switch (taskPhase){
        case 0:
        {
            Serial.printf("attack_Light_2!\n");
            VIB();//バイブレーション
            taskPhase = 1;
            currentStep=0;
            break;
        }
        case 1:
        {
            int totalSteps=15;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }

            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 2;
                currentStep=0;
            }
            break;
        }
        case 2:
        {
            int totalSteps=20;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_tar[i], armAngle_zero[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }
            
            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 3;
                currentStep=0;
            }
            break;
        }
        case 3:
        {
            taskPhase = 0;
            currentStep=0;
            return false;
        }

    }
    return true;
}


bool motion::posture::battle::attack_Medium_left(){

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int currentStep=0;

    float armAngle_zero[4]={4,80,-16,-130};//leftArmJ1~leftArmJ4 ServoArray[1]~ServoArray[4]
    float armAngle_tar1[4]={-13,41,-26,-71};
    float armAngle_tar2[4]={50,50,76,-18};


    int totalSteps=15;
    float currentPos[4];

    int index = leftArm_index_num;

    switch (taskPhase){
        case 0:
        {
            Serial.printf("attack_Medium_1!\n");
            VIB();//バイブレーション
            taskPhase = 1;
            currentStep=0;
            break;
        }
        case 1:
        {
            int totalSteps=15;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }

            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 2;
                currentStep=0;
            }
            break;
        }
        case 2:
        {
            int totalSteps=15;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_tar1[i], armAngle_tar2[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }

            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 3;
                currentStep=0;
            }
            break;
        }
        case 3:
        {
            int totalSteps=20;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_tar2[i], armAngle_zero[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }
            
            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 4;
                currentStep=0;
            }
            break;
        }
        case 4:
        {
            taskPhase = 0;
            currentStep=0;
            return false;
        }

    }
    return true;
}


bool motion::posture::battle::attack_Medium_right(){
    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int currentStep=0;

    float armAngle_zero[4]={0,70,33,-130};//ServoArray[5]~ServoArray[8]
    float armAngle_tar1[4]={-2.8,90.7,-7.8,-113.9};
    float armAngle_tar2[4]={-19,87,3.4,-100};

    int totalSteps=15;
    float currentPos[4];

    int index = rightArm_index_num;

    switch (taskPhase){
        case 0:
        {
            Serial.printf("attack_Medium_2!\n");
            VIB();//バイブレーション
            taskPhase = 1;
            currentStep=0;
            break;
        }
        case 1:
        {
            int totalSteps=15;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }

            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 2;
                currentStep=0;
            }
            break;
        }
        case 2:
        {
            int totalSteps=15;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_tar1[i], armAngle_tar2[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }

            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 3;
                currentStep=0;
            }
            break;
        }
        case 3:
        {
            int totalSteps=20;
            for(int i=0;i<4;i++){
                currentPos[i] = calculateStepMotion(armAngle_tar2[i], armAngle_zero[i], currentStep, totalSteps);
                ServoArray[i+index]->setPosDeg(currentPos[i]);
            }
            
            if(currentStep < totalSteps){
                currentStep++;
            }else{
                taskPhase = 4;
                currentStep=0;
            }
            break;
        }
        case 4:
        {
            taskPhase = 0;
            currentStep=0;
            return false;
        }

    }
    return true;
}



bool motion::posture::battle::attack_Heavy_1(){
    return false;
}

bool motion::posture::battle::attack_Heavy_2(){
    Serial.printf("attack_Heavy_2!\n");
    return false;
}

bool motion::posture::nop(){
    Dualshock4.setLed(100, 100, 100);
    Dualshock4.sendToController();
    Serial.printf(".");
    return false;
}

bool motion::posture::LOCK_DebugMode(){
    Serial.printf("DebugMode! comand!\n");
    float armAngle_set[19];
    bool free_mode = true;

    bool push_circle = Dualshock4.data.button.circle;

    while(!Dualshock4.data.button.cross){
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
    }

    return false;
}

bool motion::posture::pose() {
    float tarPos[9] = {0.00,0.0,60.00,23.59,-109.79,00.00,60.00,7.96,-99.49};
    for(int i=0;i<9;i++){
        ServoArray[i]->setPosDeg(tarPos[i]);
    }

    return false;
}

bool motion::posture::chair() {
    return false;
}

bool motion::posture::kneeling() {
    return false;
}