#include "Motion.h"
#include "ConfigDef.h"

bool motion_sub(int totalSteps,int index,int joint_num,int &currentStep,int &taskPhase,const float *armAngle_zero,const float *armAngle_tar1);

void VIB(){//バイブレーション＆赤LED
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

bool startWalking(GaitParameters &param_p){//静止状態から歩行状態への移行
// 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T * 1.00f;
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
            float h = map(frame,0,totalFrames,0,(int)param_p.h);//推移していく。

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
    float T = param_p.T * 1.0f;
    //float h = param_p.h * 0.4f;
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
            float h = map(totalFrames - frame, 0, totalFrames, 0, (int)param_p.h);

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
    float T = param_p.T;
    float h = param_p.h;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    float Spac = param_p.Spac;
    float Wd = param_p.Wd;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = param_p.offsetX_left;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = param_p.offsetX_right;

    float kickX_left_val = param_p.kickX_left;
    float kickY_left_val = param_p.kickY_left;
    float kickX_right_val = param_p.kickX_right;
    float kickY_right_val = param_p.kickY_right;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    switch (taskPhase){
        case 0:
        {
            for(int i=0;i<19;i++){
                ServoArray[i]->setStretch(stretch);
                delay(5);
            }
            leftFoot.FootMotorInvert(1, 1, 1, -1, -1);
            rightFoot.FootMotorInvert(1, 1, 1, 1, 1);

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
            float offsetY_def =0 * stick_lx / 100.0;
            float angle_def = (5*PI/360.0)*stick_lx / 100.0;

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
            //行進のコントローラがない場合、または、回転のボタンが押された場合は抜ける。 
            if((!(stick_ly > 0) || Dualshock4.data.button.l3) && (frame>=totalFrames)){//最終フレームで抜けるように
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

bool motion::walk::walkY(){
    //MV_X_PARAM_FAST_Y
    //MV_X_PARAM_1Y
    GaitParameters &param_p = Config::MV_X_PARAM_FAST_Y;

    // 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T;
    float h = param_p.h;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    float Spac = param_p.Spac;
    float Wd = param_p.Wd;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = param_p.offsetX_left;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = param_p.offsetX_right;

    float kickX_left_val = param_p.kickX_left;
    float kickY_left_val = param_p.kickY_left;
    float kickX_right_val = param_p.kickX_right;
    float kickY_right_val = param_p.kickY_right;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);



    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    //左右に移動
    static float moveValue;

    switch (taskPhase){
        case 0:
        {
            for(int i=0;i<19;i++){
                ServoArray[i]->setStretch(stretch);
                delay(5);
            }
            leftFoot.FootMotorInvert(1, 1, 1, -1, -1);
            rightFoot.FootMotorInvert(1, 1, 1, 1, 1);

            taskPhase = 1;
            moveValue = 0;
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


            
            
            moveValue = moveValue + (-(float)Dualshock4.data.button.left + (float)Dualshock4.data.button.right)*0.05;
            if(moveValue > 1){
                moveValue = 1;
            }else if(moveValue < -1){
                moveValue = -1;
            }

            Serial.printf("Dualshock4.data.button.left:%d Dualshock4.data.button.right:%d moveValue:%f\n",Dualshock4.data.button.left,Dualshock4.data.button.right,moveValue);

            float wd_left = Wd*moveValue;
            float wd_right = Wd*moveValue;

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
            
            float angle_def = 0;

            if (ts_left > (support_end - push_window) && ts_left < support_end) {
                kick_x_left = kickX_left_val - 0;
                kick_y_left = kickY_left_val + 0;
                angle_left = angle_def;
            }

            if (ts_right > (support_end - push_window) && ts_right < support_end) {
                kick_x_right = kickX_right_val - 0;
                kick_y_right = kickY_right_val + 0;
                angle_right = angle_def;
            }

            FootController::Pose leftPose={
            offsetZ_left-leftPosXY.y - kick_y_left,-Spac-leftPosXY.x  + kick_x_left*moveValue ,0 + offsetX_left,
            angle_left, 0, 0
            };

            FootController::Pose rightPose={
            offsetZ_right-rightPosXY.y - kick_y_right,+Spac-rightPosXY.x + kick_x_right*moveValue,0 + offsetX_right,
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
            //行進のコントローラがない場合、または、回転のボタンが押された場合は抜ける。 
            if((!Dualshock4.data.button.left && !Dualshock4.data.button.right) && (frame>=totalFrames)){//最終フレームで抜けるように
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

bool motion::walk::turn(){
    GaitParameters &param_p = Config::MV_X_PARAM_TURN;

    // 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T;
    float h = param_p.h;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    float Spac = param_p.Spac;
    float Wd = param_p.Wd;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = param_p.offsetX_left;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = param_p.offsetX_right;

    float kickX_left_val = param_p.kickX_left;
    float kickY_left_val = param_p.kickY_left;
    float kickX_right_val = param_p.kickX_right;
    float kickY_right_val = param_p.kickY_right;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    switch (taskPhase){
        case 0:
        {
            Serial.print(F("turn \n"));
            for(int i=0;i<19;i++){
                ServoArray[i]->setStretch(stretch);
                delay(5);
            }
            leftFoot.FootMotorInvert(1, 1, 1, -1, -1);
            rightFoot.FootMotorInvert(1, 1, 1, 1, 1);

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

            float wd_left = -Wd * (stick_lx*0.01);
            float wd_right = Wd * (stick_lx*0.01);

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
                kick_x_left = kickX_left_val - 0;
                kick_y_left = kickY_left_val + 0;
                angle_left = 0;
            }

            if (ts_right > (support_end - push_window) && ts_right < support_end) {
                kick_x_right = kickX_right_val - 0;
                kick_y_right = kickY_right_val + 0;
                angle_right = 0;
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
            //行進のコントローラがない場合、または、回転のボタンが押された場合は抜けない
            if(!Dualshock4.data.button.l3 && frame>=totalFrames){//最終フレームで抜けるように
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
    int joint_num = 4;

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
            motion_sub(15,index,joint_num,currentStep,taskPhase,armAngle_zero,armAngle_tar);
            break;
        }
        case 2:
        {
            motion_sub(30,index,joint_num,currentStep,taskPhase,armAngle_tar,armAngle_zero);
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
    Serial.printf("attack_Heavy_1!\n");
    return false;
}

bool motion::posture::battle::attack_Heavy_2(){
    Serial.printf("attack_Heavy_2!\n");
    return false;
}

bool motion::posture::getUp(){

    static int taskPhase;
    static bool getup_mode;

    switch (taskPhase){
        case 0:
        {
            Serial.printf("getUp!\n");
            for(int i=0;i<19;i++){
                ServoArray[i]->setSkip(true);
                delay(5);
            }
            taskPhase = 10;//最初のフレームは10から
            getup_mode = !g_isFaceUp;//0:仰向け 1:うつ伏せ
            break;
        }
        case 10:
        {
            bool isNext=1;
            if(getup_mode==0){
                isNext = motion::posture::getUp_prone();
            }else{
                isNext = motion::posture::getUp_supine();
            }

            if(isNext==0){
                taskPhase = 11;//次の処理へ。
            }
            break;
        }
        case 11:
        {
            taskPhase = 0;
            for(int i=0;i<19;i++){
                ServoArray[i]->setSkip(false);
                delay(5);
            }
            return false;
            break;
        }
    }
    return true;
}

bool motion::posture::getUp_prone(){
    static int taskPhase = 0;
    static int currentStep = 0;
    static int arrIndex = 0;

    // モーション構造体定義
    struct MotionStep {
        int frameNum;
        float pose[19];
    };

    // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
    static const MotionStep motionData[] PROGMEM = {
        /* 0: posZero   */ {30, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
        /* 1:    */        {20, {0.64,-101.55,3.37,90.21,-34.96,-83.23,-2.84,89.24,-10.73,-4.49,-21.60,74.55,-90.65,-16.67,14.78,28.11,76.44,77.29,20.76}},
        /*              */ {20, {0.64,-91.83,-5.16,-6.21,-4.42,-88.69,3.00,165.68,1.92,7.32,-1.49,71.35,-94.80,0.00,18.06,6.78,76.78,79.72,9.15}},
        /*              */ {120, {-9.99,-85.35,71.45,-0.47,-4.76,-93.22,-65.75,165.34,1.62,6.99,-1.49,71.68,-95.14,-0.30,18.06,6.45,76.44,80.02,8.84}},
        /* 6: posZero   */ { 0, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}}
    };

    const int arrNum = sizeof(motionData) / sizeof(motionData[0]);
    const int index = 0;
    const int joint_num = 19;

    switch (taskPhase) {
        case 0:
            Serial.printf("getUp_supine!\n");
            taskPhase = 10;
            currentStep = 0;
            arrIndex = 0;
            break;

        case 10:
        {
            // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
            int dummyPhase = 0;

            // 1ステップ分（1フレーム分）の補間・出力処理
            bool isBusy = motion_sub(
                motionData[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData[arrIndex].pose,motionData[arrIndex + 1].pose
            );

            // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
            if (!isBusy) {
                arrIndex++; // 次の姿勢ペアへ進める

                // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                if (arrIndex >= arrNum - 1) {
                    taskPhase = 11;
                }
            }
            break;
        }

        case 11:
            taskPhase = 0;
            currentStep = 0;
            arrIndex = 0;
            return false; // モーション完了
    }

    return true; // モーション継続中
}

bool motion::posture::getUp_supine() {
    static int taskPhase = 0;
    static int currentStep = 0;
    static int arrIndex = 0;

    // モーション構造体定義
    struct MotionStep {
        int frameNum;
        float pose[19];
    };

    // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
    static const MotionStep motionData[] PROGMEM = {
        /* 0: posZero   */ {30, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
        /* 1: posSupine */ {30, {0.00, -71.42, 100.95, 58.42, -90.82, 61.66, 90.45, -51.30, -85.22, 2.77, -4.18, -8.30, -6.92, -4.25, 14.48, 5.80, -1.62, -0.44, 13.67}},
        /*    支援      */ {25, {0.00,-4.86,102.53,35.98,-33.65,-3.14,91.70,-52.28,43.67,-3.14,-75.77,21.06,-96.73,-76.34,14.14,80.05,25.04,79.72,82.28}}, 
        /* 2: posTar1   */ {25, {0.00, -56.40, 1.45, 9.25, -129.74, 32.30, -3.51, 9.25, -119.31, 23.93, -90.82, 22.07, -99.93, -84.04, 11.88, 91.33, 4.25, 90.04, 99.09}},
        /* 3: posTar2   */ //{25, {0.00, 35.98, 30.21, 46.37, 0.81, -19.81, 39.05, -57.81, 6.51, 3.41, -93.08, -11.58, -90.96, -84.04, 8.94, 91.97, -6.92, 80.02, 98.75}},
        /* 4: posTar3   */ {120, {0.00, -75.60, 21.16, 28.15, -75.63, 29.67, 23.12, 31.39, -57.95, -11.47, -94.37, 37.66, -98.96, -93.01, 23.25, 92.61, 42.83, 84.27, 91.33}},
        /* 5: posTar4   */ {120, {0.00, -25.41, 33.45, 9.59, 21.67, 4.93, 36.48, 22.61, -15.63, 2.77, -0.13, 75.84, -51.17, -1.62, 11.21, 30.37, 62.61, 49.34, 32.03}},
        /* 6: posZero   */ { 0, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}}
    };

    const int arrNum = sizeof(motionData) / sizeof(motionData[0]);
    const int index = 0;
    const int joint_num = 19;

    switch (taskPhase) {
        case 0:
            Serial.printf("getUp_supine!\n");
            taskPhase = 10;
            currentStep = 0;
            arrIndex = 0;
            break;

        case 10:
        {
            // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
            int dummyPhase = 0;

            // 1ステップ分（1フレーム分）の補間・出力処理
            bool isBusy = motion_sub(
                motionData[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData[arrIndex].pose,motionData[arrIndex + 1].pose
            );

            // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
            if (!isBusy) {
                arrIndex++; // 次の姿勢ペアへ進める

                // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                if (arrIndex >= arrNum - 1) {
                    taskPhase = 11;
                }
            }
            break;
        }

        case 11:
            taskPhase = 0;
            currentStep = 0;
            arrIndex = 0;
            return false; // モーション完了
    }

    return true; // モーション継続中
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
                delay(5);
                auto returnData = ServoArray[i]->setPosFree().getPosDeg();
                Serial.printf("[%s]:%f (%s)\n",ServoArray_name[i],returnData.value,returnData.error_msg);
                delay(5);
            }
            Serial.print("{");
            for(int i = 0;i < 19;i++){
                delay(5);
                auto returnData = ServoArray[i]->setPosFree().getPosDeg();
                if(i!=0)Serial.print(",");
                Serial.print(returnData.value);
                armAngle_set[i]=returnData.value;
                delay(5);
            }
            Serial.println("}");

            delay(1500);
            Serial.println("---");
        }else{
            for(int i=0;i<19;i++){
                ServoArray[i]->setPosDeg(armAngle_set[i],false);
                delay(5);
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

//教示によるモーション再生用の補助プログラム
bool motion_sub(int totalSteps,int index,int joint_num,int &currentStep,int &taskPhase,const float *armAngle_zero,const float *armAngle_tar1){
    for(int i=0;i<joint_num;i++){
        float currentPos = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
        ServoArray[i+index]->setPosDeg(currentPos,false);
        delay(2);
    }
    if(currentStep < totalSteps){
        currentStep++;
    }else{
        taskPhase++;
        currentStep=0;
        return false;
    }
    return true;
}