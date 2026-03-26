#ifndef CONTROLLER_TO_COMMAND_HPP
#define CONTROLLER_TO_COMMAND_HPP

#include <cstdint>
#include <PS4Controller.h>

namespace ControllerApp {

    // 座標や移動量を表すデータ構造
    struct Vector2 {
        float x;
        float y;
        Vector2(float _x = 0.0f, float _y = 0.0f) : x(_x), y(_y) {}
    };

    // コマンド（動作）の定義をまとめる
    struct Commands {
        //移動・姿勢
        Vector2 moveSpeed = {0.0f, 0.0f};;  //平行移動
        float moveAngle=0.0f;    //水平視点操作

        //攻撃モーション
        bool isAttack1=0;
        bool isAttack2=0;
        bool isAttack3=0;
        bool isAttack4=0;

        bool isTaunt=0;

        //特殊機能
        bool isSp1=0;
        bool isSp2=0;
        


        //起き上がりモーション
        bool isGetup=0;
        //しゃがみモーション
        bool isSquat=0;
    };


    class CommandConverter {
    private:
        ps4_t *inData_;
        Commands cmd_;
        Commands lastCmd_;
        int deadzone = 30;

        

    public:
        CommandConverter(ps4_t *inData_) : inData_(inData_){}

        const Commands& getCommands() const {
            return cmd_;
        }

        void setDeadzone(int deadzone_){
            deadzone = deadzone_;
        }

        // 生データを受け取り、変換したコマンドを返す
        void update() {

            //0±deadzone以内の値はすべて0として扱う。倒していないときには確実に0になるよう調整。
            int stick_lx = inData_->analog.stick.lx;
            int stick_ly = inData_->analog.stick.ly;
            if(stick_lx > deadzone){//deadzoneを超えたとき
                stick_lx-=deadzone;
            }else if(stick_lx < -deadzone){//-deadzone未満のとき
                stick_lx+=deadzone;
            }else{
                stick_lx = 0;
            }

            if(stick_ly > deadzone){//deadzoneを超えたとき
                stick_ly-=deadzone;
            }else if(stick_ly < -deadzone){//-deadzone未満のとき
                stick_ly+=deadzone;
            }else{
                stick_ly = 0;
            }

            if(stick_lx <= -128 + deadzone)stick_lx = -127 + deadzone;
            if(stick_ly <= -128 + deadzone)stick_ly = -127 + deadzone;

            cmd_.moveSpeed.x = static_cast<float>(map(stick_lx,-127 + deadzone,127 - deadzone,-1000,1000) * 0.1f);
            cmd_.moveSpeed.y = static_cast<float>(map(stick_ly,-127 + deadzone,127 - deadzone,-1000,1000) * 0.1f);
            
            cmd_.isAttack1 = inData_->button.l1;
            cmd_.isAttack2 = inData_->button.l2;
            cmd_.isAttack3 = inData_->button.r1;
            cmd_.isAttack4 = inData_->button.r2;

            cmd_.isTaunt = inData_->button.touchpad;




            cmd_.isGetup = inData_->button.share;
            cmd_.isSquat = inData_->button.options;

            cmd_.isSp1 = inData_->button.triangle;
            cmd_.isSp2 = inData_->button.circle;
        }
    };

} // namespace ControllerApp

#endif