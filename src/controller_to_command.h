#ifndef CONTROLLER_TO_COMMAND_HPP
#define CONTROLLER_TO_COMMAND_HPP

#include <cstdint>
#include <cmath>
#include <PS4Controller.h>

#include "Vector.h"

namespace ControllerApp {
// Config.h 等に定義
    enum ButtonID : uint8_t {
        BTN_NONE = 0,
        
        // 方向キー
        BTN_UP,
        BTN_DOWN,
        BTN_LEFT,
        BTN_RIGHT,
        
        // メインボタン
        BTN_CIRCLE,
        BTN_CROSS,
        BTN_SQUARE,
        BTN_TRIANGLE,
        
        // ショルダー・トリガー
        BTN_L1,
        BTN_R1,
        BTN_L2,
        BTN_R2,
        
        // 特殊ボタン
        BTN_SHARE,
        BTN_OPTIONS,
        BTN_PS,
        BTN_TOUCHPAD,
        BTN_L3,
        BTN_R3,

        // 斜め入力（必要であれば）
        BTN_UPRIGHT,
        BTN_DOWNRIGHT,
        BTN_UPLEFT,
        BTN_DOWNLEFT
        
    };

    enum AnalogID {
        ANALOG_LX, // 左スティック X軸
        ANALOG_LY, // 左スティック Y軸
        ANALOG_RX, // 右スティック X軸
        ANALOG_RY, // 右スティック Y軸
        ANALOG_L2, // L2トリガー
        ANALOG_R2  // R2トリガー
    };


    struct controllerMapping{
        ButtonID attack1;
        ButtonID attack2;
        ButtonID attack3;
        ButtonID attack4;

        ButtonID sp1;
        ButtonID sp2;

        ButtonID getup;
        ButtonID squat;

        ButtonID taunt;

        AnalogID moveX;
        AnalogID moveY;
        AnalogID lookX;

        controllerMapping(
            ButtonID attack1_, ButtonID attack2_, ButtonID attack3_, ButtonID attack4_,
            ButtonID sp1_, ButtonID sp2_,
            ButtonID getup_, ButtonID squat_,
            ButtonID taunt_,
            AnalogID moveX_, AnalogID moveY_, AnalogID lookX_
        )
        :
        attack1(attack1_), attack2(attack2_), attack3(attack3_), attack4(attack4_),
        sp1(sp1_), sp2(sp2_),
        getup(getup_), squat(squat_),
        taunt(taunt_),
        moveX(moveX_), moveY(moveY_), lookX(lookX_)
        {}
    };


    // コマンド（動作）の定義をまとめる
    struct Commands {
        //攻撃モーション
        bool isAttack1=0;
        bool isAttack2=0;
        bool isAttack3=0;
        bool isAttack4=0;
        //特殊機能
        bool isSp1=0;
        bool isSp2=0;
        //非攻撃モーション
        bool isGetup=0;     //起き上がりモーション
        bool isSquat=0;     //しゃがみモーション
        bool isTaunt=0;     //特殊モーション

        Vector2 move = {0.0f, 0.0f};

        float look;
    };



    class CommandConverter {
    private:
        ps4_t *inData_;
        controllerMapping mapping_;
        Commands cmd_;
        Commands lastCmd_;
        int deadzone = 20;

        bool getButtonState(const ps4_button_t& data, ButtonID id) {
            switch (id) {
                case BTN_UP:       return data.up;
                case BTN_DOWN:     return data.down;
                case BTN_LEFT:     return data.left;
                case BTN_RIGHT:    return data.right;
                
                case BTN_CIRCLE:   return data.circle;
                case BTN_CROSS:    return data.cross;
                case BTN_SQUARE:   return data.square;
                case BTN_TRIANGLE: return data.triangle;
                
                case BTN_L1:       return data.l1;
                case BTN_R1:       return data.r1;
                case BTN_L2:       return data.l2;
                case BTN_R2:       return data.r2;
                
                case BTN_SHARE:    return data.share;
                case BTN_OPTIONS:  return data.options;
                case BTN_PS:       return data.ps;
                case BTN_TOUCHPAD: return data.touchpad;
                case BTN_L3:       return data.l3;
                case BTN_R3:       return data.r3;

                default:           return false;
            }
        }

        int16_t getAnalogValue(const ps4_analog_t& data, AnalogID id) {
            switch (id) {
                case ANALOG_LX: return data.stick.lx;
                case ANALOG_LY: return data.stick.ly;
                case ANALOG_RX: return data.stick.rx;
                case ANALOG_RY: return data.stick.ry;
                case ANALOG_L2: return data.button.l2;
                case ANALOG_R2: return data.button.r2;
                default: return 0;
            }
        }

        

    public:
        CommandConverter(ps4_t *inData_,controllerMapping mapping__) : inData_(inData_), mapping_(mapping__){}

        const Commands& getCommands() const {
            return cmd_;
        }

        void setDeadzone(int deadzone_){
            deadzone = deadzone_;
        }

        // 生データを受け取り、変換したコマンドを返す
        void update() {

            //0±deadzone以内の値はすべて0として扱う。倒していないときには確実に0になるよう調整。
            int move_x = getAnalogValue(inData_->analog,mapping_.moveX);
            int move_y = getAnalogValue(inData_->analog,mapping_.moveY);

            int look = getAnalogValue(inData_->analog,mapping_.lookX);

            if(move_x == -128){
                move_x = -127;
            }
            if(move_y == -128){
                move_y = -127;
            }
            if(look == -128){
                look = -127;
            }

            if(abs(move_x) < deadzone){
                move_x = 0;
            }else if(move_x > 0){
                move_x -= deadzone;
            }else{
                move_x += deadzone;            
            }
            if(abs(move_y) < deadzone){
                move_y = 0;
            }else if(move_y > 0){
                move_y -= deadzone;
            }else{
                move_y += deadzone;            
            }
            if(abs(look) < deadzone){
                look = 0;
            }else if(look > 0){
                look -= deadzone;
            }else{
                look += deadzone;            
            }


            float move_x_f = map(move_x,-127+deadzone,127-deadzone,-100,100)*0.01;
            float move_y_f = map(move_y,-127+deadzone,127-deadzone,-100,100)*0.01;
            float look_f = map(look,-127+deadzone,127-deadzone,-100,100)*0.01;

            cmd_.move = Vector2(move_x_f,move_y_f);
            cmd_.look = look_f;

            cmd_.isAttack1 = getButtonState(inData_->button,mapping_.attack1);
            cmd_.isAttack2 = getButtonState(inData_->button,mapping_.attack2);
            cmd_.isAttack3 = getButtonState(inData_->button,mapping_.attack3);
            cmd_.isAttack4 = getButtonState(inData_->button,mapping_.attack4);
            
            cmd_.isTaunt = getButtonState(inData_->button,mapping_.taunt);
            
            cmd_.isGetup = getButtonState(inData_->button,mapping_.getup);
            cmd_.isSquat = getButtonState(inData_->button,mapping_.squat);

            cmd_.isSp1 = getButtonState(inData_->button,mapping_.sp1);
            cmd_.isSp2 = getButtonState(inData_->button,mapping_.sp2);
        }
    };

}
#endif