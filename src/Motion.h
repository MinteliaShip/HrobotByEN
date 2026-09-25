#include <Arduino.h>
#include "AssistiveProgram.h"

#ifndef MOTION
#define MOTION

namespace motion{//モーション関数の置き場所
    namespace walk{
        bool walk1();
        bool walkBack();
        bool walkY();
        bool turn();
    }

    namespace posture{
        namespace battle{
            bool attack_Light_left();
            bool attack_Light_right();
            bool attack_Medium_left();
            bool attack_Medium_right();
            bool attack_Heavy_1();
            bool attack_Heavy_2();
        }

        bool taunt();
        bool nop();
        bool LOCK_DebugMode();//ブロッキング
        bool pose();
        bool chair();   //椅子に座る
        bool kneeling();   //膝立ち
        bool getUp();//起き上がり

        bool getUp_supine();//仰向け
        bool getUp_prone();//うつ伏せ

        bool sit_stand();
        bool sitDown();//座る
        bool standUp();//立ち上がる

        bool lowerPos();//姿勢を低く、

        bool hip();//腰回転
    }
}

#endif
