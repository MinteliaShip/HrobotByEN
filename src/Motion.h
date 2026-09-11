#include <Arduino.h>
#include "AssistiveProgram.h"

#ifndef MOTION
#define MOTION

namespace motion{//モーション関数の置き場所
    namespace walk{
        bool walk1();
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

        bool hip();//腰回転
    }
}

#endif
