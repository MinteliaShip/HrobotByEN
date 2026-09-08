#include <Arduino.h>
#include "AssistiveProgram.h"

#ifndef MOTION
#define MOTION

namespace motion{//モーション関数の置き場所
    namespace walk{
        void walk1();
    }

    namespace posture{
        namespace battle{
            void attack_Light_1();
            void attack_Light_2();
            void attack_Medium_1();
            void attack_Medium_2();
            void attack_Heavy_1();
            void attack_Heavy_2();
        }

        void taunt();
        void nop();
        void DebugMode();
        void pose();
    }
}

#endif
