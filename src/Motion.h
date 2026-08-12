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
        void attack1();
        void attack2();
        }

        void taunt();
        void nop();
    }
}

#endif
