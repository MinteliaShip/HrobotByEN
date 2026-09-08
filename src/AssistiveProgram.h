#include "Declaration.h"

#ifndef ASPROG
#define ASPROG

/**********************************************/
//補助関数
int map_controller(int in,int ineRange,int inMin,int inMax,int outMin,int outMax);

void bondReset();

class FrameLimiter {//処理間隔の一定
    public:
        unsigned long interval_ms;
        unsigned long next_target_ms;

        // 1. 指定時間設定（間隔変更時にカウンタをリセット）
        void setInterval(unsigned long interval);
    private:

        // 2. 指定時間待ち（指定時間未満なら待機、超過ならそのまま通過）
        void wait();

        // 3. 時間更新（処理終了時に次回の目標時間を登録）
        void update();
    public:
        void sync();
};


void LittleFS_ini();
void listFiles();
#endif