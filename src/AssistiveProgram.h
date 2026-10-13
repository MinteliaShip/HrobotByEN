#ifndef ASPROG
#define ASPROG

#include <Arduino.h>
#include <esp_gap_bt_api.h>
#include <LittleFS.h>

// 2次元ベクトル構造体
struct Vector2 {
    float x;
    float y;
    Vector2(float _x = 0.0f, float _y = 0.0f) : x(_x), y(_y) {}
};

/**********************************************/
// 補助関数群
int map_controller(int in, int ineRange, int inMin, int inMax, int outMin, int outMax);

void bondReset();

class FrameLimiter {
public:
    unsigned long interval_ms;
    unsigned long next_target_ms;

    // 指定時間設定（間隔変更時にカウンタをリセット）
    void setInterval(unsigned long interval);

private:
    // 指定時間待ち
    void wait();

    // 時間更新
    void update();

public:
    // 同期を行い、目標時刻からの超過時間(ms)を返す（超過なしは 0）
    long sync();
};

void LittleFS_ini();
void listFiles();

#endif // ASPROG