#include "Declaration.h"

#ifndef ASPROG
#define ASPROG

/**********************************************/
//補助関数
//未検証
int map_controller(int in,int ineRange,int inMin,int inMax,int outMin,int outMax){//コントローラ用に不感範囲を設けたmap
    int in_abs = abs(in);
    int in_pn = in / in_abs;

    int in_cut;
    if(in_abs < ineRange){
        in_cut = 0;
    }else{
        in_cut = in - ineRange*in_pn;
    }

    return map(in_cut,inMin+ineRange,inMax-ineRange,outMin,outMax);
}

void bondReset(){//ボード履歴削除
    // 1. 保存されているデバイスの数を確認
    int dev_num = esp_bt_gap_get_bond_device_num();

    if (dev_num > 0) {
        esp_bd_addr_t dev_list[dev_num];
        // 2. デバイスリストを取得
        esp_bt_gap_get_bond_device_list(&dev_num, dev_list);
        
        // 3. 全てのデバイス情報を削除（リセット）
        for (int i = 0; i < dev_num; i++) {
        esp_bt_gap_remove_bond_device(dev_list[i]);
        }
    }
}

class FrameLimiter {//処理間隔の一定
    public:
        unsigned long interval_ms;
        unsigned long next_target_ms;

        // 1. 指定時間設定（間隔変更時にカウンタをリセット）
        void setInterval(unsigned long interval) {
        interval_ms = interval;
        next_target_ms = millis() + interval_ms;
        }
    private:

        // 2. 指定時間待ち（指定時間未満なら待機、超過ならそのまま通過）
        void wait() {
        unsigned long now = millis();
        if ((long)(next_target_ms - now) > 0) {
            delay(next_target_ms - now);
        }
        }

        // 3. 時間更新（処理終了時に次回の目標時間を登録）
        void update() {
        next_target_ms += interval_ms;
        // 処理落ちが激しすぎて目標時間を大幅に超過した場合のリカバリ
        if ((long)(millis() - next_target_ms) > 0) {
            next_target_ms = millis() + interval_ms;
        }
        }
    public:
        void sync(){
        wait();
        update();
        }
};


#endif