#include "AssistiveProgram.h"

int map_controller(int in,int ineRange,int inMin,int inMax,int outMin,int outMax){//コントローラ用に不感範囲を設けたmap
    int in_abs = abs(in);
    int in_pn = in / in_abs;
    //Serial.printf("in_pn:%d \n",in_pn);

    int in_cut;
    if(in_abs < ineRange){
        in_cut = 0;
    }else{
        in_cut = in - ineRange*in_pn;
    }
    //Serial.printf("in_cut:%d \n",in_cut);

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

void FrameLimiter::setInterval(unsigned long interval){
    interval_ms = interval;
    next_target_ms = millis() + interval_ms;
}

void FrameLimiter::wait() {
    unsigned long now = millis();
    if ((long)(next_target_ms - now) > 0) {
        delay(next_target_ms - now);
    }
}

void FrameLimiter::update() {
    next_target_ms += interval_ms;
    // 処理落ちが激しすぎて目標時間を大幅に超過した場合のリカバリ
    if ((long)(millis() - next_target_ms) > 0) {
        next_target_ms = millis() + interval_ms;
    }
}

void FrameLimiter::sync(){
    wait();
    update();
}

void LittleFS_ini(){
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }
    Serial.println("LittleFS Mounted Successfully");
}

void listFiles() {
    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) {
        Serial.println("Failed to open root directory");
        return;
    }

    Serial.println("--- File List ---");
    File file = root.openNextFile();
    bool found = false;
    
    while (file) {
        found = true;
        // ファイル名とサイズを表示
        Serial.printf("Name: %-15s | Size: %u bytes\n", file.name(), file.size());
        file = root.openNextFile();
    }
    
    if (!found) {
        Serial.println("No files found.");
    }
    Serial.println("-----------------");
}

