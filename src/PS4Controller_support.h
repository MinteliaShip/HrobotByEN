/*
PS4Controllerライブラリをより使用しやすくする派生クラス
・begin()実行後に必ず、bondリセット関数を呼ぶようにし、接続失敗を減らす。
・Right()のように値を取得する関数を使用する場合、updata()を実行しない限り、値が更新されないようにした。
*/

#ifndef PS4Controller_support_h
#define PS4Controller_support_h
#include <PS4Controller.h>
#include <esp_gap_bt_api.h>

class PS4Controller_support : public PS4Controller {
public:
    PS4Controller_support(){};
    
    ps4_t data_support;
    
    void bondReset(){
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

    bool begin(){
        //成功時にtrueを返すようだ。
        bool result = PS4Controller::begin();
        if(result) bondReset();
        return result;
    }

    bool begin(const char* mac) {
        bool result = PS4Controller::begin(mac);
        if(result) bondReset();
        return result;
    }

    void update() {
        if (isConnected()) {
            data_support = data; // 親クラスの data を丸ごとコピー
        }
    }

public:
    bool Right() { return data_support.button.right; }
    bool Down() { return data_support.button.down; }
    bool Up() { return data_support.button.up; }
    bool Left() { return data_support.button.left; }

    bool Square() { return data_support.button.square; }
    bool Cross() { return data_support.button.cross; }
    bool Circle() { return data_support.button.circle; }
    bool Triangle() { return data_support.button.triangle; }

    bool UpRight() { return data_support.button.upright; }
    bool DownRight() { return data_support.button.downright; }
    bool UpLeft() { return data_support.button.upleft; }
    bool DownLeft() { return data_support.button.downleft; }

    bool L1() { return data_support.button.l1; }
    bool R1() { return data_support.button.r1; }
    bool L2() { return data_support.button.l2; }
    bool R2() { return data_support.button.r2; }

    bool Share() { return data_support.button.share; }
    bool Options() { return data_support.button.options; }
    bool L3() { return data_support.button.l3; }
    bool R3() { return data_support.button.r3; }

    bool PSButton() { return data_support.button.ps; }
    bool Touchpad() { return data_support.button.touchpad; }

    uint8_t L2Value() { return data_support.analog.button.l2; }
    uint8_t R2Value() { return data_support.analog.button.r2; }

    int8_t LStickX() { return data_support.analog.stick.lx; }
    int8_t LStickY() { return data_support.analog.stick.ly; }
    int8_t RStickX() { return data_support.analog.stick.rx; }
    int8_t RStickY() { return data_support.analog.stick.ry; }

    uint8_t Battery() { return data_support.status.battery; }
    bool Charging() { return data_support.status.charging; }
    bool Audio() { return data_support.status.audio; }
    bool Mic() { return data_support.status.mic; }
};

#endif