// SPDX-License-Identifier: LGPL-2.1-or-later
// Copyright (C) 2025 MinteIiaShip

#include "servoICS.h"

namespace servoICS {

    // 静的メンバ変数の定義
    std::vector<Servo*> Servo::instances_;

    void Servo::registerInstance_(Servo* instance) {
        for (auto p : instances_) {
            if (p == instance) return;
        }
        instances_.push_back(instance);
    }

    void Servo::unregisterInstance_(Servo* instance) {
        for (auto it = instances_.begin(); it != instances_.end(); ++it) {
            if (*it == instance) {
                instances_.erase(it);
                break;
            }
        }
    }

    Result<void> Servo::transfer_(unsigned char* txBuf, unsigned long txLen, unsigned char* rxBuf, unsigned long rxLen){
        Result<void> result;
        result.success = 1;

        if(portStream_ == nullptr){
            result.success = 0;
            result.error_msg = "[E]transfer_:portStream_に実体がない。";
            return result;
        }

        if(txLen > 0){
            ::delayMicroseconds(waitTimeUs_);
            if(enPin_ >= 0)digitalWrite(enPin_, HIGH);
            portStream_->write(txBuf, txLen);
            portStream_->flush();
            if(enPin_ >= 0)digitalWrite(enPin_, LOW);
        }
        
        if(rxLen > 0){
            if(enPin_ >= 0)digitalWrite(enPin_, LOW);
            while (portStream_->available() > 0) portStream_->read();
            unsigned long val = portStream_->readBytes(rxBuf, rxLen);
            if (val != rxLen) {
                result.success = 0;
                result.error_msg = "[E]transfer_:受信のデータ数が合わない";
            }
        }

        return result;
    }

    Result<void> Servo::attach(Stream* port, char enPin, uint8_t id) {
        Result<void> result;
        result.success = 1;
        portStream_ = port;
        servoId_ = id;
        enPin_ = enPin;
        if(enPin_ >= 0)pinMode(enPin_, OUTPUT);
        if(enPin_ >= 0)digitalWrite(enPin_, LOW);

        sub.p = this;
        return result;
    }

    Servo::SubGetPos& Servo::setPos(long ics, bool offsetEn){
        static char error_str[160] = "";

        status.success = 1;

        if(ics != 0){
            //オフセット有効時は論理値→生値へ変換
            if(offsetEn) ics = ics + (offSet_ - 7500);

            //異常指令値をはじく（送信しない＝直前の姿勢を保持）
            if(ics > maxIcs_){
                status.success = 0;
                snprintf(error_str, sizeof(error_str), "[E]setPos:上限maxIcs_より指令値icsの方が大きいです。ICS=%ld Deg=%d", ics, (int)fromIcs_toDeg(ics));
                status.error_msg = error_str;
                return sub;
            }else if(ics < minIcs_){
                status.success = 0;
                snprintf(error_str, sizeof(error_str), "[E]setPos:下限minIcs_より指令値icsの方が小さいです。ICS=%ld Deg=%d", ics, (int)fromIcs_toDeg(ics));
                status.error_msg = error_str;
                return sub;
            }
        }

        //スキップ判定は「オフセット適用後の実際に送る値」で行う。
        //旧実装はオフセット適用前の値で比較し、さらに送信失敗/リミット拒否でもprePos_を更新していたため、
        //同じ値を再送しても送られないことがあった。
        if(isSkip_ && (ics == prePos_)){
            status.success = 1;
            status.error_msg = "[I]スキップを実行しました。";
            return sub;
        }

        unsigned char txByte[3];
        txByte[0] = 0b10000000 | (0b00011111 & servoId_);
        txByte[1] = (unsigned char)((ics >> 7) & 0x7F);
        txByte[2] = (unsigned char)(ics & 0x7F);

        status = transfer_(txByte, 3, 0, 0);

        // 送信処理に成功した場合、保持値を更新
        if (status.success) {
            lastSetPos_ = ics;
            prePos_ = ics;
            if (ics != 0) hasSetPos_ = true;
        }

        if (postWaitUs_ > 0) delayMicroseconds(postWaitUs_);

        return sub;
    }

    Result<void> Servo::setSpeed(unsigned char speed){
        Result<void> result;
        result.success = 1;
        unsigned char txByte[3];

        if(speed < 1 || speed > 127){
            result.success = 0;
            result.error_msg = "[E]setSpeed:speed指令値が範囲外(1~127)";
            return result;
        }

        txByte[0] = 0b11000000 | (0b00011111 & servoId_);
        txByte[1] = 0x02;
        txByte[2] = speed;

        result = transfer_(txByte, 3, 0, 0);
        return result;
    }

    Result<void> Servo::setStretch(unsigned char stretch){
        Result<void> result;
        result.success = 1;
        unsigned char txByte[3];

        if(stretch < 1 || stretch > 127){
            result.success = 0;
            result.error_msg = "[E]setStretch:stretch指令値が範囲外(1~127)";
            return result;
        }

        txByte[0] = 0b11000000 | (0b00011111 & servoId_);
        txByte[1] = 0x01;
        txByte[2] = stretch;

        result = transfer_(txByte, 3, 0, 0);
        return result;
    }

    Result<long> Servo::SubGetPos::getPosRecive_(){
        Result<long> result = p->status;
        if(result.success == 0){
            return result;
        }

        unsigned char rxByte[3] = {0, 0, 0};
        result = p->transfer_(0, 0, rxByte, 3);
        result.value = result.success ? (long)(((rxByte[1] & 0x7F) << 7) | (rxByte[2] & 0x7F)) : 0;
        return result;
    }

    Result<long> Servo::getPosCommand_(){
        Result<long> result;

        unsigned char txByte[3];
        txByte[0] = 0b10100000 | (0b00011111 & servoId_);
        txByte[1] = 0x05;

        unsigned char rxByte[4] = {0, 0, 0, 0};
        result = transfer_(txByte, 2, rxByte, 4);
        // 失敗時は未初期化バッファのゴミ値を返さない
        result.value = result.success ? (long)(((rxByte[2] & 0x7F) << 7) | (rxByte[3] & 0x7F)) : 0;
        return result;
    }

    Result<int> Servo::getSpeed(){
        Result<int> result;

        unsigned char txByte[3];
        txByte[0] = 0b10100000 | (0b00011111 & servoId_);
        txByte[1] = 0x02;

        unsigned char rxByte[3];
        result = transfer_(txByte, 2, rxByte, 3);
        result.value = rxByte[2];
        return result;
    }

    Result<int> Servo::getStretch(){
        Result<int> result;
        result.success = 1;

        unsigned char txByte[3];
        txByte[0] = 0b10100000 | (0b00011111 & servoId_);
        txByte[1] = 0x01;

        unsigned char rxByte[3];
        result = transfer_(txByte, 2, rxByte, 3);
        result.value = rxByte[2];
        return result;
    }

    Result<void> Servo::setOffset(long offsetIcs){
        Result<void> result;
        result.success = 1;

        // 旧: (offsetIcs<minIcs_)&&(maxIcs_<offsetIcs) は常にfalseでチェックが効いていなかった
        if((offsetIcs<minIcs_)||(maxIcs_<offsetIcs)){
            result.error_msg = "[E] setOffset:オフセット設定値が設置した角度内にありません";
            result.success = 0;
            return result;
        }

        offSet_ = offsetIcs;
        return result;
    }

    Result<long> Servo::getOffset(){
        Result<long> result;
        result.success = 1;
        result.value = offSet_;
        return result;
    }

    Result<void> Servo::setMSMin(long minIcs){
        Result<void> result;
        result.success = 1;
        minIcs_ = minIcs;
        return result;
    }

    Result<void> Servo::setMSMax(long maxIcs){
        Result<void> result;
        result.success = 1;
        maxIcs_ = maxIcs;
        return result;
    }

    Result<long> Servo::getMSMin(){
        Result<long> result;
        result.success = 1;
        result.value = minIcs_;
        return result;
    }

    Result<long> Servo::getMSMax(){
        Result<long> result;
        result.success = 1;
        result.value = maxIcs_;
        return result;
    }

    void Servo::setSkip(bool isSkip){
        isSkip_ = isSkip;
    }

}