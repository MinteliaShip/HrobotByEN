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
        static char error_str[1024] = ""; 

        if((ics == prePos_) && isSkip_){
            status.success = 1;
            status.error_msg = "[I]スキップを実行しました。";
            return sub;
        }
        prePos_ = ics;

        status.success = 1;

        if(ics != 0){
            //オフセット無効時には弾く
            if(offsetEn) ics = ics + (offSet_ - 7500);

            //異常指令値をはじく
            if(ics > maxIcs_){
                status.success = 0;
                sprintf(error_str,"[E]setPos:上限maxIcs_より指令値icsの方が大きいです。ICS=%d Deg=%d \0",ics,(int)fromIcs_toDeg(ics));
                status.error_msg = error_str;
                return sub;
            }else if((ics < minIcs_)&&(ics != 0)){
                status.success = 0;
                sprintf(error_str,"[E]setPos:下限minIcs_より指令値icsの方が小さいです。ICS=%d Deg=%d \0",ics,(int)fromIcs_toDeg(ics));
                status.error_msg = error_str;
                return sub;
            }
        }

        unsigned char txByte[3];
        txByte[0] = 0b10000000 | (0b00011111 & servoId_);
        txByte[1] = char((ics & 0b00011111110000000) >> 7);
        txByte[2] = char(ics & 0b00000000001111111);

        status = transfer_(txByte, 3, 0, 0);

        // 送信処理に成功した場合、保持値を更新
        if (status.success) {
            lastSetPos_ = ics;
        }

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

        unsigned char rxByte[3];
        result = p->transfer_(0, 0, rxByte, 3);
        result.value = (unsigned long int)(rxByte[1] << 7) | rxByte[2];
        return result;
    }

    Result<long> Servo::getPosCommand_(){
        Result<long> result;

        unsigned char txByte[3];
        txByte[0] = 0b10100000 | (0b00011111 & servoId_);
        txByte[1] = 0x05;

        unsigned char rxByte[4];
        result = transfer_(txByte, 2, rxByte, 4);
        result.value = (unsigned long int)(rxByte[2] << 7) | rxByte[3];
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

        if((offsetIcs<minIcs_)&&(maxIcs_<offsetIcs)){
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