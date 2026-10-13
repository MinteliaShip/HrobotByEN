#ifndef SERVOICS_H
#define SERVOICS_H
// SPDX-License-Identifier: LGPL-2.1-or-later
// Copyright (C) 2025 MinteIiaShip

#include <Arduino.h>
#include <type_traits>
#include <vector> // インスタンスリスト用

// --- 教示（TX）モード用グローバル変数の extern 宣言 ---
extern volatile bool g_teachCmdPending;
extern uint8_t g_teachRxBuffer[64];
extern size_t g_teachRxLength;

namespace servoICS {

    // KRSモータ用定数
    const long ICS_LOW  = 3500;   // ICS最小値
    const long ICS_HIGH = 11500;  // ICS最大値
    const float DEG_LOW  = -135.0;   // ICS対応角度最小値
    const float DEG_HIGH = 135.0;    // ICS対応角度最大値

    const int ICS_MAX_SPEED = 127;
    const int ICS_MIN_SPEED = 1;

    const int ICS_MAX_STRETCH = 127;
    const int ICS_MIN_STRETCH = 1;

    const float myPI = 3.141592653589;

    template<typename servoICS_T>
    servoICS_T servoICS_map(servoICS_T value, servoICS_T fromLow, servoICS_T fromHigh, servoICS_T toLow, servoICS_T toHigh) {
        servoICS_T inRange = fromHigh - fromLow;
        servoICS_T outRange = toHigh - toLow;
        return ((value - fromLow) * outRange) / inRange  + toLow;
    }

    template<typename servoICS_U>
    struct Result {
        bool success;
        const char* error_msg;
        servoICS_U value;

        inline bool s() const { return success; }
        inline const char* m() const { return error_msg; }
        inline servoICS_U v() const { return value; }

        Result() : success(false), error_msg(""), value(servoICS_U{}) {}

        template<typename servoICS_U_2>
        void assign_value(const Result<servoICS_U_2>& other, std::false_type) {
            value = static_cast<servoICS_U>(other.value);
        }

        template<typename servoICS_U_2>
        void assign_value(const Result<servoICS_U_2>& other, std::true_type) {
            value = static_cast<servoICS_U>(0);
        }

        template<typename servoICS_U_2>
        Result(const Result<servoICS_U_2>& other) : success(other.success), error_msg(other.error_msg)
        {
            assign_value(other, std::is_void<servoICS_U_2>());
        }

        operator servoICS_U() const {
            return value;
        }
    };

    template<>
    struct Result<void> {
        bool success;
        const char* error_msg;

        inline bool s() const { return success; }
        inline const char* m() const { return error_msg; }

        Result() : success(false), error_msg("") {}

        template<typename servoICS_Z>
        Result(const Result<servoICS_Z>& other) 
            : success(other.success), 
            error_msg(other.error_msg) {}
    };

    inline float fromIcs_toDeg(long icsPos){
        return servoICS_map((float)icsPos, (float)ICS_LOW, (float)ICS_HIGH, DEG_LOW, DEG_HIGH);
    }

    inline long fromDeg_toIcs(float degPos){
        return (long)servoICS_map((float)degPos, (float)DEG_LOW, (float)DEG_HIGH, (float)ICS_LOW, (float)ICS_HIGH);
    }

    inline float fromDeg_toRad(float degPos){
        return degPos * (myPI / 180.0f);
    }

    inline float fromRad_toDeg(float radPos){
        return radPos * (180.0f / myPI);
    }

    inline float fromRad_toIcs(float radPos){
        return fromDeg_toIcs(fromRad_toDeg(radPos));
    }

    inline float fromIcs_toRad(long icsPos){
        return fromDeg_toRad(fromIcs_toDeg(icsPos));
    }

    inline Result<float> fromIcs_toDeg_Result(Result<long> icsPos){
        Result<float> result;
        result.value = 0.0;
        if(icsPos.success==true)result.value = fromIcs_toDeg(icsPos.value);
        result.success = icsPos.success;
        result.error_msg = icsPos.error_msg;
        return result;
    }

    inline Result<long> fromDeg_toIcs_Result(Result<float> degPos){
        Result<long> result;
        result.value = 0;
        if(degPos.success==true)result.value = fromDeg_toIcs(degPos.value);
        result.success = degPos.success;
        result.error_msg = degPos.error_msg;
        return result;
    }

    inline Result<float> fromDeg_toRad_Result(Result<float> degPos){
        Result<float> result;
        result.value = 0.0;
        if(degPos.success==true)result.value = fromDeg_toRad(degPos.value);
        result.success = degPos.success;
        result.error_msg = degPos.error_msg;
        return result;
    }

    inline Result<float> fromRad_toDeg_Result(Result<float> radPos){
        Result<float> result;
        result.value = 0.0;
        if(radPos.success==true)result.value = fromRad_toDeg(radPos.value);
        result.success = radPos.success;
        result.error_msg = radPos.error_msg;
        return result;
    }

    inline Result<long> fromRad_toIcs_Result(Result<float> radPos){
        Result<long> result;
        result.value = 0;
        if(radPos.success==true)result.value = fromRad_toIcs(radPos.value);
        result.success = radPos.success;
        result.error_msg = radPos.error_msg;
        return result;
    }

    inline Result<float> fromIcs_toRad_Result(Result<long> icsPos){
        Result<float> result;
        result.value = 0.0;
        if(icsPos.success==true)result.value = fromIcs_toRad(icsPos.value);
        result.success = icsPos.success;
        result.error_msg = icsPos.error_msg;
        return result;
    }

    class Servo{
        private:
            //通信設定
            Stream* portStream_ = nullptr;   // 使用するシリアルインスタンス
            char enPin_ = -1;        // 送受切替ピン（不要な場合は-1）
            uint8_t servoId_ = 0;    // サーボID

            //チェーンメソッド用変数
            Result<void> status;

            //スキップ機能有効化フラグ(0:off 1:on)
            bool isSkip_ = 0;
            //前の角度を保持。スキップ判定に使用。
            long prePos_ = 0;

            //送信角度の保持用変数
            long lastSetPos_ = 7500; // 最後に送信されたICS値（初期値: 7500）

            //追加機能パラメータ
            long offSet_ = 7500;    //ソフトウェアオフセット値(ICS値)
            long minIcs_ = ICS_LOW; //ソフトウェアリミット最小
            long maxIcs_ = ICS_HIGH;//ソフトウェアリミット最大
            unsigned long waitTimeUs_ = 1000; //通信を空ける時間(us)

            //インスタンス一括管理用
            static std::vector<Servo*> instances_;
            static void registerInstance_(Servo* instance);
            static void unregisterInstance_(Servo* instance);

            //データの送受信
            Result<void> transfer_(unsigned char* txBuf, unsigned long txLen, unsigned char* rxBuf, unsigned long rxLen);
            Result<long> getPosCommand_();

            class SubGetPos{
                public:
                    Servo* p = nullptr;
                    Result<long> getPosRecive_();
                    Result<long> getPos(){return getPosRecive_();}
                    Result<long> getPosIcs(){return getPos();}
                    Result<float> getPosDeg(){return fromIcs_toDeg_Result(getPos());}
                    Result<float> getPosRad(){return fromIcs_toRad_Result(getPos());}

                    Result<void> returnStatus(){return p->status;}
            };
            
            SubGetPos sub;

        public:

            Servo(Stream* port, char enPin, uint8_t id){
                registerInstance_(this);
                status = attach(port, enPin, id);
            }

            Servo(){
                registerInstance_(this);
            }

            ~Servo(){
                unregisterInstance_(this);
            }

            // インスタンス一覧取得
            static const std::vector<Servo*>& getInstances() {
                return instances_;
            }

            //成功の場合true、失敗の場合falseを返す
            Result<void> attach(Stream* port, char enPin, uint8_t id);

            //setPos (offsetEn 引数を維持)
            SubGetPos& setPos(long ics, bool offsetEn = true);
            SubGetPos& setPosIcs(long ics, bool offsetEn = true){ return setPos(ics, offsetEn); }
            SubGetPos& setPosDeg(float deg, bool offsetEn = true){ return setPos(fromDeg_toIcs(deg), offsetEn); }
            SubGetPos& setPosRad(float rad, bool offsetEn = true){ return setPos(fromRad_toIcs(rad), offsetEn); }
            SubGetPos& setPosFree(){ return setPos(0); }

            // 保持した送信角度の取得機能
            long getLastSetPosIcs() const { return lastSetPos_; }
            float getLastSetPosDeg() const { return fromIcs_toDeg(lastSetPos_); }
            float getLastSetPosRad() const { return fromIcs_toRad(lastSetPos_); }

            //スピードとストレッチのset
            Result<void> setSpeed(unsigned char speed);
            Result<void> setStretch(unsigned char stretch);

            //getPos
            Result<long> getPos(){return getPosCommand_();}
            Result<long> getPosIcs(){return getPos();}
            Result<float> getPosDeg(){return fromIcs_toDeg_Result(getPos());}
            Result<float> getPosRad(){return fromIcs_toRad_Result(getPos());}

            //スピードとストレッチのget
            Result<int> getSpeed();
            Result<int> getStretch();

            //追加機能の設定
            //オフセット
            Result<void> setOffset(long offsetIcs);
            Result<void> setOffsetIcs(long offsetIcs){return setOffset(offsetIcs);}
            Result<void> setOffsetDeg(float offsetDeg){return setOffset(fromDeg_toIcs(offsetDeg));}
            Result<void> setOffsetRad(float offsetRad){return setOffset(fromRad_toIcs(offsetRad));}

            Result<long> getOffset();
            Result<long> getOffsetIcs(){return getOffset();}
            Result<float> getOffsetDeg(){return fromIcs_toDeg_Result(getOffset());}
            Result<float> getOffsetRad(){return fromIcs_toRad_Result(getOffset());}

            //リミット設定
            Result<void> setMSMin(long minIcs);
            Result<void> setMSMinIcs(long minIcs){return setMSMin(minIcs);}
            Result<void> setMSMinDeg(float minDeg){return setMSMin(fromDeg_toIcs(minDeg));}
            Result<void> setMSMinRad(float minRad){return setMSMin(fromRad_toIcs(minRad));}
            
            Result<void> setMSMax(long maxIcs);
            Result<void> setMSMaxIcs(long maxIcs){return setMSMax(maxIcs);}
            Result<void> setMSMaxDeg(float maxDeg){return setMSMax(fromDeg_toIcs(maxDeg));}
            Result<void> setMSMaxRad(float maxRad){return setMSMax(fromRad_toIcs(maxRad));}
            
            Result<long> getMSMin();
            Result<long> getMSMinIcs(){return getMSMin();}
            Result<float> getMSMinDeg(){return fromIcs_toDeg_Result(getMSMin());}
            Result<float> getMSMinRad(){return fromIcs_toRad_Result(getMSMin());}

            Result<long> getMSMax();
            Result<long> getMSMaxIcs(){return getMSMax();}
            Result<float> getMSMaxDeg(){return fromIcs_toDeg_Result(getMSMax());}
            Result<float> getMSMaxRad(){return fromIcs_toRad_Result(getMSMax());}

            //スキップ機能
            void setSkip(bool isSkip);
            bool getSkip(){return isSkip_;}

            //サーボIDの取得
            uint8_t getId() const { return servoId_; }

            Result<void> returnStatus()const{return status;}
    };
}

#endif