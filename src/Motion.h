#ifndef MOTION_H_
#define MOTION_H_

#include <Arduino.h>
#include "Config.h"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "FootController.h"
#include "MotionData.h"    // 手作りの旧モーション（生ICS）
#include "ExportData.h"    // PCアプリがエクスポートしたモーション（論理ICS）と校正値

// ==============================================================
// Motion.cpp が提供する、教示・事前転送再生・モーション管理の関数（RobotController / main から使う）
// ==============================================================
// ---- モーション管理 ----
// 実行中の全モーションを止め、関数内static状態を次回起動時に初期化させる
void motionForceStopAll();

// ---- 教示(TEACH) ----
// BLE callbackから呼ぶ（最新値優先のメールボックスへ置くだけ）。payloadは [type][seq] の後ろ。
bool teachPostAxis(const uint8_t* payload, size_t len);
bool teachPostPose(const uint8_t* payload, size_t len);
// loopから呼ぶ
void teachEnter();
void teachExit();
void teachUpdate();
bool teachNeutral(uint8_t foot);   // ニュートラル座標を適用し POSE_ECHO を送る
uint32_t teachErrorMask();         // IK到達不可/範囲外などのエラー軸

// ---- 事前転送モーション ----
uint8_t uploadBegin(uint16_t frames);                              // callback（status）
uint8_t uploadData(uint16_t offset, const uint8_t* d, size_t n);   // callback（status）
uint8_t uploadCommit();     // loop: 全フレーム検証(IK含む)。RAMに保持するだけでフラッシュには書かない（1件のみ）
uint16_t uploadHash16();    // 直近のCOMMIT内容の16bitハッシュ（ACKのtokenに入れる）
uint8_t playbackStart();    // loop: RAMの内容を検査して再生準備
bool playbackUpdate();      // loop: 再生中はtrue
void playbackAbort();
bool uploadReady();

class motionBase;

namespace motionList {
    inline motionBase* motionArr[128];
    inline int motionArrNum = 0;
}

class motionBase {
public:
    explicit motionBase(char motionArrUsingBit_ = 0) {
        motionArrUsingBit = motionArrUsingBit_;
        if (motionList::motionArrNum < 128) {
            motionList::motionArr[motionList::motionArrNum] = this;
            motionList::motionArrNum++;
        }
    }

    bool isActive = false;

    virtual bool canStart() = 0;
    virtual bool motionLoop() = 0;
    // 状態遷移などで強制中断されたときに呼ばれる（メンバで状態を持つモーション用）
    virtual void onForceStop() {}
    char motionArrUsingBit = 0;
};

// モーションデータ逐次再生関数の前方宣言
//  logical=false: MotionData.h（旧来の生ICS。旧オフセットで論理値へ変換して出す）
//  logical=true : ExportData.h（PCアプリが書き出した論理ICS。そのまま出す）
bool playCodeMotion(const int16_t motionData[][19], uint16_t totalFrames, bool logical, char motionArrUsingBit, int &taskPhase, uint16_t &currentFrame);

// MotionData系モーションの共通基底クラス
//  DECLARE_DATA_MOTION の第2引数（モーション名の namespace）が MotionData.h のものでも ExportData.h のものでも使える。
//  （配列の型で見分ける: int16_t[][19] = 旧来の生ICS / ExportFrame[] = 論理ICS）
class motionDataBase : public motionBase {
protected:
    const int16_t (*motionData)[19];
    uint16_t totalFrames;
    bool logical;
    int taskPhase = 0;
    uint16_t currentFrame = 0;

public:
    motionDataBase(const int16_t motionData[][19], uint16_t totalFrames, char usingBit = ALL_BIT)
        : motionBase(usingBit), motionData(motionData), totalFrames(totalFrames), logical(false) {}
    motionDataBase(const ExportFrame* frames, uint16_t totalFrames, char usingBit = ALL_BIT)
        : motionBase(usingBit), motionData(reinterpret_cast<const int16_t (*)[19]>(frames)), totalFrames(totalFrames), logical(true) {}

    bool motionLoop() override {
        return playCodeMotion(
            motionData,
            totalFrames,
            logical,
            this->motionArrUsingBit,
            taskPhase,
            currentFrame
        );
    }

    void resetState() {
        taskPhase = 0;
        currentFrame = 0;
    }
    void onForceStop() override { resetState(); }
};

#define DECLARE_MOTION(ClassName, UsingBit) \
    class ClassName : public motionBase { \
    public: \
        ClassName(char bit = UsingBit) : motionBase(bit) {} \
        bool canStart() override; \
        bool motionLoop() override; \
    }; \
    inline ClassName ClassName##_inst;

#define DECLARE_DATA_MOTION(ClassName, TargetNamespace, UsingBit) \
    class ClassName : public motionDataBase { \
    public: \
        ClassName() : motionDataBase(TargetNamespace::flameData, TargetNamespace::flameNum, UsingBit) {} \
        bool canStart() override; \
    }; \
    inline ClassName ClassName##_inst;

namespace motion {
    DECLARE_MOTION(walk1, FOOT_BIT)
    DECLARE_MOTION(walkBack, FOOT_BIT)
    DECLARE_MOTION(walkY, FOOT_BIT)
    DECLARE_MOTION(turn, FOOT_BIT | HIP_BIT)

    DECLARE_MOTION(attack_Light_left, LEFT_ARM_BIT)
    DECLARE_MOTION(attack_Light_right, RIGHT_ARM_BIT)
    DECLARE_MOTION(attack_Medium_left, LEFT_ARM_BIT)
    DECLARE_MOTION(attack_Medium_right, RIGHT_ARM_BIT)
    
    // MotionData指定による簡略化宣言
    DECLARE_DATA_MOTION(attack_Heavy_1, Attack_left_flont, ALL_BIT)
    DECLARE_MOTION(attack_Heavy_2, ALL_BIT)

    DECLARE_MOTION(taunt, ALL_BIT)
    DECLARE_MOTION(nop, ALL_BIT)
    DECLARE_MOTION(LOCK_DebugMode, ALL_BIT)
    DECLARE_MOTION(pose, ALL_BIT)
    DECLARE_MOTION(chair, ALL_BIT)
    DECLARE_MOTION(kneeling, ALL_BIT)

    DECLARE_MOTION(getUp, ALL_BIT)
    DECLARE_MOTION(getUp_supine, ALL_BIT)
    DECLARE_MOTION(getUp_prone, ALL_BIT)

    DECLARE_MOTION(sit_stand, ALL_BIT)
    DECLARE_MOTION(sitDown, ALL_BIT)
    DECLARE_MOTION(standUp, ALL_BIT)
    DECLARE_MOTION(lowerPos, ALL_BIT)
    DECLARE_MOTION(hip, HIP_BIT)

    DECLARE_MOTION(jump, ALL_BIT)
} // namespace motion

#endif // MOTION_H_