#ifndef MOTION_H_
#define MOTION_H_

#include <Arduino.h>
#include "AssistiveProgram.h"
#include "Config.h"
#include "Declaration.h"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "FootController.h"
#include "MotionData.h"
#include <LittleFS.h>

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
    char motionArrUsingBit = 0;
};

// モーションデータ逐次再生関数の前方宣言
bool playCodeMotion(const int16_t motionData[][19], uint16_t totalFrames, char motionArrUsingBit, int &taskPhase, uint16_t &currentFrame);

// MotionData系モーションの共通基底クラス
class motionDataBase : public motionBase {
protected:
    const int16_t (*motionData)[19];
    uint16_t totalFrames;
    int taskPhase = 0;
    uint16_t currentFrame = 0;

public:
    motionDataBase(const int16_t motionData[][19], uint16_t totalFrames, char usingBit = ALL_BIT)
        : motionBase(usingBit), motionData(motionData), totalFrames(totalFrames) {}

    bool motionLoop() override {
        return playCodeMotion(
            motionData,
            totalFrames,
            this->motionArrUsingBit,
            taskPhase,
            currentFrame
        );
    }

    void resetState() {
        taskPhase = 0;
        currentFrame = 0;
    }
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
    DECLARE_MOTION(TeachingMode, ALL_BIT)  // ★ 教示モード（全軸占有）を追加
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
    DECLARE_MOTION(ServoStateDeBug, NON_BIT)
} // namespace motion

#endif // MOTION_H_