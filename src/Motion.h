#ifndef MOTION_H_
#define MOTION_H_

#include <Arduino.h>
#include "AssistiveProgram.h"

class motionBase;

namespace motionList {
    // インスタンス化されたポインタをlistに収容（※必ずポインタ配列 motionBase* にする）
    inline motionBase* motionArr[128];
    inline int motionArrNum = 0;
}

// モーションの基底クラス
class motionBase {
public:
    explicit motionBase(char motionArrUsingBit_ = 0) {
        motionArrUsingBit = motionArrUsingBit_;
        
        // 配列外アクセス（オーバーフロー）を防ぐための安全処理
        if (motionList::motionArrNum < 128) {
            motionList::motionArr[motionList::motionArrNum] = this;
            motionList::motionArrNum++;
        }
    }

    bool isActive = false; // 起動中フラグ

    virtual bool canStart() = 0;   // 実行条件の判定（および開始時リセット）
    virtual bool motionLoop() = 0; // ループ処理（継続: true / 終了: false）
    char motionArrUsingBit = 0;
};

// --- クラス宣言 ＋ インスタンス化を行うマクロ ---
#define DECLARE_MOTION(ClassName, UsingBit) \
    class ClassName : public motionBase { \
    public: \
        ClassName(char bit = UsingBit) : motionBase(bit) {} \
        bool canStart() override; \
        bool motionLoop() override; \
    }; \
    /* 引数なしで宣言することで、コンストラクタのデフォルト引数 (UsingBit) が適用される */ \
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
    DECLARE_MOTION(attack_Heavy_1, ALL_BIT)
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