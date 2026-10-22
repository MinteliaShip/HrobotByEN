#include "Motion.h"
#include <cmath>

extern volatile bool deviceConnected;
extern void sendBLE(uint8_t* data, size_t length);

// ==============================================================
// 教示モード（TX / 同期モード）
//  - BLE callbackは「最新値優先のメールボックス」へ置くだけ（サーボUARTには触れない）。
//  - 実処理は TeachingMode::motionLoop() が main loop から毎周期呼ばれて行う。
//  - 軸指令(0x01)は未処理分をマージ（軸ごとに最新値）。座標指令(0x40)は脚ごとに最新値。
// ==============================================================
volatile bool g_teachCmdPending = false;   // 未処理の軸/座標指令がある（main.cpp互換）
volatile bool g_teachModeActive = false;
static portMUX_TYPE g_teachCmdMux = portMUX_INITIALIZER_UNLOCKED;

namespace {
constexpr uint32_t TEACH_ALL_MASK = (1UL << 19) - 1UL;

struct TeachAxisCmd {
    uint32_t cmdMask;
    uint32_t fbMask;
    uint16_t ics[19];
};
struct TeachPoseCmd {
    float v[6];     // X, Y, Z, Roll[rad], Pitch[rad], Yaw[rad]
    uint8_t mode;   // IK解の選択(0/1)
};

TeachAxisCmd g_axisPending = {};
bool g_axisPendingValid = false;
TeachPoseCmd g_posePending[2] = {};
bool g_posePendingValid[2] = {false, false};
bool g_poseNeutralReq[2] = {false, false};
bool g_teachNeedReset = true;
}

bool queueTeachingCommand(const uint8_t* data, size_t length) {
    if (data == nullptr || length != 45 || data[0] != 0x01) return false;
    const uint32_t cmd = (static_cast<uint32_t>(data[1]) | (static_cast<uint32_t>(data[2]) << 8) |
                          (static_cast<uint32_t>(data[3]) << 16)) & TEACH_ALL_MASK;
    const uint32_t fb = (static_cast<uint32_t>(data[4]) | (static_cast<uint32_t>(data[5]) << 8) |
                         (static_cast<uint32_t>(data[6]) << 16)) & TEACH_ALL_MASK;

    bool accepted = false;
    portENTER_CRITICAL(&g_teachCmdMux);
    // TX(同期)モードでない時は受け付けない。RXへ戻した後に遅れて届いたフレームで
    // 勝手にTXへ戻ってしまう不具合の修正。
    if (g_teachModeActive) {
        if (!g_axisPendingValid) {
            g_axisPending.cmdMask = 0;
            g_axisPendingValid = true;
        }
        for (int i = 0; i < 19; ++i) {
            if (cmd & (1UL << i)) {
                g_axisPending.ics[i] = static_cast<uint16_t>(data[7 + i * 2]) |
                                       (static_cast<uint16_t>(data[8 + i * 2]) << 8);
            }
        }
        g_axisPending.cmdMask |= cmd;   // 未処理の軸指令は失わずマージする
        g_axisPending.fbMask = fb;      // フィードバック要求は最新値
        g_teachCmdPending = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&g_teachCmdMux);
    return accepted;
}

// 座標指令: [0x40][foot:0=左,1=右][mode][X,Y,Z,Roll,Pitch,Yaw : float32 LE x6] = 27B
bool queueTeachingPose(const uint8_t* data, size_t length) {
    if (data == nullptr || length != 27 || data[0] != 0x40) return false;
    const uint8_t foot = data[1];
    if (foot > 1) return false;
    TeachPoseCmd cmd;
    cmd.mode = data[2] ? 1 : 0;
    memcpy(cmd.v, data + 3, sizeof(cmd.v));
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(cmd.v[i])) return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (fabsf(cmd.v[i]) > 400.0f) return false;      // [mm]
    }
    if (fabsf(cmd.v[3]) > 1.6f || fabsf(cmd.v[5]) > 1.6f) return false; // [rad]
    cmd.v[4] = 0.0f; // Pitchは現行IKでは未使用

    bool accepted = false;
    portENTER_CRITICAL(&g_teachCmdMux);
    if (g_teachModeActive) {
        g_posePending[foot] = cmd;
        g_posePendingValid[foot] = true;
        g_teachCmdPending = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&g_teachCmdMux);
    return accepted;
}

// ニュートラル座標要求: [0x41][foot]。適用後に [0x37][foot][6 floats] で適用座標を返す。
bool queueTeachingNeutral(uint8_t foot) {
    if (foot > 1) return false;
    bool accepted = false;
    portENTER_CRITICAL(&g_teachCmdMux);
    if (g_teachModeActive) {
        g_poseNeutralReq[foot] = true;
        g_teachCmdPending = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&g_teachCmdMux);
    return accepted;
}

void teachingRequestReset() {
    portENTER_CRITICAL(&g_teachCmdMux);
    g_teachNeedReset = true;
    portEXIT_CRITICAL(&g_teachCmdMux);
}

void startTeachingMode() {
    portENTER_CRITICAL(&g_teachCmdMux);
    g_teachModeActive = true;
    g_axisPendingValid = false;
    g_posePendingValid[0] = g_posePendingValid[1] = false;
    g_poseNeutralReq[0] = g_poseNeutralReq[1] = false;
    g_teachCmdPending = false;
    g_teachNeedReset = true;
    portEXIT_CRITICAL(&g_teachCmdMux);
}

void stopTeachingMode() {
    portENTER_CRITICAL(&g_teachCmdMux);
    g_teachModeActive = false;
    g_axisPendingValid = false;
    g_posePendingValid[0] = g_posePendingValid[1] = false;
    g_poseNeutralReq[0] = g_poseNeutralReq[1] = false;
    g_teachCmdPending = false;
    g_teachNeedReset = true;
    portEXIT_CRITICAL(&g_teachCmdMux);
}

// サーボID(0〜18)に対応する部位ビットのルックアップテーブル
static const BusyPartsBit SERVO_PART_MAP[19] = {
    HIP_BIT,                                                     // サーボ 0   : 腰
    LEFT_ARM_BIT,  LEFT_ARM_BIT,  LEFT_ARM_BIT,  LEFT_ARM_BIT,   // サーボ 1..4 : 左腕
    RIGHT_ARM_BIT, RIGHT_ARM_BIT, RIGHT_ARM_BIT, RIGHT_ARM_BIT,  // サーボ 5..8 : 右腕
    LEFT_FOOT_BIT, LEFT_FOOT_BIT, LEFT_FOOT_BIT, LEFT_FOOT_BIT, LEFT_FOOT_BIT,     // サーボ 9..13  : 左足
    RIGHT_FOOT_BIT, RIGHT_FOOT_BIT, RIGHT_FOOT_BIT, RIGHT_FOOT_BIT, RIGHT_FOOT_BIT // サーボ 14..18 : 右足
};

/// PROGMEM上のモーション配列を逐次再生する共通関数
/// @return true: 再生中, false: モーション終了
bool playCodeMotion(const int16_t motionData[][19], uint16_t totalFrames, char motionArrUsingBit, int &taskPhase, uint16_t &currentFrame) {
    static uint32_t lastFrameMs = 0;

    switch (taskPhase) {
        case 0: // 初期化フェーズ
        {
            currentFrame = 0;
            lastFrameMs = 0;

            if (totalFrames == 0) {
                taskPhase = 0;
                return false;
            }

            taskPhase = 1;
            break;
        }

        case 1: // 逐次フレーム再生フェーズ
        {
            if (currentFrame >= totalFrames) {
                taskPhase = 2; // 全フレームの再生完了
                break;
            }

            // G_fps に基づく1フレームあたりのインターバル(ms)を計算 (例: 40FPSなら 25ms)
            uint32_t frameIntervalMs = 1000 / G_fps;
            uint32_t now = millis();

            // 指定時間が経過するまで次のフレームに進めない
            if (currentFrame > 0 && (now - lastFrameMs < frameIntervalMs)) {
                break; // 時間未到達のため今回はスキップ
            }

            lastFrameMs = now;

            // PROGMEMから現在の1フレーム（19軸分）データを読み出し
            int16_t currentPos[19];
            memcpy_P(currentPos, motionData[currentFrame], sizeof(currentPos));

            // 自クラスの所有権限（motionArrUsingBit）に基づく有効サーボマスクを作成
            uint32_t myAuthorityMask = 0;
            for (int i = 0; i < 19; i++) {
                if (motionArrUsingBit & SERVO_PART_MAP[i]) {
                    myAuthorityMask |= (1UL << i);
                }
            }

            // 出力指示
            for (int i = 0; i < 19; i++) {
                // 1. 自クラスが所有する部位であること
                // 2. モーションデータ側が -1 (無視) でないこと
                if ((myAuthorityMask & (1UL << i)) && currentPos[i] >= 0) {
                    if (ServoArray[i] != nullptr) {
                        ServoArray[i]->setPosIcs(static_cast<uint16_t>(currentPos[i]), false);
                    }
                }
            }

            currentFrame++;
            break;
        }

        case 2: // クリーンアップ・終了フェーズ
            taskPhase = 0;
            currentFrame = 0;
            return false; // モーション完了
    }

    return true; // 再生継続中
}
// ==============================================================
// BLE事前転送モーション。1フレーム41byte: duration(uint16 LE), mode(uint8), ICS[19](int16 LE)
// mode 0=なめらか / 1=イチギメ。角度 -1=無視、0=FREE、それ以外=ICS指令。
// ==============================================================
static constexpr uint8_t UPLOAD_BEGIN = 0x30;
static constexpr uint8_t UPLOAD_DATA = 0x31;
static constexpr uint8_t UPLOAD_COMMIT = 0x32;
static constexpr uint8_t UPLOAD_START = 0x33;
static constexpr uint8_t UPLOAD_EMERGENCY_STOP = 0x34;
static constexpr size_t UPLOAD_FRAME_BYTES = 41;
static constexpr uint16_t UPLOAD_MAX_FRAMES = 100;
static constexpr size_t UPLOAD_MAX_BYTES = UPLOAD_FRAME_BYTES * UPLOAD_MAX_FRAMES;
static constexpr char UPLOAD_MOTION_PATH[] = "/uploaded_motion.bin";
static constexpr uint32_t ALL_SERVO_MASK = (1UL << 19) - 1UL;

static uint8_t g_uploadedMotionBytes[UPLOAD_MAX_BYTES];
static uint16_t g_uploadedFrameCount = 0;
static size_t g_uploadedExpectedBytes = 0;
static size_t g_uploadedReceivedBytes = 0;
static volatile bool g_uploadedMotionReady = false;
static volatile bool g_uploadedMotionActive = false;
static volatile bool g_uploadedEmergencyPending = false;
static volatile bool g_uploadedEmergencyLatched = false;
static volatile uint32_t g_uploadedEmergencyServoMask = ALL_SERVO_MASK;
static volatile uint32_t g_uploadedMotionGeneration = 0;
static portMUX_TYPE g_uploadedMotionMux = portMUX_INITIALIZER_UNLOCKED;

bool handleUploadedMotionCommand(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0) return false;
    const uint8_t command = data[0];

    if (command == UPLOAD_BEGIN) {
        if (length != 3) return false;
        const uint16_t count = static_cast<uint16_t>(data[1]) |
                               (static_cast<uint16_t>(data[2]) << 8);
        if (count == 0 || count > UPLOAD_MAX_FRAMES) return false;
        bool accepted = false;
        portENTER_CRITICAL(&g_uploadedMotionMux);
        if (!g_uploadedMotionActive && !g_uploadedEmergencyPending) {
            g_uploadedFrameCount = count;
            g_uploadedExpectedBytes = static_cast<size_t>(count) * UPLOAD_FRAME_BYTES;
            g_uploadedReceivedBytes = 0;
            g_uploadedMotionReady = false;
            accepted = true;
        }
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        return accepted;
    }

    if (command == UPLOAD_DATA) {
        if (length < 4 || length > 20) return false;
        const size_t offset = static_cast<size_t>(data[1]) |
                              (static_cast<size_t>(data[2]) << 8);
        const size_t count = length - 3;
        bool accepted = false;
        portENTER_CRITICAL(&g_uploadedMotionMux);
        if (!g_uploadedMotionActive && !g_uploadedMotionReady &&
            g_uploadedExpectedBytes > 0 && offset == g_uploadedReceivedBytes &&
            offset + count <= g_uploadedExpectedBytes) {
            memcpy(g_uploadedMotionBytes + offset, data + 3, count);
            g_uploadedReceivedBytes += count;
            accepted = true;
        }
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        return accepted;
    }

    if (command == UPLOAD_COMMIT) {
        if (length != 1) return false;
        bool complete = false;
        uint16_t framesToValidate = 0;
        size_t expectedBytesSnapshot = 0;
        portENTER_CRITICAL(&g_uploadedMotionMux);
        complete = !g_uploadedMotionActive && !g_uploadedEmergencyPending &&
                   g_uploadedExpectedBytes > 0 &&
                   g_uploadedReceivedBytes == g_uploadedExpectedBytes;
        if (complete) {
            framesToValidate = g_uploadedFrameCount;
            expectedBytesSnapshot = g_uploadedExpectedBytes;
        }
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        if (!complete) return false;

        // COMMIT時に全フレームを検証し、不正値をサーボへ渡さない。
        bool valid = true;
        for (uint16_t f = 0; f < framesToValidate && valid; ++f) {
            const size_t base = static_cast<size_t>(f) * UPLOAD_FRAME_BYTES;
            const uint16_t duration = static_cast<uint16_t>(g_uploadedMotionBytes[base]) |
                (static_cast<uint16_t>(g_uploadedMotionBytes[base + 1]) << 8);
            const uint8_t mode = g_uploadedMotionBytes[base + 2];
            if (duration < 20 || mode > 1) {
                valid = false;
                break;
            }
            for (int i = 0; i < 19; ++i) {
                const size_t offset = base + 3 + static_cast<size_t>(i) * 2;
                const int16_t target = static_cast<int16_t>(
                    static_cast<uint16_t>(g_uploadedMotionBytes[offset]) |
                    (static_cast<uint16_t>(g_uploadedMotionBytes[offset + 1]) << 8));
                if (target < -1 || (target > 0 && target < 3500) || target > 11500) {
                    valid = false;
                    break;
                }
            }
        }

        // COMMIT前に不正値を検査し、検証済みデータをLittleFSへ永続化する。
        // このハンドラはBLE受信コールバックから呼ばれるため、ここではサーボUARTへ触れない。
        bool saved = false;
        if (valid) {
            File out = LittleFS.open(UPLOAD_MOTION_PATH, "w");
            if (out) {
                const size_t written = out.write(g_uploadedMotionBytes, expectedBytesSnapshot);
                out.close();
                saved = (written == expectedBytesSnapshot);
            }
        }

        bool accepted = false;
        portENTER_CRITICAL(&g_uploadedMotionMux);
        if (!g_uploadedMotionActive && !g_uploadedEmergencyPending &&
            g_uploadedExpectedBytes == expectedBytesSnapshot &&
            g_uploadedFrameCount == framesToValidate &&
            g_uploadedExpectedBytes > 0 &&
            g_uploadedReceivedBytes == g_uploadedExpectedBytes) {
            if (valid && saved) {
                g_uploadedMotionReady = true;
                accepted = true;
            } else {
                g_uploadedMotionReady = false;
                g_uploadedExpectedBytes = 0;
                g_uploadedReceivedBytes = 0;
                g_uploadedFrameCount = 0;
            }
        }
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        if (valid && !saved) {
            Serial.println("[MOTION] LittleFS save failed; playback was not armed.");
        }
        return accepted;
    }

    if (command == UPLOAD_START) {
        if (length != 1) return false;
        size_t expectedBytes = 0;
        uint16_t expectedFrames = 0;
        bool canStart = false;
        portENTER_CRITICAL(&g_uploadedMotionMux);
        canStart = g_uploadedMotionReady && !g_uploadedMotionActive &&
                   !g_uploadedEmergencyPending && g_uploadedFrameCount > 0 &&
                   g_uploadedReceivedBytes == g_uploadedExpectedBytes;
        expectedBytes = g_uploadedExpectedBytes;
        expectedFrames = g_uploadedFrameCount;
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        if (!canStart || expectedBytes == 0 || expectedBytes > UPLOAD_MAX_BYTES) return false;

        // 実行データをファイルシステムから読み直す。RAM上の転送バッファをそのまま
        // 再生するのではなく、保存に成功したファイルが完全であることを確認して開始する。
        File in = LittleFS.open(UPLOAD_MOTION_PATH, "r");
        if (!in || in.size() != expectedBytes) {
            if (in) in.close();
            return false;
        }
        const size_t readBytes = in.read(g_uploadedMotionBytes, expectedBytes);
        in.close();
        if (readBytes != expectedBytes) return false;

        bool accepted = false;
        portENTER_CRITICAL(&g_uploadedMotionMux);
        if (g_uploadedMotionReady && !g_uploadedMotionActive &&
            !g_uploadedEmergencyPending && g_uploadedFrameCount == expectedFrames &&
            g_uploadedExpectedBytes == expectedBytes &&
            g_uploadedReceivedBytes == expectedBytes) {
            g_uploadedEmergencyLatched = false;
            g_uploadedEmergencyPending = false;
            g_uploadedMotionReady = false;
            g_uploadedMotionActive = true;
            ++g_uploadedMotionGeneration;
            accepted = true;
        }
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        if (accepted) {
            teachingRequestReset(); // 再生でサーボ位置が変わるため、教示側の目標値を破棄
            Serial.printf("[MOTION] Loaded %u frames from LittleFS and started playback.\n", expectedFrames);
        }
        return accepted;
    }

    if (command == UPLOAD_EMERGENCY_STOP) {
        // 新形式は [0x34][ServoMask:3B]。旧形式1byteは全軸FREEとして互換維持。
        if (length != 1 && length != 4) return false;
        uint32_t servoMask = ALL_SERVO_MASK;
        if (length == 4) {
            servoMask = static_cast<uint32_t>(data[1]) |
                        (static_cast<uint32_t>(data[2]) << 8) |
                        (static_cast<uint32_t>(data[3]) << 16);
            servoMask &= ALL_SERVO_MASK;
        }
        portENTER_CRITICAL(&g_uploadedMotionMux);
        g_uploadedEmergencyServoMask = servoMask;
        g_uploadedEmergencyPending = true;
        g_uploadedMotionReady = false;
        g_uploadedExpectedBytes = 0;
        g_uploadedReceivedBytes = 0;
        g_uploadedFrameCount = 0;
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        return true;
    }
    return false;
}

bool uploadedMotionOwnsRobot() {
    portENTER_CRITICAL(&g_uploadedMotionMux);
    const bool result = g_uploadedMotionActive || g_uploadedEmergencyPending;
    portEXIT_CRITICAL(&g_uploadedMotionMux);
    return result;
}

bool uploadedMotionEmergencyLatched() {
    portENTER_CRITICAL(&g_uploadedMotionMux);
    const bool result = g_uploadedEmergencyLatched;
    portEXIT_CRITICAL(&g_uploadedMotionMux);
    return result;
}

static void freeServosForEmergency(uint32_t servoMask) {
    for (int i = 0; i < 19; ++i) {
        if ((servoMask & (1UL << i)) == 0) continue;
        if (ServoArray[i] != nullptr) ServoArray[i]->setPosFree();
    }
}

static void sendUploadedMotionEvent(uint8_t status) {
    // 0x36: [status] 0=完了 / 1=緊急停止 / 2=BLE切断停止
    uint8_t eventPacket[2] = {0x36, status};
    sendBLE(eventPacket, sizeof(eventPacket));
}

void processUploadedMotion() {
    bool emergency = false;
    bool active = false;
    uint32_t generation = 0;
    uint32_t emergencyServoMask = ALL_SERVO_MASK;
    uint16_t frameCount = 0;
    portENTER_CRITICAL(&g_uploadedMotionMux);
    if (g_uploadedEmergencyPending) {
        g_uploadedEmergencyPending = false;
        g_uploadedMotionActive = false;
        g_uploadedEmergencyLatched = true;
        emergencyServoMask = g_uploadedEmergencyServoMask;
        emergency = true;
    }
    active = g_uploadedMotionActive;
    generation = g_uploadedMotionGeneration;
    frameCount = g_uploadedFrameCount;
    portEXIT_CRITICAL(&g_uploadedMotionMux);

    if (emergency) {
        stopTeachingMode();
        freeServosForEmergency(emergencyServoMask);
        Serial.println("[MOTION] EMERGENCY STOP: playback aborted; selected enabled servos freed.");
        sendUploadedMotionEvent(1);
        return;
    }
    if (!active) return;

    if (!deviceConnected) {
        portENTER_CRITICAL(&g_uploadedMotionMux);
        g_uploadedMotionActive = false;
        g_uploadedEmergencyLatched = true;
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        stopTeachingMode();
        freeServosForEmergency(ALL_SERVO_MASK);
        Serial.println("[MOTION] BLE disconnected during playback: stopped and freed all servos.");
        sendUploadedMotionEvent(2);
        return;
    }

    static uint32_t localGeneration = 0;
    static uint16_t frameIndex = 0;
    static bool frameInitialized = false;
    static uint32_t frameStartMs = 0;
    static int32_t startIcs[19] = {0};

    if (localGeneration != generation) {
        localGeneration = generation;
        frameIndex = 0;
        frameInitialized = false;
    }
    if (frameIndex >= frameCount) {
        portENTER_CRITICAL(&g_uploadedMotionMux);
        g_uploadedMotionActive = false;
        portEXIT_CRITICAL(&g_uploadedMotionMux);
        Serial.println("[MOTION] Uploaded motion completed.");
        frameInitialized = false;
        sendUploadedMotionEvent(0);
        return;
    }

    const uint8_t* frame = g_uploadedMotionBytes + (static_cast<size_t>(frameIndex) * UPLOAD_FRAME_BYTES);
    uint16_t duration = static_cast<uint16_t>(frame[0]) |
                        (static_cast<uint16_t>(frame[1]) << 8);
    if (duration < 20) duration = 20;
    const bool snap = frame[2] == 1;

    if (!frameInitialized) {
        frameStartMs = millis();
        for (int i = 0; i < 19; ++i) {
            const size_t offset = 3 + (static_cast<size_t>(i) * 2);
            const int16_t target = static_cast<int16_t>(static_cast<uint16_t>(frame[offset]) |
                                     (static_cast<uint16_t>(frame[offset + 1]) << 8));
            startIcs[i] = (ServoArray[i] != nullptr) ? ServoArray[i]->getLastSetPosIcs() : 0;
            if (ServoArray[i] == nullptr || target == -1) continue;
            if (target == 0) {
                ServoArray[i]->setPosFree();
            } else if (snap) {
                ServoArray[i]->setPosIcs(static_cast<uint16_t>(target), false);
            }
        }
        frameInitialized = true;
    }

    const uint32_t elapsed = millis() - frameStartMs;
    const float t = (elapsed >= duration) ? 1.0f : (static_cast<float>(elapsed) / static_cast<float>(duration));
    const float eased = t * t * (3.0f - 2.0f * t);
    if (!snap) {
        for (int i = 0; i < 19; ++i) {
            if (ServoArray[i] == nullptr) continue;
            const size_t offset = 3 + (static_cast<size_t>(i) * 2);
            const int16_t target = static_cast<int16_t>(static_cast<uint16_t>(frame[offset]) |
                                     (static_cast<uint16_t>(frame[offset + 1]) << 8));
            if (target <= 0) continue; // -1=ignore, 0=FREE (frame entryで処理済み)
            int32_t start = startIcs[i];
            if (start <= 0 || start == 0xFFFF) start = target;
            const int32_t pos = static_cast<int32_t>(start + ((static_cast<int32_t>(target) - start) * eased));
            ServoArray[i]->setPosIcs(static_cast<uint16_t>(pos), false);
        }
    }

    if (elapsed >= duration) {
        if (!snap) {
            for (int i = 0; i < 19; ++i) {
                if (ServoArray[i] == nullptr) continue;
                const size_t offset = 3 + (static_cast<size_t>(i) * 2);
                const int16_t target = static_cast<int16_t>(static_cast<uint16_t>(frame[offset]) |
                                         (static_cast<uint16_t>(frame[offset + 1]) << 8));
                if (target > 0) ServoArray[i]->setPosIcs(static_cast<uint16_t>(target), false);
            }
        }
        ++frameIndex;
        frameInitialized = false;
    }
}

bool motion_sub(int totalSteps,int index,int joint_num,int &currentStep,int &taskPhase,const float *armAngle_zero,const float *armAngle_tar1);

void VIB(){//バイブレーション＆赤LED
    Dualshock4.setLed(255, 10, 10);
    Dualshock4.setRumble(255, 0);
    Dualshock4.sendToController();
    resetRumble(300);
}

float phaseShift_f(float inStep,float phaseShift){
    float result = 0;
    if(phaseShift>inStep){
        result = inStep + phaseShift;
    }else{
        result = inStep - phaseShift;
    }
    return result;
}

bool startWalking(GaitParameters &param_p){//静止状態から歩行状態への移行
// 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T * 1.5f;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    float Spac = param_p.Spac;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = -15.0f;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = -15.0f;

    float kickX_left_val = param_p.kickX_left * 0.8f;
    float kickY_left_val = param_p.kickY_left * 0.8f;
    float kickX_right_val = param_p.kickX_right * 0.8f;
    float kickY_right_val = param_p.kickY_right * 0.8f;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    switch (taskPhase){
        case 0:
        {
            taskPhase = 1;
            break;
        }
        case 1:
        {
            // 経過時間 ts の計算 (秒)
            float ts_left = (float)frame / Fps;
            // 右足は位相を半周期 (T / 2.0) ずらす
            float ts_right = phaseShift_f(ts_left,T/2.0);

            float Wd = map(frame,0,totalFrames,0,(int)param_p.Wd);//推移していく。
            float h = map(frame,0,totalFrames,0,(int)param_p.h);//推移していく。

            // 軌道生成処理 (FootController.cpp の tread 関数を利用)
            Vector2 leftPosXY = leftFoot.tread(h,Wd,DutyX,DutyY,T,ts_left);
            Vector2 rightPosXY = rightFoot.tread(h,Wd,DutyX,DutyY,T,ts_right);

            float kick_x_left = 0.0f;
            float kick_y_left = 0.0f;
            float kick_x_right = 0.0f;
            float kick_y_right = 0.0f;

            // 接地期の終盤で後ろ・下へ押し込む
            float support_end = DutyX * T / 2.0f;

            if (ts_left > (support_end - push_window) && ts_left < support_end) {
                kick_x_left = kickX_left_val;
                kick_y_left = kickY_left_val;
            }

            if (ts_right > (support_end - push_window) && ts_right < support_end) {
                kick_x_right = kickX_right_val;
                kick_y_right = kickY_right_val;
            }


            FootController::Pose leftPose={
            offsetZ_left-leftPosXY.y - kick_y_left,-Spac,-leftPosXY.x + offsetX_left + kick_x_left,
            0, 0, 0
            };

            FootController::Pose rightPose={
            offsetZ_right-rightPosXY.y - kick_y_right,+Spac,-rightPosXY.x + offsetX_right + kick_x_right,
            0, 0, 0
            };

            // 逆運動学 (IK) を介して各足のサーボへ指令を出力
            leftFoot.setTargetPose(leftPose);
            rightFoot.setTargetPose(rightPose);

            
            if(frame < totalFrames){
                frame++;
            }else{
                frame = 0;
                taskPhase = 0;//最初に戻る

                return false;//終了時は、falseを返す
            }
            break;
        }
    }
    return true;//実行中は、trueを返す
}

bool endWalking(GaitParameters &param_p) {//歩行状態から静止状態へ移行
    // 毎フレーム定義・計算する変数（ローカル変数）
    float T = param_p.T * 1.0f;
    //float h = param_p.h * 0.4f;
    float DutyX = param_p.DutyX;
    float DutyY = param_p.DutyY;
    int Fps = param_p.Fps;
    int Spac = param_p.Spac;

    float offsetZ_left = param_p.offsetZ_left;
    float offsetX_left = param_p.offsetX_left;
    float offsetZ_right = param_p.offsetZ_right;
    float offsetX_right = param_p.offsetX_right;

    float kickX_left_val = param_p.kickX_left * 0.0f;
    float kickY_left_val = param_p.kickY_left * 0.0f;
    float kickX_right_val = param_p.kickX_right * 0.0f;
    float kickY_right_val = param_p.kickY_right * 0.0f;
    float push_window = param_p.kickTime;

    int totalFrames = (int)(T * Fps);

    // 状態を維持する必要がある変数のみ static にする
    static int taskPhase = 0;
    static int frame = 0;

    switch (taskPhase) {
        case 0:
        {

            taskPhase = 1;
            break;
        }
        case 1:
        {
            // 経過時間 ts の計算 (秒)
            float ts_left = (float)frame / Fps;
            // 右足は位相を半周期 (T / 2.0) ずらす
            float ts_right = phaseShift_f(ts_left, T / 2.0f);

            // 終了へ向かうため、Wdを逆向きに減衰させる
            float Wd = map(totalFrames - frame, 0, totalFrames, 0, (int)param_p.Wd);
            float h = map(totalFrames - frame, 0, totalFrames, 0, (int)param_p.h);

            // 軌道生成処理 (FootController.cpp の tread 関数を利用)
            Vector2 leftPosXY = leftFoot.tread(h, Wd, DutyX, DutyY, T, ts_left);
            Vector2 rightPosXY = rightFoot.tread(h, Wd, DutyX, DutyY, T, ts_right);

            float kick_x_left = 0.0f;
            float kick_y_left = 0.0f;
            float kick_x_right = 0.0f;
            float kick_y_right = 0.0f;

            // 接地期の終盤で後ろ・下へ押し込む
            float support_end = DutyX * T / 2.0f;

            if (ts_left > (support_end - push_window) && ts_left < support_end) {
                kick_x_left = kickX_left_val;
                kick_y_left = kickY_left_val;
            }

            if (ts_right > (support_end - push_window) && ts_right < support_end) {
                kick_x_right = kickX_right_val;
                kick_y_right = kickY_right_val;
            }

            FootController::Pose leftPose = {
                offsetZ_left - leftPosXY.y - kick_y_left, -Spac, -leftPosXY.x + offsetX_left + kick_x_left,
                0, 0, 0
            };

            FootController::Pose rightPose = {
                offsetZ_right - rightPosXY.y - kick_y_right, +Spac, -rightPosXY.x + offsetX_right + kick_x_right,
                0, 0, 0
            };

            // 逆運動学 (IK) を介して各足のサーボへ指令を出力
            leftFoot.setTargetPose(leftPose);
            rightFoot.setTargetPose(rightPose);

            if (frame < totalFrames) {
                frame++;
            } else {
                frame = 0;
                taskPhase = 0; // 最初に戻る

                return false; // 終了時は、falseを返す
            }
            break;
        }
    }
    return true; // 実行中は、trueを返す
}


/**************************************************** */


// 共通計算補助（ボタンビット値やスティック値の取得）
static inline uint32_t getButtonBits() {
    uint32_t button_bits = 0;
    memcpy(&button_bits, &Dualshock4.data.button, sizeof(Dualshock4.data.button));
    return button_bits;
}

static inline int getStickLYMapped() {
    return map_controller(Dualshock4.data.analog.stick.ly, 20, -128, 127, -10, 10);
}

// 1次元の目標角度を計算する関数（全ステップ管理版）
float calculateStepMotion(float start_angle, float target_angle, int current_step, int total_steps) {
    if (current_step >= total_steps) {
        return target_angle; // 終了
    }

    // 進行割合 (0.0 ～ 1.0)
    float t = (float)current_step / (float)total_steps;

    // 3次エルミート曲線（Smoothstep）による加減速
    // スタートとゴールで速度が 0（滑らかに発進・停止）になる
    float smooth_t = t * t * (3.0f - 2.0f * t);

    // 現在のステップにおける目標角度を返す
    return start_angle + (target_angle - start_angle) * smooth_t;
}

//教示によるモーション再生用の補助プログラム
bool motion_sub(int totalSteps,int index,int joint_num,int &currentStep,int &taskPhase,const float *armAngle_zero,const float *armAngle_tar1){
    for(int i=0;i<joint_num;i++){
        float currentPos = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
        ServoArray[i+index]->setPosDeg(currentPos,false);
        delay(2);
    }
    if(currentStep < totalSteps){
        currentStep++;
    }else{
        taskPhase++;
        currentStep=0;
        return false;
    }
    return true;
}


// ==========================================
// 2. canStart() の実装 モーション起動の条件
// ==========================================
namespace motion {
    // --- 歩行系 ---
    bool walk1::canStart() {
        int stick_ly = getStickLYMapped();
        return (stick_ly > 0) && !(Dualshock4.data.button.l3);
    }

    bool walkBack::canStart() {
        int stick_ly = getStickLYMapped();
        return (stick_ly < 0) && !(Dualshock4.data.button.l3);
    }

    bool walkY::canStart() {
        auto& btn = Dualshock4.data.button;
        return btn.right || btn.left;
    }

    bool turn::canStart() {
        return getButtonBits() == L3_BIT;
    }

    // --- 攻撃系 ---
    bool attack_Light_left::canStart() { return Dualshock4.data.button.l1; }
    bool attack_Medium_left::canStart() { return false; }//return Dualshock4.data.button.l2
    bool attack_Light_right::canStart() { return Dualshock4.data.button.r1; }
    bool attack_Medium_right::canStart() { return Dualshock4.data.button.r2; }
    bool attack_Heavy_1::canStart() { return Dualshock4.data.button.l2; }
    bool attack_Heavy_2::canStart() { return false; } // 旧実装なし（仮）

    // --- ポーズ・特殊系 ---
    bool taunt::canStart() {
        return getButtonBits() == OPTIONS_BIT;
    }

    bool nop::canStart() {
        // 攻撃モーション（L1/L2/R1/R2）がいずれも押されていない時にtrue
        auto& btn = Dualshock4.data.button;
        bool activeAttack = btn.l1 || btn.l2 || btn.r1 || btn.r2;
        return !activeAttack;
    }

    bool LOCK_DebugMode::canStart() {
        return getButtonBits() == PS_BIT;
    }

    bool pose::canStart() {
        return getButtonBits() == SHARE_BIT;
    }

    bool chair::canStart() { return false; }     // 旧実装なし（仮）
    bool kneeling::canStart() { return false; }  // 旧実装なし（仮）

    // --- 起き上がり系 ---
    bool getUp::canStart() {
        return getButtonBits() == TOUCHPAD_BIT;
    }
    bool getUp_supine::canStart() { return false; } // 旧実装なし（仮）
    bool getUp_prone::canStart() { return false; }  // 旧実装なし（仮）

    // --- 姿勢制御系 ---
    bool sit_stand::canStart() {
        return getButtonBits() == TRIANGLE_BIT;
    }

    bool sitDown::canStart() { return false; }  // sit_standに一本化されている場合（必要に応じて実装）
    bool standUp::canStart() { return false; }  // sit_standに一本化されている場合（必要に応じて実装）

    bool lowerPos::canStart() {
        return Dualshock4.data.button.down;
    }

    bool hip::canStart() {
        return true; // 常時起動
    }

    bool jump::canStart(){
        return false;//現在未割り当て。
    }

    bool ServoStateDeBug::canStart(){
        return true;//常時起動
    }

    bool TeachingMode::canStart() {
        // TeachingModeはtaskManagerから優先実行される。通常の排他制御経路向けの判定。
        return false; // TeachingModeはtaskManagerから毎周期直接駆動する
    }

/**************************************************** */

    bool walk1::motionLoop() {
        GaitParameters &param_p = Config::MV_X_PARAM_3;

        // 毎フレーム定義・計算する変数（ローカル変数）
        float T = param_p.T;
        float h = param_p.h;
        float DutyX = param_p.DutyX;
        float DutyY = param_p.DutyY;
        int Fps = param_p.Fps;
        float Spac = param_p.Spac;
        float Wd = param_p.Wd;

        float offsetZ_left = param_p.offsetZ_left;
        float offsetX_left = param_p.offsetX_left;
        float offsetZ_right = param_p.offsetZ_right;
        float offsetX_right = param_p.offsetX_right;

        float kickX_left_val = param_p.kickX_left;
        float kickY_left_val = param_p.kickY_left;
        float kickX_right_val = param_p.kickX_right;
        float kickY_right_val = param_p.kickY_right;
        float push_window = param_p.kickTime;

        int totalFrames = (int)(T * Fps);

        // 状態を維持する必要がある変数のみ static にする
        static int taskPhase = 0;
        static int frame = 0;

        switch (taskPhase){
            case 0:
            {
                for(int i=0;i<19;i++){
                    ServoArray[i]->setStretch(stretch);
                    delay(5);
                }


                taskPhase = 1;
                break;
            }
            case 1:
            {
                //歩行準備
                
                //if(!startWalking(param_p))taskPhase = 2;
                taskPhase = 2;
                
                break;
            }
            case 2:
            {
                // 経過時間 ts の計算 (秒)
                float ts_left = (float)frame / Fps;
                // 右足は位相を半周期 (T / 2.0) ずらす
                float ts_right = fmod(ts_left + (T / 2.0f), T);

                int stick_lx = map_controller(Dualshock4.data.analog.stick.lx,20,-128,127,-100,100);

                float wd_def = (Wd*0.5) * stick_lx / 100.0;
                float offsetY_def =0 * stick_lx / 100.0;
                float angle_def = (15*PI/360.0)*stick_lx / 100.0;

                float wd_left = Wd + wd_def;
                float wd_right = Wd - wd_def;

                // 軌道生成処理 (FootController.cpp の tread 関数を利用)
                Vector2 leftPosXY = leftFoot.tread(h,wd_left,DutyX,DutyY,T,ts_left);
                Vector2 rightPosXY = rightFoot.tread(h,wd_right,DutyX,DutyY,T,phaseShift_f(ts_left,T/2.0));

                float kick_x_left = 0.0f;
                float kick_y_left = 0.0f;
                float angle_left=0;

                float kick_x_right = 0.0f;
                float kick_y_right = 0.0f;
                float angle_right=0;

                // 接地期の終盤で後ろ・下へ押し込む
                float support_end = DutyX * T / 2.0f;

                if (ts_left > (support_end - push_window) && ts_left < support_end) {
                    kick_x_left = kickX_left_val - offsetY_def;
                    kick_y_left = kickY_left_val + offsetY_def;
                    angle_left = angle_def;
                }

                if (ts_right > (support_end - push_window) && ts_right < support_end) {
                    kick_x_right = kickX_right_val - offsetY_def;
                    kick_y_right = kickY_right_val + offsetY_def;
                    angle_right = angle_def;
                }

                FootController::Pose leftPose={
                offsetZ_left-leftPosXY.y - kick_y_left,-Spac,-leftPosXY.x + offsetX_left + kick_x_left,
                angle_left, 0, 0
                };

                FootController::Pose rightPose={
                offsetZ_right-rightPosXY.y - kick_y_right,+Spac,-rightPosXY.x + offsetX_right + kick_x_right,
                angle_right, 0, 0
                };

                // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                leftFoot.setTargetPose(leftPose);
                rightFoot.setTargetPose(rightPose);

                if(frame < totalFrames){
                    frame++;
                }else{
                    frame = 0;
                }

                int stick_ly = map_controller(Dualshock4.data.analog.stick.ly,20,-128,127,-10,10);
                //行進のコントローラがない場合、または、回転のボタンが押された場合は抜ける。 
                if((!(stick_ly > 0) || Dualshock4.data.button.l3) && (frame>=totalFrames)){//最終フレームで抜けるように
                    taskPhase = 3;
                    frame = 0;
                }
                break;
            }        
            case 3:
            {
                //歩行準備
                if(!endWalking(param_p)){
                    taskPhase = 0;
                    return false;//終了時は、falseを返す
                }
            }
        }
        return true;//実行中は、trueを返す
    }

    bool walkBack::motionLoop() {
        GaitParameters &param_p = Config::MV_X_PARAM_BACK;

        // 毎フレーム定義・計算する変数（ローカル変数）
        float T = param_p.T;
        float h = param_p.h;
        float DutyX = param_p.DutyX;
        float DutyY = param_p.DutyY;
        int Fps = param_p.Fps;
        float Spac = param_p.Spac;
        float Wd = param_p.Wd;

        float offsetZ_left = param_p.offsetZ_left;
        float offsetX_left = param_p.offsetX_left;
        float offsetZ_right = param_p.offsetZ_right;
        float offsetX_right = param_p.offsetX_right;

        float kickX_left_val = param_p.kickX_left;
        float kickY_left_val = param_p.kickY_left;
        float kickX_right_val = param_p.kickX_right;
        float kickY_right_val = param_p.kickY_right;
        float push_window = param_p.kickTime;

        int totalFrames = (int)(T * Fps);

        // 状態を維持する必要がある変数のみ static にする
        static int taskPhase = 0;
        static int frame = 0;

        switch (taskPhase){
            case 0:
            {
                for(int i=0;i<19;i++){
                    ServoArray[i]->setStretch(stretch);
                    delay(5);
                }

                taskPhase = 1;
                break;
            }
            case 1:
            {
                //歩行準備
                if(!startWalking(param_p))taskPhase = 2;
                break;
            }
            case 2:
            {
                // 経過時間 ts の計算 (秒)
                float ts_left = (float)frame / Fps;
                // 右足は位相を半周期 (T / 2.0) ずらす
                float ts_right = fmod(ts_left + (T / 2.0f), T);

                int stick_lx = map_controller(Dualshock4.data.analog.stick.lx,20,-128,127,-100,100);

                float wd_def = (Wd*0.5) * stick_lx / 100.0;
                float offsetY_def =0 * stick_lx / 100.0;
                float angle_def = (15*PI/360.0)*stick_lx / 100.0;

                float wd_left = Wd + wd_def;
                float wd_right = Wd - wd_def;

                // 軌道生成処理 (FootController.cpp の tread 関数を利用)
                Vector2 leftPosXY = leftFoot.tread(h,wd_left,DutyX,DutyY,T,ts_left);
                Vector2 rightPosXY = rightFoot.tread(h,wd_right,DutyX,DutyY,T,phaseShift_f(ts_left,T/2.0));

                float kick_x_left = 0.0f;
                float kick_y_left = 0.0f;
                float angle_left=0;

                float kick_x_right = 0.0f;
                float kick_y_right = 0.0f;
                float angle_right=0;

                // 接地期の終盤で後ろ・下へ押し込む
                float support_end = DutyX * T / 2.0f;

                if (ts_left > (support_end - push_window) && ts_left < support_end) {
                    kick_x_left = kickX_left_val - offsetY_def;
                    kick_y_left = kickY_left_val + offsetY_def;
                    angle_left = angle_def;
                }

                if (ts_right > (support_end - push_window) && ts_right < support_end) {
                    kick_x_right = kickX_right_val - offsetY_def;
                    kick_y_right = kickY_right_val + offsetY_def;
                    angle_right = angle_def;
                }

                FootController::Pose leftPose={
                offsetZ_left-leftPosXY.y - kick_y_left,-Spac,-leftPosXY.x + offsetX_left + kick_x_left,
                angle_left, 0, 0
                };

                FootController::Pose rightPose={
                offsetZ_right-rightPosXY.y - kick_y_right,+Spac,-rightPosXY.x + offsetX_right + kick_x_right,
                angle_right, 0, 0
                };

                // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                leftFoot.setTargetPose(leftPose);
                rightFoot.setTargetPose(rightPose);

                if(frame < totalFrames){
                    frame++;
                }else{
                    frame = 0;
                }

                int stick_ly = map_controller(Dualshock4.data.analog.stick.ly,20,-128,127,-10,10);
                //行進のコントローラがない場合、または、回転のボタンが押された場合は抜ける。 
                if((!(stick_ly < 0) || Dualshock4.data.button.l3) && (frame>=totalFrames)){//最終フレームで抜けるように
                    taskPhase = 3;
                    frame = 0;
                }
                break;
            }        
            case 3:
            {
                //歩行準備
                if(!endWalking(param_p)){
                    taskPhase = 0;
                    return false;//終了時は、falseを返す
                }
            }
        }
        return true;//実行中は、trueを返す
    }

    bool walkY::motionLoop(){
        const bool mode = 0;//single:1 doble:0
        if(mode){

            GaitParameters &param_p = Config::MV_Y_SINGLE;
            // 毎フレーム定義・計算する変数（ローカル変数）
            float T = param_p.T;
            float h = param_p.h;
            float DutyX = param_p.DutyX;
            float DutyY = param_p.DutyY;
            int Fps = param_p.Fps;
            float Spac = param_p.Spac;
            float Wd = param_p.Wd;

            float offsetZ_left = param_p.offsetZ_left;
            float offsetX_left = param_p.offsetX_left;
            float offsetZ_right = param_p.offsetZ_right;
            float offsetX_right = param_p.offsetX_right;

            float kickX_left_val = param_p.kickX_left;
            float kickY_left_val = param_p.kickY_left;
            float kickX_right_val = param_p.kickX_right;
            float kickY_right_val = param_p.kickY_right;
            float push_window = param_p.kickTime;

            int totalFrames = (int)(T * Fps);



            // 状態を維持する必要がある変数のみ static にする
            static int taskPhase = 0;
            static int frame = 0;

            //左右に移動
            static float moveValue;

            switch (taskPhase){
                case 0:
                {
                    taskPhase = 2;//スキップ
                    moveValue = 0;
                    break;
                }
                case 1:
                {
                    //歩行準備
                    //if(!startWalking(param_p))taskPhase = 2;
                    taskPhase = 2;
                    break;
                }
                case 2:
                {
                    // 経過時間 ts の計算 (秒)
                    float ts_left = (float)frame / Fps;
                    // 右足は位相を半周期 (T / 2.0) ずらす
                    float ts_right = fmod(ts_left + (T / 2.0f), T);
                    
                    moveValue = moveValue + (-(float)Dualshock4.data.button.left + (float)Dualshock4.data.button.right)*0.05;
                    if(moveValue > 1){
                        moveValue = 1;
                    }else if(moveValue < -1){
                        moveValue = -1;
                    }

                    Serial.printf("Dualshock4.data.button.left:%d Dualshock4.data.button.right:%d moveValue:%f\n",Dualshock4.data.button.left,Dualshock4.data.button.right,moveValue);

                    // 軌道生成処理 (FootController.cpp の tread 関数を利用)
                    Vector2 leftPosXY = {0,0};
                    Vector2 rightPosXY = {0,0};

                    if(moveValue==1){
                        // 軌道生成処理 (FootController.cpp の tread 関数を利用)
                        leftPosXY = leftFoot.tread(h,Wd*moveValue,DutyX,DutyY,T,ts_left);
                    }else{
                        rightPosXY = rightFoot.tread(h,Wd*moveValue,DutyX,DutyY,T,phaseShift_f(ts_left,T/2.0));
                    }

                    float kick_x_left = 0.0f;
                    float kick_y_left = 0.0f;
                    float angle_left=0;

                    float kick_x_right = 0.0f;
                    float kick_y_right = 0.0f;
                    float angle_right=0;

                    // 接地期の終盤で後ろ・下へ押し込む
                    float support_end = DutyX * T / 2.0f;
                    
                    float angle_def = -((30)*PI/360) * moveValue;

                    if (ts_left > (support_end - push_window) && ts_left < support_end) {
                        kick_x_left = kickX_left_val - 0;
                        kick_y_left = kickY_left_val + 0;
                    }

                    if (ts_right > (support_end - push_window) && ts_right < support_end) {
                        kick_x_right = kickX_right_val - 0;
                        kick_y_right = kickY_right_val + 0;
                    }

                    FootController::Pose leftPose={
                    offsetZ_left-leftPosXY.y,-Spac-leftPosXY.x,0 + offsetX_left,
                    0, 0, 0
                    };

                    FootController::Pose rightPose={
                    offsetZ_right-rightPosXY.y,+Spac-rightPosXY.x ,0 + offsetX_right,
                    0, 0, 0
                    };

                    // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                    leftFoot.setTargetPose(leftPose,angle_def,30);
                    rightFoot.setTargetPose(rightPose,angle_def,-30);

                    if(frame < totalFrames){
                        frame++;
                    }else{
                        frame = 0;
                    }

                    //行進のコントローラがない場合、または、回転のボタンが押された場合は抜ける。 
                    if((!Dualshock4.data.button.left && !Dualshock4.data.button.right) && (frame>=totalFrames)){//最終フレームで抜けるように
                        taskPhase = 3;
                        frame = 0;
                    }
                    break;
                }        
                case 3:
                {
                    taskPhase = 0;
                    FootController::Pose leftPose={
                    offsetZ_left,-Spac,0 + offsetX_left,
                    0, 0, 0
                    };
                    FootController::Pose rightPose={
                    offsetZ_right,+Spac,0 + offsetX_right,
                    0, 0, 0
                    };
                    // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                    leftFoot.setTargetPose(leftPose);
                    rightFoot.setTargetPose(rightPose);
                    return false;
                }
            }
            return true;//実行中は、trueを返す
        }else{
            
            GaitParameters &param_p = Config::MV_Y_SINGLE;
            // 毎フレーム定義・計算する変数（ローカル変数）
            float T = param_p.T;
            float h = param_p.h;
            float DutyX = param_p.DutyX;
            float DutyY = param_p.DutyY;
            int Fps = param_p.Fps;
            float Spac = param_p.Spac;
            float Wd = param_p.Wd;

            float offsetZ_left = param_p.offsetZ_left;
            float offsetX_left = param_p.offsetX_left;
            float offsetZ_right = param_p.offsetZ_right;
            float offsetX_right = param_p.offsetX_right;

            float kickX_left_val = param_p.kickX_left;
            float kickY_left_val = param_p.kickY_left;
            float kickX_right_val = param_p.kickX_right;
            float kickY_right_val = param_p.kickY_right;
            float push_window = param_p.kickTime;

            int totalFrames = (int)(T * Fps);

            // 状態を維持する必要がある変数のみ static にする
            static int taskPhase = 0;
            static int frame = 0;

            //左右に移動
            static float moveValue;

            switch (taskPhase){
                case 0:
                {
                    taskPhase = 2;//スキップ
                    moveValue = 0;
                    break;
                }
                case 1:
                {
                    //歩行準備
                    //if(!startWalking(param_p))taskPhase = 2;
                    taskPhase = 2;
                    break;
                }
                case 2:
                {
                    // 経過時間 ts の計算 (秒)
                    float ts_left = (float)frame / Fps;
                    // 右足は位相を半周期 (T / 2.0) ずらす
                    float ts_right = fmod(ts_left + (T / 2.0f), T);
                    
                    moveValue = moveValue + (-(float)Dualshock4.data.button.left + (float)Dualshock4.data.button.right)*0.05;
                    if(moveValue > 1){
                        moveValue = 1;
                    }else if(moveValue < -1){
                        moveValue = -1;
                    }

                    Serial.printf("Dualshock4.data.button.left:%d Dualshock4.data.button.right:%d moveValue:%f\n",Dualshock4.data.button.left,Dualshock4.data.button.right,moveValue);

                    // 軌道生成処理 (FootController.cpp の tread 関数を利用)
                    Vector2 leftPosXY = {0,0};
                    Vector2 rightPosXY = {0,0};
                    leftPosXY = leftFoot.tread(h,Wd*moveValue,DutyX,DutyY,T,ts_left);
                    rightPosXY = rightFoot.tread(h,Wd*moveValue,DutyX,DutyY,T,phaseShift_f(ts_left,T/2.0));
                    
                    if(moveValue==1){//足の上下はさせるが、水平方向の足の動きはさせない。
                        rightPosXY.x = 0;
                        rightPosXY.y = rightPosXY.y;
                    }else{
                        leftPosXY.x = 0;
                        leftPosXY.y = leftPosXY.y;
                    }

                    float kick_x_left = 0.0f;
                    float kick_y_left = 0.0f;
                    float angle_left=0;

                    float kick_x_right = 0.0f;
                    float kick_y_right = 0.0f;
                    float angle_right=0;

                    // 接地期の終盤で後ろ・下へ押し込む
                    float support_end = DutyX * T / 2.0f;
                    
                    float angle_def = -((30)*PI/360) * moveValue;

                    if (ts_left > (support_end - push_window) && ts_left < support_end) {
                        kick_x_left = kickX_left_val - 0;
                        kick_y_left = kickY_left_val + 0;
                    }

                    if (ts_right > (support_end - push_window) && ts_right < support_end) {
                        kick_x_right = kickX_right_val - 0;
                        kick_y_right = kickY_right_val + 0;
                    }

                    FootController::Pose leftPose={
                    offsetZ_left-leftPosXY.y,-Spac-leftPosXY.x,0 + offsetX_left,
                    0, 0, 0
                    };

                    FootController::Pose rightPose={
                    offsetZ_right-rightPosXY.y,+Spac-rightPosXY.x ,0 + offsetX_right,
                    0, 0, 0
                    };

                    // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                    leftFoot.setTargetPose(leftPose,angle_def);
                    rightFoot.setTargetPose(rightPose,angle_def);

                    if(frame < totalFrames){
                        frame++;
                    }else{
                        frame = 0;
                    }

                    //行進のコントローラがない場合、または、回転のボタンが押された場合は抜ける。 
                    if((!Dualshock4.data.button.left && !Dualshock4.data.button.right) && (frame>=totalFrames)){//最終フレームで抜けるように
                        taskPhase = 3;
                        frame = 0;
                    }
                    break;
                }        
                case 3:
                {
                    taskPhase = 0;
                    FootController::Pose leftPose={
                    offsetZ_left,-Spac,0 + offsetX_left,
                    0, 0, 0
                    };
                    FootController::Pose rightPose={
                    offsetZ_right,+Spac,0 + offsetX_right,
                    0, 0, 0
                    };
                    // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                    leftFoot.setTargetPose(leftPose);
                    rightFoot.setTargetPose(rightPose);
                    return false;
                }
            }
            return true;//実行中は、trueを返す
        }
    }

    bool turn::motionLoop(){

        const char flag = 0;
        if(flag){
            GaitParameters &param_p = Config::MV_X_PARAM_TURN;

            // 毎フレーム定義・計算する変数（ローカル変数）
            float T = param_p.T;
            float h = param_p.h;
            float DutyX = param_p.DutyX;
            float DutyY = param_p.DutyY;
            int Fps = param_p.Fps;
            float Spac = param_p.Spac;
            float Wd = param_p.Wd;

            float offsetZ_left = param_p.offsetZ_left;
            float offsetX_left = param_p.offsetX_left;
            float offsetZ_right = param_p.offsetZ_right;
            float offsetX_right = param_p.offsetX_right;

            float kickX_left_val = param_p.kickX_left;
            float kickY_left_val = param_p.kickY_left;
            float kickX_right_val = param_p.kickX_right;
            float kickY_right_val = param_p.kickY_right;
            float push_window = param_p.kickTime;

            int totalFrames = (int)(T * Fps);

            // 状態を維持する必要がある変数のみ static にする
            static int taskPhase = 0;
            static int frame = 0;

            switch (taskPhase){
                case 0:
                {
                    Serial.print(F("turn \n"));
                    for(int i=0;i<19;i++){
                        ServoArray[i]->setStretch(stretch);
                        delay(5);
                    }

                    taskPhase = 1;
                    break;
                }
                case 1:
                {
                    //歩行準備
                    taskPhase = 2;
                    break;
                }
                case 2:
                {
                    // 経過時間 ts の計算 (秒)
                    float ts_left = (float)frame / Fps;
                    // 右足は位相を半周期 (T / 2.0) ずらす
                    float ts_right = fmod(ts_left + (T / 2.0f), T);

                    int stick_lx = map_controller(Dualshock4.data.analog.stick.lx,20,-128,127,-100,100);

                    float wd_left = -Wd * (stick_lx*0.01);
                    float wd_right = Wd * (stick_lx*0.01);

                    // 軌道生成処理 (FootController.cpp の tread 関数を利用)
                    Vector2 leftPosXY = leftFoot.tread(h,wd_left,DutyX,DutyY,T,ts_left);
                    Vector2 rightPosXY = rightFoot.tread(h,wd_right,DutyX,DutyY,T,phaseShift_f(ts_left,T/2.0));

                    float kick_x_left = 0.0f;
                    float kick_y_left = 0.0f;
                    float angle_left=0;

                    float kick_x_right = 0.0f;
                    float kick_y_right = 0.0f;
                    float angle_right=0;

                    // 接地期の終盤で後ろ・下へ押し込む
                    float support_end = DutyX * T / 2.0f;

                    if (ts_left > (support_end - push_window) && ts_left < support_end) {
                        kick_x_left = kickX_left_val - 0;
                        kick_y_left = kickY_left_val + 0;
                        angle_left = 0;
                    }

                    if (ts_right > (support_end - push_window) && ts_right < support_end) {
                        kick_x_right = kickX_right_val - 0;
                        kick_y_right = kickY_right_val + 0;
                        angle_right = 0;
                    }

                    FootController::Pose leftPose={
                    offsetZ_left-leftPosXY.y - kick_y_left,-Spac-40,-leftPosXY.x + offsetX_left + kick_x_left*(stick_lx*0.01),
                    angle_left, 0, 0
                    };

                    FootController::Pose rightPose={
                    offsetZ_right-rightPosXY.y - kick_y_right,+Spac-40,-rightPosXY.x + offsetX_right + kick_x_right*(stick_lx*0.01),
                    angle_right, 30,0 
                    };

                    // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                    leftFoot.setTargetPose(leftPose);
                    //rightFoot.setTargetPose(rightPose);

                    if(frame < totalFrames){
                        frame++;
                    }else{
                        frame = 0;
                    }

                    int stick_ly = map_controller(Dualshock4.data.analog.stick.ly,20,-128,127,-10,10);
                    //行進のコントローラがない場合、または、回転のボタンが押された場合は抜けない
                    if(!Dualshock4.data.button.l3 && frame>=totalFrames){//最終フレームで抜けるように
                        taskPhase = 3;
                        frame = 0;
                    }
                    break;
                }        
                case 3:
                {
                    taskPhase = 0;
                    return false;//終了時は、falseを返す
                }
            }
            return true;//実行中は、trueを返す
        }else{
            static int taskDelay=0;

            static int taskPhase=0;
            switch (taskPhase)
            {
            case 0:
            {
                FootController::Pose leftPose={
                NeutralPoseY,-NeutralSpac*0.5,NeutralPoseX,
                0, 0, 0
                };

                FootController::Pose rightPose={
                NeutralPoseY,+NeutralSpac*0.5,NeutralPoseX,
                0, 0, 0
                };

                // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                leftFoot.setTargetPose(leftPose);
                rightFoot.setTargetPose(rightPose);
                ServoArray[0]->setPosDeg(0);
                taskPhase=1;
            }
            case 1:
            {
                taskDelay++;
                if(taskDelay>(int)(G_fps*0.15)){//0.1秒間だけ固定
                    taskPhase++;
                    taskDelay=0;
                }
                break;
            }
            case 2:
            {
                taskDelay=0;

                FootController::Pose leftPose={
                NeutralPoseY+30,-NeutralSpac*0.5,NeutralPoseX,
                0, 0, 0
                };

                FootController::Pose rightPose={
                NeutralPoseY+30,+NeutralSpac*0.5,NeutralPoseX,
                0, 0, 0
                };

                // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                leftFoot.setTargetPose(leftPose);
                rightFoot.setTargetPose(rightPose);
                
                ServoArray[0]->setPosDeg(30);
                
                taskPhase++;
                break;
            }
            case 3:
            {
                taskDelay++;
                if(taskDelay>(int)(G_fps*0.1)){//0.1秒間だけ固定
                    taskPhase++;
                    taskDelay=0;
                }
                break;
            }
            case 4:
            {
                ServoArray[0]->setPosDeg(0);
                FootController::Pose leftPose={
                NeutralPoseY,-NeutralSpac*0.5,NeutralPoseX,
                0, 0, 0
                };

                FootController::Pose rightPose={
                NeutralPoseY,+NeutralSpac*0.5,NeutralPoseX,
                0, 0, 0
                };

                // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                leftFoot.setTargetPose(leftPose);
                rightFoot.setTargetPose(rightPose);

                taskPhase=0;
                return false;
                break;
            }
            }
            return true;
        }
    }

    bool sit_stand::motionLoop(){
        static int taskPhase = 0;
        static int currentStep = 0;
        static int arrIndex = 0;

        switch (taskPhase) {
            case 0:
                Serial.printf("sitDown!\n");
                taskPhase = 10;
                arrIndex = 0;
                break;

            case 10:
            {
                Serial.printf("sitDown!\n");
                if(!sitDown_inst.motionLoop()){
                    taskPhase = 11;
                }
                break;
            }
            case 11:
            {
                Serial.printf("!circle\n");
                if(!Dualshock4.data.button.triangle){//押しっぱなしの場合はここで止まる。
                    taskPhase = 12;
                }
                break;
            }
            case 12:
            {
                Serial.printf("circle\n");
                if(Dualshock4.data.button.triangle){//押すまで待つ。
                    taskPhase = 13;
                }
                break;
            }
            case 13:
            {
                Serial.printf("standUp\n");
                if(!standUp_inst.motionLoop()){
                    taskPhase = 14;
                }
                break;
            }
            case 14:
            {
                Serial.printf("!circle\n");
                if(!Dualshock4.data.button.triangle){//押しっぱなしの場合はここで止まる。
                    taskPhase = 15;
                }
                break;
            }
            case 15:
                taskPhase = 0;
                return false;
        }

        return true; // モーション継続中

    }

    bool sitDown::motionLoop(){
        static int taskPhase = 0;
        static int currentStep = 0;
        static int arrIndex = 0;

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        static const MotionStep motionData[] PROGMEM = {
            /* 0: posZero   */ {60, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
            /* 1:    */        {0,  {0.00,-6.55,86.67,-9.11,-67.10,0.30,78.43,1.11,-2.60,-1.15,-12.89,77.12,-94.80,-3.91,12.86,-1.92,73.81,82.45,7.56}},
        };

        const int arrNum = sizeof(motionData) / sizeof(motionData[0]);
        const int index = 0;
        const int joint_num = 19;

        switch (taskPhase) {
            case 0:
                Serial.printf("sitDown!\n");
                taskPhase = 10;
                currentStep = 0;
                arrIndex = 0;
                break;

            case 10:
            {
                // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
                int dummyPhase = 0;

                // 1ステップ分（1フレーム分）の補間・出力処理
                bool isBusy = motion_sub(
                    motionData[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData[arrIndex].pose,motionData[arrIndex + 1].pose
                );

                // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
                if (!isBusy) {
                    arrIndex++; // 次の姿勢ペアへ進める

                    // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                    if (arrIndex >= arrNum - 1) {
                        taskPhase = 11;
                    }
                }
                break;
            }

            case 11:
                taskPhase = 0;
                currentStep = 0;
                arrIndex = 0;
                for(int i = 0;i<10;i++){
                    for(int i=0;i<19;i++){
                        ServoArray[i]->setPosFree();
                    delay(5);
                }
                }
                return false; // モーション完了
        }

        return true; // モーション継続中
    }

    bool standUp::motionLoop(){
        static int taskPhase = 0;
        static int currentStep = 0;
        static int arrIndex = 0;

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        static const MotionStep motionData[] PROGMEM = {
            /* 1:    */        {60,  {0.00,-6.55,86.67,-9.11,-67.10,0.30,78.43,1.11,-2.60,-1.15,-12.89,77.12,-94.80,-3.91,12.86,-1.92,73.81,82.45,7.56}},
            /* 0: posZero   */ { 0,  {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
    };

        const int arrNum = sizeof(motionData) / sizeof(motionData[0]);
        const int index = 0;
        const int joint_num = 19;

        switch (taskPhase) {
            case 0:
                Serial.printf("standUp!\n");
                taskPhase = 10;
                currentStep = 0;
                arrIndex = 0;
                break;

            case 10:
            {
                // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
                int dummyPhase = 0;

                // 1ステップ分（1フレーム分）の補間・出力処理
                bool isBusy = motion_sub(
                    motionData[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData[arrIndex].pose,motionData[arrIndex + 1].pose
                );

                // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
                if (!isBusy) {
                    arrIndex++; // 次の姿勢ペアへ進める

                    // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                    if (arrIndex >= arrNum - 1) {
                        taskPhase = 11;
                    }
                }
                break;
            }
            case 11:
                taskPhase = 0;
                currentStep = 0;
                arrIndex = 0;
                return false; // モーション完了
        }

        return true; // モーション継続中

    }

    bool lowerPos::motionLoop(){
        static int taskPhase = 0;
        static int currentStep = 0;
        static int arrIndex = 0;

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[10];
        };

        
        static const MotionStep motionData_1[] PROGMEM = {
            /* 0: posZero   */ {20, {-0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
            /* 2:           */ {10,  {-3.14 ,-22.92,26.73 ,-40.20 ,11.31 ,13.16 ,21.97 ,40.53 ,34.15 ,1.42 }},
            /* 1:    */        {0,  {6.99 ,-26.26,29.02 ,-38.27 ,-23.56,20.99 ,24.23 ,29.97 ,32.03 ,28.79 }},
        };

        const int arrNum_1 = sizeof(motionData_1) / sizeof(motionData_1[0]);

        static const MotionStep motionData_2[] PROGMEM = {
            /* 1:    */        {20,  {6.99  ,-26.26,29.02 ,-38.27 ,-23.56,20.99 ,24.23 ,29.97 ,32.03 ,28.79 }},
            /* 2:           */ {10,  {-3.14 ,-22.92,26.73 ,-40.20 ,11.31 ,13.16 ,21.97 ,40.53 ,34.15 ,1.42 }},
            /* 0: posZero   */ {0,   { -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
        };

        const int arrNum_2 = sizeof(motionData_1) / sizeof(motionData_1[0]);

        const int index = foot_index_num;
        const int joint_num = 10;

        switch (taskPhase) {
            case 0:
                Serial.printf("lowerPos!\n");
                taskPhase = 10;
                currentStep = 0;
                arrIndex = 0;
                break;

            case 10:
            {
                // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
                int dummyPhase = 0;

                // 1ステップ分（1フレーム分）の補間・出力処理
                bool isBusy = motion_sub(
                    motionData_1[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData_1[arrIndex].pose,motionData_1[arrIndex + 1].pose
                );

                // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
                if (!isBusy) {
                    arrIndex++; // 次の姿勢ペアへ進める

                    // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                    if (arrIndex >= arrNum_1 - 1) {
                        taskPhase = 11;
                        currentStep = 0;
                        arrIndex = 0;
                    }
                }
                break;
            }
            case 11:
            {
                if(!Dualshock4.data.button.down){//押している間はストップ
                    taskPhase = 12;
                    currentStep = 0;
                    arrIndex = 0;
                }
                break;
            }
            case 12:
            {
                // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
                int dummyPhase = 0;
                // 1ステップ分（1フレーム分）の補間・出力処理
                bool isBusy = motion_sub(
                    motionData_2[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData_2[arrIndex].pose,motionData_2[arrIndex + 1].pose
                );

                // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
                if (!isBusy) {
                    arrIndex++; // 次の姿勢ペアへ進める

                    // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                    if (arrIndex >= arrNum_2 - 1) {
                        taskPhase = 13;
                        currentStep = 0;
                    }
                }
                break;
            }
            case 13:
                taskPhase = 0;
                currentStep = 0;
                arrIndex = 0;
                return false; // モーション完了
        }

        return true; // モーション継続中
    }

    bool hip::motionLoop(){
        // 現在の角度を保持する静的変数
        static float current_angle = 0.0f;
        // スティック入力から目標角度を計算
        float target_angle = map_controller(Dualshock4.data.analog.stick.rx, 20, -128, 127, -45, 45);
        // 追従係数（0.0 〜 1.0）
        // 値が小さいほど滑らかに遅れて追従し、大きいほど素早く追従する
        float alpha = 0.5f; 
        current_angle += (target_angle - current_angle) * alpha;
        // 補間された角度をサーボに出力
        ServoArray[0]->setPosDeg(current_angle);

        return 0;
    }

    bool taunt::motionLoop(){
        Serial.printf("taunt 立ち姿勢!\n");
        for(int i=0;i<19;i++){
            ServoArray[i]->setPosDeg(0);
        }
        return 0;
    }

    bool attack_Light_right::motionLoop(){//弱攻撃

        // 状態を維持する必要がある変数のみ static にする
        static int taskPhase = 0;
        static int currentStep=0;

        float armAngle_zero[4]={0,70,33,-130};//rightArmJ1~rightArmJ4 ServoArray[5]~ServoArray[8]
        float armAngle_tar[4]={-14,73,-55.8,-43.6};


        int totalSteps=15;
        float currentPos[4];

        int index = rightArm_index_num;

        switch (taskPhase){
            case 0:
            {
                Serial.printf("attack_Light_1!\n");
                VIB();//バイブレーション
                taskPhase = 1;
                currentStep=0;
                break;
            }
            case 1:
            {
                int totalSteps=15;
                for(int i=0;i<4;i++){
                    currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar[i], currentStep, totalSteps);
                    ServoArray[i+index]->setPosDeg(currentPos[i]);
                }

                if(currentStep < totalSteps){
                    currentStep++;
                }else{
                    taskPhase = 2;
                    currentStep=0;
                }
                break;
            }
            case 2:
            {
                int totalSteps=20;
                for(int i=0;i<4;i++){
                    currentPos[i] = calculateStepMotion(armAngle_tar[i], armAngle_zero[i], currentStep, totalSteps);
                    ServoArray[i+index]->setPosDeg(currentPos[i]);
                }
                
                if(currentStep < totalSteps){
                    currentStep++;
                }else{
                    taskPhase = 3;
                    currentStep=0;
                }
                break;
            }
            case 3:
            {
                taskPhase = 0;
                currentStep=0;
                return false;
            }

        }
        return true;
    }

    bool attack_Light_left::motionLoop(){//弱攻撃
        // 状態を維持する必要がある変数のみ static にする
        static int taskPhase = 0;
        static int currentStep=0;

        float armAngle_zero[4]={4,80,-16,-130};//leftArmJ1~leftArmJ4 ServoArray[1]~ServoArray[4]
        float armAngle_tar[4]={5,43,75,-14};


        int totalSteps=15;
        float currentPos[4];
        int joint_num = 4;

        int index = leftArm_index_num;

        switch (taskPhase){
            case 0:
            {
                Serial.printf("attack_Light_2!\n");
                VIB();//バイブレーション
                taskPhase = 1;
                currentStep=0;
                break;
            }
            case 1:
            {
                motion_sub(15,index,joint_num,currentStep,taskPhase,armAngle_zero,armAngle_tar);
                break;
            }
            case 2:
            {
                motion_sub(30,index,joint_num,currentStep,taskPhase,armAngle_tar,armAngle_zero);
                break;
            }
            case 3:
            {
                taskPhase = 0;
                currentStep=0;
                return false;
            }

        }
        return true;
    }

    bool attack_Medium_left::motionLoop(){

        // 状態を維持する必要がある変数のみ static にする
        static int taskPhase = 0;
        static int currentStep=0;

        float armAngle_zero[4]={4,80,-16,-130};//leftArmJ1~leftArmJ4 ServoArray[1]~ServoArray[4]
        float armAngle_tar1[4]={-13,41,-26,-71};
        float armAngle_tar2[4]={50,50,76,-18};


        int totalSteps=15;
        float currentPos[4];

        int index = leftArm_index_num;

        switch (taskPhase){
            case 0:
            {
                Serial.printf("attack_Medium_1!\n");
                VIB();//バイブレーション
                taskPhase = 1;
                currentStep=0;
                break;
            }
            case 1:
            {
                int totalSteps=15;
                for(int i=0;i<4;i++){
                    currentPos[i] = calculateStepMotion(armAngle_zero[i], armAngle_tar1[i], currentStep, totalSteps);
                    ServoArray[i+index]->setPosDeg(currentPos[i]);
                }

                if(currentStep < totalSteps){
                    currentStep++;
                }else{
                    taskPhase = 2;
                    currentStep=0;
                }
                break;
            }
            case 2:
            {
                int totalSteps=15;
                for(int i=0;i<4;i++){
                    currentPos[i] = calculateStepMotion(armAngle_tar1[i], armAngle_tar2[i], currentStep, totalSteps);
                    ServoArray[i+index]->setPosDeg(currentPos[i]);
                }

                if(currentStep < totalSteps){
                    currentStep++;
                }else{
                    taskPhase = 3;
                    currentStep=0;
                }
                break;
            }
            case 3:
            {
                int totalSteps=20;
                for(int i=0;i<4;i++){
                    currentPos[i] = calculateStepMotion(armAngle_tar2[i], armAngle_zero[i], currentStep, totalSteps);
                    ServoArray[i+index]->setPosDeg(currentPos[i]);
                }
                
                if(currentStep < totalSteps){
                    currentStep++;
                }else{
                    taskPhase = 4;
                    currentStep=0;
                }
                break;
            }
            case 4:
            {
                taskPhase = 0;
                currentStep=0;
                return false;
            }

        }
        return true;
    }

    bool attack_Medium_right::motionLoop(){
        static int taskPhase = 0;
        static int currentStep = 0;
        static int arrIndex = 0;

        const int joint_num = 4;

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[joint_num];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        static const MotionStep motionData[] PROGMEM = {
            /* 0: posZero   */ { 5,  {0,70,33,-130}},
            /* 1:    */        {10,  {57.71,36.18,-10.90,-0.30}},
            /* 1:    */        {20,  {-60.11,28.21,-28.15,-57.31}},
            /* 0: posZero   */ { 0,  {0,70,33,-130}}
        };

        const int arrNum = sizeof(motionData) / sizeof(motionData[0]);
        const int index = rightArm_index_num;

        switch (taskPhase) {
            case 0:
                Serial.printf("attack_Medium_right!\n");
                VIB();//バイブレーション
                taskPhase = 10;
                currentStep = 0;
                arrIndex = 0;
                break;

            case 10:
            {
                // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
                int dummyPhase = 0;

                // 1ステップ分（1フレーム分）の補間・出力処理
                bool isBusy = motion_sub(
                    motionData[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData[arrIndex].pose,motionData[arrIndex + 1].pose
                );

                // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
                if (!isBusy) {
                    arrIndex++; // 次の姿勢ペアへ進める

                    // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                    if (arrIndex >= arrNum - 1) {
                        taskPhase = 11;
                    }
                }
                break;
            }
            case 11:
                taskPhase = 0;
                currentStep = 0;
                arrIndex = 0;
                return false; // モーション完了
        }

        return true; // モーション継続中
    }

    // attack_Heavy_1 の motionLoop は motionDataBase により定義不要となったため削除

    bool attack_Heavy_2::motionLoop(){
        Serial.printf("attack_Heavy_2!\n");
        return false;
    }

    bool getUp::motionLoop(){

        static int taskPhase;
        static bool getup_mode;

        switch (taskPhase){
            case 0:
            {
                Serial.printf("getUp!\n");
                for(int i=0;i<19;i++){
                    ServoArray[i]->setSkip(true);
                    delay(5);
                }
                taskPhase = 10;//最初のフレームは10から
                getup_mode = !g_isFaceUp;//0:仰向け 1:うつ伏せ
                break;
            }
            case 10:
            {
                bool isNext=1;
                if(getup_mode==0){
                    isNext = getUp_prone_inst.motionLoop();
                }else{
                    isNext = getUp_supine_inst.motionLoop();
                }

                if(isNext==0){
                    taskPhase = 11;//次の処理へ。
                }
                break;
            }
            case 11:
            {
                taskPhase = 0;
                for(int i=0;i<19;i++){
                    ServoArray[i]->setSkip(false);
                    delay(5);
                }
                return false;
                break;
            }
        }
        return true;
    }

    bool getUp_prone::motionLoop(){
        static int taskPhase = 0;
        static int currentStep = 0;
        static int arrIndex = 0;

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        static const MotionStep motionData[] PROGMEM = {
            /* 0: posZero   */ {30, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
            /* 1:    */        {20, {0.64,-101.55,3.37,90.21,-34.96,-83.23,-2.84,89.24,-10.73,-4.49,-21.60,74.55,-90.65,-16.67,14.78,28.11,76.44,77.29,20.76}},
            /*              */ {20, {0.64,-91.83,-5.16,-6.21,-4.42,-88.69,3.00,165.68,1.92,7.32,-1.49,71.35,-94.80,0.00,18.06,6.78,76.78,79.72,9.15}},
            /*              */ {120, {-9.99,-85.35,71.45,-0.47,-4.76,-93.22,-65.75,165.34,1.62,6.99,-1.49,71.68,-95.14,-0.30,18.06,6.45,76.44,80.02,8.84}},
            /* 6: posZero   */ { 0, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}}
        };

        const int arrNum = sizeof(motionData) / sizeof(motionData[0]);
        const int index = 0;
        const int joint_num = 19;

        switch (taskPhase) {
            case 0:
                Serial.printf("getUp_supine!\n");
                taskPhase = 10;
                currentStep = 0;
                arrIndex = 0;
                break;

            case 10:
            {
                // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
                int dummyPhase = 0;

                // 1ステップ分（1フレーム分）の補間・出力処理
                bool isBusy = motion_sub(
                    motionData[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData[arrIndex].pose,motionData[arrIndex + 1].pose
                );

                // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
                if (!isBusy) {
                    arrIndex++; // 次の姿勢ペアへ進める

                    // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                    if (arrIndex >= arrNum - 1) {
                        taskPhase = 11;
                    }
                }
                break;
            }

            case 11:
                taskPhase = 0;
                currentStep = 0;
                arrIndex = 0;
                return false; // モーション完了
        }

        return true; // モーション継続中
    }

    bool getUp_supine::motionLoop() {
        static int taskPhase = 0;
        static int currentStep = 0;
        static int arrIndex = 0;

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        static const MotionStep motionData[] PROGMEM = {
            /* 0: posZero   */ {30, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}},
            /* 1: posSupine */ {30, {0.00, -71.42, 100.95, 58.42, -90.82, 61.66, 90.45, -51.30, -85.22, 2.77, -4.18, -8.30, -6.92, -4.25, 14.48, 5.80, -1.62, -0.44, 13.67}},
            /*    支援      */ {25, {0.00,-4.86,102.53,35.98,-33.65,-3.14,91.70,-52.28,43.67,-3.14,-75.77,21.06,-96.73,-76.34,14.14,80.05,25.04,79.72,82.28}}, 
            /* 2: posTar1   */ {25, {0.00, -56.40, 1.45, 9.25, -129.74, 32.30, -3.51, 9.25, -119.31, 23.93, -90.82, 22.07, -99.93, -84.04, 11.88, 91.33, 4.25, 90.04, 99.09}},
            /* 3: posTar2   */ //{25, {0.00, 35.98, 30.21, 46.37, 0.81, -19.81, 39.05, -57.81, 6.51, 3.41, -93.08, -11.58, -90.96, -84.04, 8.94, 91.97, -6.92, 80.02, 98.75}},
            /* 4: posTar3   */ {120, {0.00, -75.60, 21.16, 28.15, -75.63, 29.67, 23.12, 31.39, -57.95, -11.47, -94.37, 37.66, -98.96, -93.01, 23.25, 92.61, 42.83, 84.27, 91.33}},
            /* 5: posTar4   */ {120, {0.00, -25.41, 33.45, 9.59, 21.67, 4.93, 36.48, 22.61, -15.63, 2.77, -0.13, 75.84, -51.17, -1.62, 11.21, 30.37, 62.61, 49.34, 32.03}},
            /* 6: posZero   */ { 0, {0.00, 4.000, 80.00, -16.00, -130.00, 00.00, 70.00, 33.000, -130.00, -0.81, -4.83, 32.03, -25.01, -3.91, 15.46, 10.33, 32.94, 15.63, 10.77}}
        };

        const int arrNum = sizeof(motionData) / sizeof(motionData[0]);
        const int index = 0;
        const int joint_num = 19;

        switch (taskPhase) {
            case 0:
                Serial.printf("getUp_supine!\n");
                taskPhase = 10;
                currentStep = 0;
                arrIndex = 0;
                break;

            case 10:
            {
                // motion_sub 内で taskPhase++ されるのを防ぐためダミー変数を渡す
                int dummyPhase = 0;

                // 1ステップ分（1フレーム分）の補間・出力処理
                bool isBusy = motion_sub(
                    motionData[arrIndex].frameNum,index,joint_num,currentStep,dummyPhase,motionData[arrIndex].pose,motionData[arrIndex + 1].pose
                );

                // currentStep が frameNum に達して motion_sub が false を返した場合（区間完了）
                if (!isBusy) {
                    arrIndex++; // 次の姿勢ペアへ進める

                    // 全ての遷移（全 arrNum - 1 区間）が終わった場合
                    if (arrIndex >= arrNum - 1) {
                        taskPhase = 11;
                    }
                }
                break;
            }

            case 11:
                taskPhase = 0;
                currentStep = 0;
                arrIndex = 0;
                return false; // モーション完了
        }

        return true; // モーション継続中
    }

    bool nop::motionLoop(){
        Dualshock4.setLed(100, 100, 100);
        Dualshock4.sendToController();

        static int count = 0;
        if(count>G_fps*4){
            for (int i = 0; i < 19; i++) {
                ServoArray[i]->setStretch(stretch);
                delay(1);
            }
            count=0;
        }else{
            count++;
        }

        Serial.printf(".");
        return false;
    }

    bool LOCK_DebugMode::motionLoop(){
        Serial.printf("DebugMode! comand!\n");
        float armAngle_set[19];
        bool free_mode = true;

        bool push_circle = Dualshock4.data.button.circle;

        while(!Dualshock4.data.button.cross && !g_teachModeActive){
            Serial.printf("DebugMode!\n");

            if(push_circle==0 && Dualshock4.data.button.circle==1){
                free_mode = !free_mode;
            }
            push_circle=Dualshock4.data.button.circle;

            if(free_mode){
                for(int i = 0;i < 19;i++){
                    delay(5);
                    auto returnData = ServoArray[i]->setPosFree().getPosDeg();
                    Serial.printf("[%s]:%f (%s)\n",ServoArray_name[i],returnData.value,returnData.error_msg);
                    delay(5);
                }
                Serial.print("{");
                for(int i = 0;i < 19;i++){
                    delay(5);
                    auto returnData = ServoArray[i]->setPosFree().getPosDeg();
                    if(i!=0)Serial.print(",");
                    Serial.print(returnData.value);
                    armAngle_set[i]=returnData.value;
                    delay(5);
                }
                Serial.println("}");

                delay(1500);
                Serial.println("---");
            }else{
                for(int i=0;i<19;i++){
                    ServoArray[i]->setPosDeg(armAngle_set[i],false);
                    delay(5);
                }
                Serial.println("{");
                for(int i = 0;i < 19;i++){
                    if(i!=0)Serial.print(",");
                    Serial.print(armAngle_set[i]);
                }
                Serial.println("}");
                delay(1000);
            }
        }

        return false;
    }

    bool pose::motionLoop() {
        FootController::Pose leftPose={
        NeutralPoseY+footPoseDefLeft,-NeutralSpac,NeutralPoseX,
        0, 0, 0
        };

        FootController::Pose rightPose={
        NeutralPoseY+footPoseDefRight,+NeutralSpac,NeutralPoseX,
        0, 0, 0
        };

        // 逆運動学 (IK) を介して各足のサーボへ指令を出力
        leftFoot.setTargetPose(leftPose);
        rightFoot.setTargetPose(rightPose);

        return false;
    }

    bool chair::motionLoop() {
        return false;
    }

    bool kneeling::motionLoop() {
        return false;
    }

    bool jump::motionLoop(){//ジャンプ動作

        static int taskDelay=0;

        static int taskPhase=0;
        switch (taskPhase)
        {
        case 0:
        {
            taskDelay=0;

            FootController::Pose leftPose={
            NeutralPoseY+100,-NeutralSpac*0.5,NeutralPoseX,
            0, 0, 0
            };

            FootController::Pose rightPose={
            NeutralPoseY+100,+NeutralSpac*0.5,NeutralPoseX,
            0, 0, 0
            };

            // 逆運動学 (IK) を介して各足のサーボへ指令を出力
            leftFoot.setTargetPose(leftPose);
            rightFoot.setTargetPose(rightPose);

            taskPhase++;
            break;
        }
        case 1:
        {

            taskDelay++;
            if(taskDelay>(int)(G_fps*0.1)){//0.1秒間だけ固定
                taskPhase++;
                taskDelay=0;
            }
            break;
        }
        case 2:
        {
            FootController::Pose leftPose={
            NeutralPoseY,-NeutralSpac,NeutralPoseX,
            0, 0, 0
            };

            FootController::Pose rightPose={
            NeutralPoseY,+NeutralSpac,NeutralPoseX,
            0, 0, 0
            };

            // 逆運動学 (IK) を介して各足のサーボへ指令を出力
            leftFoot.setTargetPose(leftPose);
            rightFoot.setTargetPose(rightPose);

            taskPhase=0;
            return false;
            break;
        }
        }
        return true;
    }


    
    bool ServoStateDeBug::motionLoop(){
        if (!deviceConnected) {
            return false;
        }

        // 1フレーム = [frameNo:2byte][servo0..18: 19 * 2byte] = 40byte
        // 全19軸の順番は固定なので、サーボIDは毎回送信しない。
        // 各位置値は符号なし16bit、リトルエンディアン。
        static uint16_t frameNumber = 0;
        uint8_t txBuffer[40];

        txBuffer[0] = static_cast<uint8_t>(frameNumber & 0xFF);
        txBuffer[1] = static_cast<uint8_t>((frameNumber >> 8) & 0xFF);

        for (int i = 0; i < 19; ++i) {
            uint16_t pos = 0;
            if (ServoArray[i] != nullptr) {
                pos = static_cast<uint16_t>(ServoArray[i]->getLastSetPosIcs());
            }

            const int offset = 2 + i * 2;
            txBuffer[offset]     = static_cast<uint8_t>(pos & 0xFF);
            txBuffer[offset + 1] = static_cast<uint8_t>((pos >> 8) & 0xFF);
        }

        sendBLE(txBuffer, sizeof(txBuffer));
        ++frameNumber; // uint16_tなので65535の次は0に戻る

        return false; // 次のメインループで次フレームを生成する
    }

    // ----------------------------------------------------------------
    // 同期(TX)モードの1周期処理。ブロッキングせず、毎周期:
    //  1. メールボックスから最新の軸/座標指令を取り出す
    //  2. 座標指令はマイコン側でIK計算 → 軸目標(絶対ICS)へ変換
    //  3. 全軸の目標へ一次遅れ+速度制限で滑らかに追従して出力（PC側の送信間隔が
    //     揺れても動きが階段状にならない）
    //  4. FB要求軸を1周期1軸ずつ読む（1軸の応答待ちで全体が止まらない）
    //  5. 全軸の現在値+エラーを 42B [0x01][Err:3B][19*2B] で返す（変化時/250ms毎）
    // ----------------------------------------------------------------
    namespace {
    constexpr float    TEACH_ALPHA = 0.35f;        // 追従係数(0..1)。大きいほど機敏
    constexpr float    TEACH_MAX_STEP = 600.0f;    // 1周期あたりの最大移動量[ICS]
    constexpr uint32_t TEACH_HEARTBEAT_MS = 250;
    constexpr uint32_t TEACH_AXIS_GAP_MS = 1;      // UART送信後の待ち。0〜2で調整
    constexpr int      TEACH_ICS_MIN = 3500;
    constexpr int      TEACH_ICS_MAX = 11500;
    constexpr uint8_t  TEACH_FOOT_FIRST[2] = {9, 14};          // 左足/右足の先頭サーボID
    constexpr uint32_t TEACH_FOOT_MASK[2] = {0x3E00UL, 0x7C000UL}; // bit9..13 / bit14..18

    float    g_tCur[19];
    float    g_tTarget[19];
    uint32_t g_tHasTarget = 0;
    int32_t  g_tLastSent[19];
    uint32_t g_tIkErr = 0;
    uint32_t g_tFbErr = 0;
    uint32_t g_tFbMask = 0;
    uint32_t g_tFbValid = 0;
    uint16_t g_tFbIcs[19];
    uint8_t  g_tFbRR = 0;
    // setPos()系の値 → 絶対ICSへの差分。最初の座標指令で学習する（足軸のみ）。
    int32_t  g_tDelta[19];
    uint32_t g_tDeltaKnown = 0;
    uint8_t  g_tLastResp[42];
    bool     g_tRespValid = false;
    uint32_t g_tLastRespMs = 0;

    void teachReset() {
        g_tHasTarget = 0;
        g_tIkErr = 0;
        g_tFbErr = 0;
        g_tFbMask = 0;
        g_tFbValid = 0;
        g_tFbRR = 0;
        g_tRespValid = false;
        for (int i = 0; i < 19; ++i) {
            g_tCur[i] = 0;
            g_tTarget[i] = 0;
            g_tLastSent[i] = -1;
            g_tFbIcs[i] = 0;
        }
    }

    inline bool validIcs(int32_t v) { return v >= TEACH_ICS_MIN && v <= TEACH_ICS_MAX; }

    void teachSetTarget(int i, float t) {
        const uint32_t bit = (1UL << i);
        if (!(g_tHasTarget & bit)) {
            // 現在のサーボ位置から滑らかに動かし始める。現在位置が不明ならその場で目標へ。
            const int32_t last = static_cast<int32_t>(ServoArray[i]->getLastSetPosIcs());
            if (validIcs(last)) {
                g_tCur[i] = static_cast<float>(last);
                g_tLastSent[i] = last;
            } else {
                g_tCur[i] = t;
                g_tLastSent[i] = -1;
            }
            g_tHasTarget |= bit;
        }
        g_tTarget[i] = t;
    }

    void teachFree(int i) {
        ServoArray[i]->setPosFree();
        g_tHasTarget &= ~(1UL << i);
        g_tLastSent[i] = -1;
    }

    bool teachApplyPose(int f, const TeachPoseCmd& pc) {
        FootController& foot = (f == 0) ? leftFoot : rightFoot;
        const int base = TEACH_FOOT_FIRST[f];
        const uint32_t fmask = TEACH_FOOT_MASK[f];
        FootController::Pose pose = {pc.v[0], pc.v[1], pc.v[2], pc.v[3], pc.v[4], pc.v[5]};
        long out[5];
        if (!foot.computeJointPos(pose, 0.0f, pc.mode, out)) {
            g_tIkErr |= fmask;   // 到達不可。直前の姿勢を保持してPCへエラー通知
            return false;
        }
        for (int k = 0; k < 5; ++k) {
            if (ServoArray[base + k] == nullptr) { g_tIkErr |= fmask; return false; }
        }

        if ((g_tDeltaKnown & fmask) == fmask) {
            int32_t absT[5];
            for (int k = 0; k < 5; ++k) {
                absT[k] = static_cast<int32_t>(out[k]) + g_tDelta[base + k];
                if (!validIcs(absT[k])) { g_tIkErr |= fmask; return false; }
            }
            g_tIkErr &= ~fmask;
            for (int k = 0; k < 5; ++k) teachSetTarget(base + k, static_cast<float>(absT[k]));
        } else {
            // 初回のみ: ライブラリ経由で直接指令し、実際に設定された絶対ICSとの差分を学習する。
            g_tIkErr &= ~fmask;
            foot.setJointAngles(out[0], out[1], out[2], out[3], out[4]);
            delay(5);
            bool ok = true;
            int32_t last[5];
            for (int k = 0; k < 5; ++k) {
                last[k] = static_cast<int32_t>(ServoArray[base + k]->getLastSetPosIcs());
                if (!validIcs(last[k])) ok = false;
            }
            if (ok) {
                for (int k = 0; k < 5; ++k) {
                    const int i = base + k;
                    g_tDelta[i] = last[k] - static_cast<int32_t>(out[k]);
                    g_tCur[i] = g_tTarget[i] = static_cast<float>(last[k]);
                    g_tLastSent[i] = last[k];
                    g_tHasTarget |= (1UL << i);
                }
                g_tDeltaKnown |= fmask;
                Serial.printf("[TEACH] foot%d IK offset learned: %ld %ld %ld %ld %ld\n", f,
                    (long)g_tDelta[base], (long)g_tDelta[base + 1], (long)g_tDelta[base + 2],
                    (long)g_tDelta[base + 3], (long)g_tDelta[base + 4]);
            }
        }
        return true;
    }
    } // namespace

    bool TeachingMode::motionLoop() {
        if (!deviceConnected || !g_teachModeActive) return false;

        // --- 1. メールボックス取り出し ---
        TeachAxisCmd axis;
        bool hasAxis = false;
        TeachPoseCmd poses[2];
        bool hasPose[2] = {false, false};
        bool neutral[2] = {false, false};
        bool reset = false;
        portENTER_CRITICAL(&g_teachCmdMux);
        reset = g_teachNeedReset;
        g_teachNeedReset = false;
        if (g_axisPendingValid) {
            axis = g_axisPending;
            hasAxis = true;
            g_axisPendingValid = false;
        }
        for (int f = 0; f < 2; ++f) {
            if (g_posePendingValid[f]) {
                poses[f] = g_posePending[f];
                hasPose[f] = true;
                g_posePendingValid[f] = false;
            }
            neutral[f] = g_poseNeutralReq[f];
            g_poseNeutralReq[f] = false;
        }
        g_teachCmdPending = false;
        portEXIT_CRITICAL(&g_teachCmdMux);

        if (reset) teachReset();

        // --- 2. 軸指令 ---
        if (hasAxis) {
            g_tFbMask = axis.fbMask;
            for (int i = 0; i < 19; ++i) {
                const uint32_t bit = (1UL << i);
                if (!(axis.cmdMask & bit)) continue;
                if (ServoArray[i] == nullptr) { g_tFbErr |= bit; continue; }
                if (axis.ics[i] == 0) {
                    teachFree(i);
                } else {
                    int32_t v = axis.ics[i];
                    if (v < TEACH_ICS_MIN) v = TEACH_ICS_MIN;
                    if (v > TEACH_ICS_MAX) v = TEACH_ICS_MAX;
                    teachSetTarget(i, static_cast<float>(v));
                }
            }
        }

        // --- 3. 座標(IK)指令 ---
        for (int f = 0; f < 2; ++f) {
            if (neutral[f]) {
                TeachPoseCmd n;
                n.v[0] = static_cast<float>(NeutralPoseY);                              // X(上下)
                n.v[1] = (f == 0) ? -static_cast<float>(NeutralSpac) : static_cast<float>(NeutralSpac); // Y(左右)
                n.v[2] = static_cast<float>(NeutralPoseX);                              // Z(前後)
                n.v[3] = n.v[4] = n.v[5] = 0.0f;
                n.mode = 0;
                teachApplyPose(f, n);
                uint8_t echo[26];
                echo[0] = 0x37;
                echo[1] = static_cast<uint8_t>(f);
                memcpy(echo + 2, n.v, sizeof(n.v));
                sendBLE(echo, sizeof(echo));
            } else if (hasPose[f]) {
                teachApplyPose(f, poses[f]);
            }
        }

        // --- 4. 目標への滑らかな追従出力 ---
        for (int i = 0; i < 19; ++i) {
            const uint32_t bit = (1UL << i);
            if (!(g_tHasTarget & bit) || ServoArray[i] == nullptr) continue;
            const float diff = g_tTarget[i] - g_tCur[i];
            if (fabsf(diff) <= 1.0f) {
                g_tCur[i] = g_tTarget[i];
            } else {
                float step = diff * TEACH_ALPHA;
                if (fabsf(step) < 1.0f) step = (diff > 0) ? 1.0f : -1.0f;
                if (step > TEACH_MAX_STEP) step = TEACH_MAX_STEP;
                if (step < -TEACH_MAX_STEP) step = -TEACH_MAX_STEP;
                g_tCur[i] += step;
            }
            const int32_t v = static_cast<int32_t>(lroundf(g_tCur[i]));
            if (v != g_tLastSent[i]) {
                ServoArray[i]->setPosIcs(static_cast<uint16_t>(v), false);
                g_tLastSent[i] = v;
                if (TEACH_AXIS_GAP_MS) delay(TEACH_AXIS_GAP_MS);
            }
        }

        // --- 5. FB読み取り（1周期1軸のラウンドロビン） ---
        g_tFbValid &= g_tFbMask;
        g_tFbErr &= (g_tFbMask | g_tIkErr); // 解除されたFB要求のエラーは消す
        if (g_tFbMask) {
            for (int n = 0; n < 19; ++n) {
                g_tFbRR = static_cast<uint8_t>((g_tFbRR + 1) % 19);
                const int i = g_tFbRR;
                const uint32_t bit = (1UL << i);
                if (!(g_tFbMask & bit) || ServoArray[i] == nullptr) continue;
                auto r = ServoArray[i]->getPosIcs();
                if (r.success) {
                    g_tFbIcs[i] = static_cast<uint16_t>(r.value);
                    g_tFbValid |= bit;
                    g_tFbErr &= ~bit;
                } else {
                    g_tFbValid &= ~bit;
                    g_tFbErr |= bit;
                }
                break;
            }
        }

        // --- 6. 応答 [0x01][Err:3B][19*2B] ---
        uint8_t resp[42] = {0};
        resp[0] = 0x01;
        const uint32_t err = (g_tIkErr | g_tFbErr) & TEACH_ALL_MASK;
        resp[1] = static_cast<uint8_t>(err & 0xFF);
        resp[2] = static_cast<uint8_t>((err >> 8) & 0xFF);
        resp[3] = static_cast<uint8_t>((err >> 16) & 0xFF);
        for (int i = 0; i < 19; ++i) {
            uint16_t v = 0;
            if (ServoArray[i] != nullptr) {
                if (g_tFbValid & (1UL << i)) v = g_tFbIcs[i];
                else v = static_cast<uint16_t>(ServoArray[i]->getLastSetPosIcs());
            }
            resp[4 + i * 2] = static_cast<uint8_t>(v & 0xFF);
            resp[5 + i * 2] = static_cast<uint8_t>((v >> 8) & 0xFF);
        }
        const uint32_t now = millis();
        if (!g_tRespValid || memcmp(resp, g_tLastResp, sizeof(resp)) != 0 ||
            (now - g_tLastRespMs) >= TEACH_HEARTBEAT_MS) {
            memcpy(g_tLastResp, resp, sizeof(resp));
            g_tRespValid = true;
            g_tLastRespMs = now;
            sendBLE(resp, sizeof(resp));
        }
        return false; // 常駐処理ではなく毎周期の呼び出し。ALL_BITは占有しない。
    }

    /*****ここまで***** */
}