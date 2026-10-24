#include "Motion.h"
#include "servoICS.h"
#include "RobotController.h"
#include <cmath>

// ==============================================================
// 角度の単位について（設計方針）
//  モーションデータ・プロトコル・UIはすべて「論理値」（= setPos(x, true) に渡す値。
//  7500 / 0deg = 校正済みの中立）で扱う。生ICSへの変換（オフセット加算）はservoICSの中だけ。
//  旧データ（オフセット無しの生値で記録）は calib::legacyRawToLogicalIcs() で変換して使う。
// ==============================================================

// ==============================================================
// 中断されたモーションの「関数内static状態」を初期化する仕組み
//  motionForceStopAll() で g_motionEpoch を進めると、各 static 変数の直後に置いた
//  MOTION_STATIC_RESET が、そのモーションの次回実行時に初期値へ戻す。
//  （旧実装では、TX開始や再生で中断したモーションが次回フェーズの途中から再開していた）
// ==============================================================
static volatile uint32_t g_motionEpoch = 1;
template <typename T, typename U> static inline void motionResetValue(T& v, const U& init) { v = static_cast<T>(init); }
#define MOTION_STATIC_RESET(var, init) \
    do { static uint32_t _motionEp = 0; if (_motionEp != g_motionEpoch) { _motionEp = g_motionEpoch; motionResetValue(var, init); } } while (0)

void motionForceStopAll() {
    for (int i = 0; i < motionList::motionArrNum; ++i) {
        motionList::motionArr[i]->isActive = false;
        motionList::motionArr[i]->onForceStop();
    }
    g_motionEpoch = g_motionEpoch + 1;
}

namespace {
constexpr int FOOT_BASE[2] = {9, 14};
constexpr uint32_t FOOT_MASK[2] = {0x3E00UL, 0x7C000UL};   // bit9..13 / bit14..18
constexpr uint32_t ALL_AXIS_MASK = (1UL << 19) - 1UL;

inline uint16_t rdU16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline int16_t rdI16(const uint8_t* p) { return static_cast<int16_t>(rdU16(p)); }
inline uint32_t rdU24(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16);
}

// 論理値が、その軸のオフセットを足した後に ICS の範囲内か
bool logicalInRange(int i, long v) {
    if (v == 0) return true;
    if (i < 0 || i >= 19 || ServoArray[i] == nullptr) return false;
    const long raw = ServoArray[i]->toRawIcs(v);
    return raw >= servoICS::ICS_LOW && raw <= servoICS::ICS_HIGH;
}

// 今の指令値（論理）。目標が登録されていればそれ、無ければ最後に送った値。0=FREE/不明
long currentLogical(int i) {
    servoICS::Servo* s = ServoArray[i];
    if (s == nullptr) return 0;
    if (s->hasTarget()) return s->toLogicalIcs(s->targetRaw());
    if (s->hasSetPos()) return s->getLastSetPosLogicalIcs();
    return 0;
}

// 軸ごとのストレッチ（保持力）。足首5(13/18)は setup() と同じ柔らかい値を維持する。
// 旧実装: setup()で足首を10にしても、nop(4秒毎)や歩行開始で全軸 stretch(120) に戻していた。
unsigned char axisStretch(int i) { return (i == 13 || i == 18) ? 10 : stretch; }

// 脚の座標 → 関節(論理)。FootController::setTargetPose() と同じ変換。
bool footIk(int f, const FootController::Pose& pose, uint8_t ik, long out[5]) {
    FootController& foot = (f == 0) ? leftFoot : rightFoot;
    if (!foot.computeJointPos(pose, 0.0f, ik, out)) return false;
    for (int k = 0; k < 5; ++k) {
        if (!logicalInRange(FOOT_BASE[f] + k, out[k])) return false;
    }
    return true;
}
} // namespace

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
bool playCodeMotion(const int16_t motionData[][19], uint16_t totalFrames, bool logical, char motionArrUsingBit, int &taskPhase, uint16_t &currentFrame) {
    static uint32_t lastFrameMs = 0;
    MOTION_STATIC_RESET(lastFrameMs, 0);

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
                        // ExportData.h の値は論理ICS（そのまま）。MotionData.h の値は旧来の「生ICS」(オフセット無し)なので旧オフセットで論理値に変換する。
                        const long v = logical ? currentPos[i] : calib::legacyRawToLogicalIcs(i, currentPos[i]);
                        ServoArray[i]->setPosIcs(v, true);
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
// 事前転送モーション（形式v2: 関節キーフレーム + 座標キーフレーム）
//  - BEGIN/DATA は BLE callback でRAMへコピーするだけ。
//  - COMMIT（全フレーム検証・IK到達確認）は loop 側。START はRAMのバッファをそのまま再生する。
//  - これは「検証・プレビュー用」で、保持できるのは1件だけ。フラッシュには書かない（電源を切ると消える）。
//    変更がなければPCは転送を省略して START だけ送れば、同じ内容を何度でも再生できる。
//  - 本番用のモーションは PCアプリが ExportData.h にエクスポートし、DECLARE_DATA_MOTION で組み込む（playCodeMotion で再生）。
//  - 座標指定の脚は、前のフレームも座標なら「足先の直線補間 + 毎周期IK」、
//    そうでなければ「目標をIKで関節にしてから関節補間」で再生する。
// ==============================================================
namespace {
using namespace proto;
constexpr size_t UP_MAX_BYTES = static_cast<size_t>(FRAME_BYTES) * MAX_FRAMES;
uint32_t g_upHash = 0;   // COMMIT時点の内容のFNV-1a。START時に再計算して、RAM上のデータが壊れていないか確認する

uint8_t g_upBuf[UP_MAX_BYTES];
uint16_t g_upFrames = 0;
size_t g_upExpected = 0;
size_t g_upReceived = 0;
volatile bool g_upReady = false;
volatile bool g_playing = false;
portMUX_TYPE g_upMux = portMUX_INITIALIZER_UNLOCKED;

struct KeyFrame {
    uint16_t dur;
    uint8_t flags;
    int16_t joint[19];
    int16_t pose[2][5];   // X,Y,Z[0.1mm], Roll,Yaw[0.01deg]
};

void decodeFrame(const uint8_t* b, KeyFrame& k) {
    k.dur = rdU16(b);
    k.flags = b[2];
    for (int i = 0; i < 19; ++i) k.joint[i] = rdI16(b + 3 + i * 2);
    for (int f = 0; f < 2; ++f)
        for (int j = 0; j < 5; ++j) k.pose[f][j] = rdI16(b + 41 + f * 10 + j * 2);
}

inline bool framePose(const KeyFrame& k, int f) { return k.flags & (f == 0 ? KF_POSE_L : KF_POSE_R); }
inline uint8_t frameIk(const KeyFrame& k, int f) { return (k.flags & (f == 0 ? KF_IK_L : KF_IK_R)) ? 1 : 0; }

void poseFromFrame(const int16_t q[5], float v[5]) {
    v[0] = q[0] * 0.1f; v[1] = q[1] * 0.1f; v[2] = q[2] * 0.1f;
    v[3] = q[3] * 0.01f; v[4] = q[4] * 0.01f;
}
FootController::Pose poseFromVals(const float v[5]) {
    return FootController::Pose{v[0], v[1], v[2], v[3] * static_cast<float>(DEG_TO_RAD), 0.0f, v[4] * static_cast<float>(DEG_TO_RAD)};
}

// 再生状態（v3: 部位トラックごとに独立して進む）
//  トラック = 腰 / 左腕 / 右腕 / 左脚 / 右脚。各トラックは「自分の軸に値(-1以外)がある
//  フレーム」だけを順に再生する。全軸に値のあるフレームなら全トラックが同時に進むので、
//  従来の全身キーフレームと同じ動きになる。
constexpr int TRACKS = 5;
constexpr int8_t TRACK_FIRST[TRACKS] = {0, 1, 5, 9, 14};
constexpr int8_t TRACK_LEN[TRACKS] = {1, 4, 4, 5, 5};
constexpr int8_t TRACK_FOOT[TRACKS] = {-1, -1, -1, 0, 1};
struct TrackState {
    int16_t frame = -1;       // 再生中のフレーム番号（-1 = このトラックは終了）
    bool init = false;
    uint32_t startMs = 0;
    uint16_t dur = 20;
    bool snap = false;
    long start[5];
    long target[5];           // 論理, -1=触らない, 0=FREE
    bool cart = false;
    float from[5], to[5];
    uint8_t ik = 0;
    bool lastPoseValid = false;
    float lastPose[5];
};
TrackState g_tr[TRACKS];

bool trackUses(const KeyFrame& k, int t) {
    if (TRACK_FOOT[t] >= 0 && framePose(k, TRACK_FOOT[t])) return true;
    for (int j = 0; j < TRACK_LEN[t]; ++j) if (k.joint[TRACK_FIRST[t] + j] != -1) return true;
    return false;
}

int16_t trackNextFrame(int t, int from) {
    for (int fr = from; fr < g_upFrames; ++fr) {
        KeyFrame k;
        decodeFrame(g_upBuf + static_cast<size_t>(fr) * FRAME_BYTES, k);
        if (trackUses(k, t)) return static_cast<int16_t>(fr);
    }
    return -1;
}

uint32_t fnv1a32(const uint8_t* p, size_t n) {
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 16777619UL; }
    return h;
}

// 全フレームを検証（範囲外の関節値・IK到達不可の座標を弾く）。不正なフレーム番号を返す（全て正常なら -1）
int validateFrames(uint16_t frames) {
    for (uint16_t fr = 0; fr < frames; ++fr) {
        KeyFrame k;
        decodeFrame(g_upBuf + static_cast<size_t>(fr) * FRAME_BYTES, k);
        bool ok = k.dur >= 20 && (k.flags & ~0x1F) == 0;
        for (int i = 0; ok && i < 19; ++i) {
            const int f = (i >= 14) ? 1 : (i >= 9 ? 0 : -1);
            if (f >= 0 && framePose(k, f)) continue;   // 座標指定の脚の関節値は使わない
            const int16_t v = k.joint[i];
            if (v == -1 || v == 0 || v == -2) continue;     // -2 = 空白（そのトラックは保持）
            if (v < -2 || !logicalInRange(i, v)) ok = false;
        }
        for (int f = 0; ok && f < 2; ++f) {
            if (!framePose(k, f)) continue;
            float pv[5];
            poseFromFrame(k.pose[f], pv);
            long out[5];
            if (!footIk(f, poseFromVals(pv), frameIk(k, f), out)) ok = false;
        }
        if (!ok) return fr;
    }
    return -1;
}

void resetUploadLocked() {
    g_upFrames = 0;
    g_upExpected = 0;
    g_upReceived = 0;
    g_upReady = false;
}
} // namespace

uint8_t uploadBegin(uint16_t frames) {
    if (frames == 0 || frames > MAX_FRAMES) return ST_INVALID;
    uint8_t st = ST_OK;
    portENTER_CRITICAL(&g_upMux);
    if (g_playing) {
        st = ST_BUSY;
    } else {
        g_upFrames = frames;
        g_upExpected = static_cast<size_t>(frames) * FRAME_BYTES;
        g_upReceived = 0;
        g_upReady = false;
    }
    portEXIT_CRITICAL(&g_upMux);
    return st;
}

uint8_t uploadData(uint16_t offset, const uint8_t* d, size_t n) {
    uint8_t st = ST_REJECTED;
    portENTER_CRITICAL(&g_upMux);
    if (!g_playing && !g_upReady && g_upExpected > 0 && n > 0) {
        if (offset == g_upReceived && offset + n <= g_upExpected) {
            memcpy(g_upBuf + offset, d, n);
            g_upReceived += n;
            st = ST_OK;
        } else if (offset + n == g_upReceived) {
            st = ST_OK;   // ACKが失われてPCが再送した分（受信済み）
        }
    }
    portEXIT_CRITICAL(&g_upMux);
    return st;
}

bool uploadReady() { return g_upReady; }

uint8_t uploadCommit() {
    portENTER_CRITICAL(&g_upMux);
    const bool complete = !g_playing && g_upExpected > 0 && g_upReceived == g_upExpected;
    const uint16_t frames = g_upFrames;
    const size_t bytes = g_upExpected;
    portEXIT_CRITICAL(&g_upMux);
    if (!complete) return ST_BAD_STATE;

    const int bad = validateFrames(frames);
    if (bad >= 0) {
        Serial.printf("[MOTION] Upload rejected: frame %d is invalid (range/IK).\n", bad);
        portENTER_CRITICAL(&g_upMux);
        resetUploadLocked();
        portEXIT_CRITICAL(&g_upMux);
        return ST_INVALID;
    }

    g_upHash = fnv1a32(g_upBuf, bytes);
    g_upReady = true;
    Serial.printf("[MOTION] Upload committed (RAM only): %u frames.\n", frames);
    return ST_OK;
}

// ACKのtokenに入れる、内容の16bitハッシュ（PCが送った内容と一致するか確認するため）
uint16_t uploadHash16() { return static_cast<uint16_t>((g_upHash ^ (g_upHash >> 16)) & 0xFFFF); }

uint8_t playbackStart() {
    if (!g_upReady) return ST_BAD_STATE;
    // RAM上のデータがCOMMIT時のまま壊れていないことを確認してから再生する
    const uint16_t frames = g_upFrames;
    const size_t bytes = static_cast<size_t>(frames) * FRAME_BYTES;
    if (frames == 0 || frames > MAX_FRAMES || fnv1a32(g_upBuf, bytes) != g_upHash) return ST_IO_ERROR;
    for (int t = 0; t < TRACKS; ++t) {
        g_tr[t] = TrackState();
        g_tr[t].frame = trackNextFrame(t, 0);
    }
    g_playing = true;
    Serial.printf("[MOTION] Playback started: %u frames.\n", frames);
    return ST_OK;
}

void playbackAbort() {
    g_playing = false;
    for (int t = 0; t < TRACKS; ++t) g_tr[t].frame = -1;
}

bool playbackUpdate() {
    if (!g_playing) return false;
    bool any = false;
    for (int t = 0; t < TRACKS; ++t) {
        TrackState& s = g_tr[t];
        if (s.frame < 0) continue;
        any = true;
        KeyFrame k;
        decodeFrame(g_upBuf + static_cast<size_t>(s.frame) * FRAME_BYTES, k);
        const int first = TRACK_FIRST[t];
        const int len = TRACK_LEN[t];
        const int f = TRACK_FOOT[t];
        const bool pose = f >= 0 && framePose(k, f);

        if (!s.init) {
            s.startMs = millis();
            s.dur = k.dur < 20 ? 20 : k.dur;
            s.snap = k.flags & KF_SNAP;
            s.cart = false;
            for (int j = 0; j < len; ++j) {
                s.target[j] = k.joint[first + j];   // -2(空白)は「動かさない」: 下の補間/FREE処理はどれも >0 / ==0 だけを扱う
                s.start[j] = currentLogical(first + j);
            }
            if (pose) {
                poseFromFrame(k.pose[f], s.to);
                s.ik = frameIk(k, f);
                if (s.lastPoseValid && !s.snap) {
                    s.cart = true;                 // 足先を直線で動かす
                    memcpy(s.from, s.lastPose, sizeof(s.from));
                    for (int j = 0; j < len; ++j) s.target[j] = -1;
                } else {
                    long out[5];
                    const bool ok = footIk(f, poseFromVals(s.to), s.ik, out);
                    for (int j = 0; j < len; ++j) s.target[j] = ok ? out[j] : -1;
                }
            }
            for (int j = 0; j < len; ++j) {
                servoICS::Servo* sv = ServoArray[first + j];
                if (sv == nullptr) continue;
                if (s.target[j] == 0) sv->setPosFree();
                else if (s.snap && s.target[j] > 0) sv->setPos(s.target[j], true);
            }
            s.init = true;
        }

        const uint32_t elapsed = millis() - s.startMs;
        const float tt = (elapsed >= s.dur) ? 1.0f : static_cast<float>(elapsed) / static_cast<float>(s.dur);
        const float e = tt * tt * (3.0f - 2.0f * tt);
        if (!s.snap) {
            for (int j = 0; j < len; ++j) {
                const long tgt = s.target[j];
                if (tgt <= 0 || ServoArray[first + j] == nullptr) continue;
                const long st = s.start[j] > 0 ? s.start[j] : tgt;
                ServoArray[first + j]->setPos(lroundf(st + (tgt - st) * e), true);
            }
        }
        if (s.cart) {
            float v[5];
            for (int j = 0; j < 5; ++j) v[j] = s.from[j] + (s.to[j] - s.from[j]) * e;
            long out[5];
            if (footIk(f, poseFromVals(v), s.ik, out)) {
                for (int j = 0; j < 5; ++j) ServoArray[first + j]->setPos(out[j], true);
            }
        }

        if (elapsed >= s.dur) {
            if (f >= 0) {
                if (pose) {
                    memcpy(s.lastPose, s.to, sizeof(s.lastPose));
                    s.lastPoseValid = true;
                } else {
                    // 関節で動かしたら座標の連続性は切れる（-2の空白=保持なら連続のまま）
                    for (int j = 0; j < len; ++j) if (k.joint[first + j] >= 0) s.lastPoseValid = false;
                }
            }
            s.init = false;
            s.frame = trackNextFrame(t, s.frame + 1);
        }
    }
    if (!any) {
        g_playing = false;
        Serial.println("[MOTION] Uploaded motion completed.");
        return false;
    }
    return true;
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
    MOTION_STATIC_RESET(taskPhase, 0);
    static int frame = 0;
    MOTION_STATIC_RESET(frame, 0);

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
    MOTION_STATIC_RESET(taskPhase, 0);
    static int frame = 0;
    MOTION_STATIC_RESET(frame, 0);

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
        ServoArray[i+index]->setPosDeg(currentPos);   // 論理角（旧: 生角度 + delay(2)/軸）
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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int frame = 0;
        MOTION_STATIC_RESET(frame, 0);

        switch (taskPhase){
            case 0:
            {
                for(int i=0;i<19;i++){
                    ServoArray[i]->setStretch(axisStretch(i));
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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int frame = 0;
        MOTION_STATIC_RESET(frame, 0);

        switch (taskPhase){
            case 0:
            {
                for(int i=0;i<19;i++){
                    ServoArray[i]->setStretch(axisStretch(i));
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
            MOTION_STATIC_RESET(taskPhase, 0);
            static int frame = 0;
            MOTION_STATIC_RESET(frame, 0);

            //左右に移動
            static float moveValue;
            MOTION_STATIC_RESET(moveValue, 0);

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
            MOTION_STATIC_RESET(taskPhase, 0);
            static int frame = 0;
            MOTION_STATIC_RESET(frame, 0);

            //左右に移動
            static float moveValue;
            MOTION_STATIC_RESET(moveValue, 0);

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
            MOTION_STATIC_RESET(taskPhase, 0);
            static int frame = 0;
            MOTION_STATIC_RESET(frame, 0);

            switch (taskPhase){
                case 0:
                {
                    Serial.print(F("turn \n"));
                    for(int i=0;i<19;i++){
                        ServoArray[i]->setStretch(axisStretch(i));
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
            MOTION_STATIC_RESET(taskDelay, 0);

            static int taskPhase=0;

            MOTION_STATIC_RESET(taskPhase, 0);
            switch (taskPhase)
            {
            case 0:
            {
                FootController::Pose leftPose={
                NeutralPoseY,-NeutralSpac*0.3,NeutralPoseX,
                0, 0, 0
                };

                FootController::Pose rightPose={
                NeutralPoseY,+NeutralSpac*0.3,NeutralPoseX,
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
                if(taskDelay>(int)(G_fps*0.2)){//0.1秒間だけ固定
                    taskPhase++;
                    taskDelay=0;
                }
                break;
            }
            case 2:
            {
                taskDelay=0;

                FootController::Pose leftPose={
                NeutralPoseY+20,-NeutralSpac*0.3,NeutralPoseX,
                0, 0, 0
                };

                FootController::Pose rightPose={
                NeutralPoseY+20,+NeutralSpac*0.3,NeutralPoseX,
                0, 0, 0
                };

                // 逆運動学 (IK) を介して各足のサーボへ指令を出力
                leftFoot.setTargetPose(leftPose);
                rightFoot.setTargetPose(rightPose);
                
                ServoArray[0]->setPosDeg(15);
                
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
                NeutralPoseY,-NeutralSpac*0.3,NeutralPoseX,
                0, 0, 0
                };

                FootController::Pose rightPose={
                NeutralPoseY,+NeutralSpac*0.3,NeutralPoseX,
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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep = 0;
        MOTION_STATIC_RESET(currentStep, 0);
        static int arrIndex = 0;
        MOTION_STATIC_RESET(arrIndex, 0);

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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep = 0;
        MOTION_STATIC_RESET(currentStep, 0);
        static int arrIndex = 0;
        MOTION_STATIC_RESET(arrIndex, 0);

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        // 角度は論理角[deg]（オフセット差し引き後）。旧データ(生角度)は既定オフセットで変換済み。
        static const MotionStep motionData[] PROGMEM = {
            /* 0: posZero   */ {60, {0.00,4.000,80.00,-16.00,-130.00,00.00,70.00,33.000,-130.00,-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}},
            /* 1:    */        {0,  {0.00,-6.55,86.67,-9.11,-67.10,0.30,78.43,1.11,-2.60,-1.15,-18.83,70.03,-104.15,-7.29,12.86,-4.45,80.59,89.23,0.13}},
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
                // 全軸FREE（旧: 10回×19軸×delay(5) = 約1秒メインループ停止）
                for(int i=0;i<19;i++){
                    ServoArray[i]->setPosFree();
                }
                return false; // モーション完了
        }

        return true; // モーション継続中
    }

    bool standUp::motionLoop(){
        static int taskPhase = 0;
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep = 0;
        MOTION_STATIC_RESET(currentStep, 0);
        static int arrIndex = 0;
        MOTION_STATIC_RESET(arrIndex, 0);

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        // 角度は論理角[deg]（オフセット差し引き後）。旧データ(生角度)は既定オフセットで変換済み。
        static const MotionStep motionData[] PROGMEM = {
            /* 1:    */        {60,  {0.00,-6.55,86.67,-9.11,-67.10,0.30,78.43,1.11,-2.60,-1.15,-18.83,70.03,-104.15,-7.29,12.86,-4.45,80.59,89.23,0.13}},
            /* 0: posZero   */ { 0,  {0.00,4.000,80.00,-16.00,-130.00,00.00,70.00,33.000,-130.00,-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}},
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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep = 0;
        MOTION_STATIC_RESET(currentStep, 0);
        static int arrIndex = 0;
        MOTION_STATIC_RESET(arrIndex, 0);

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[10];
        };

        
        static const MotionStep motionData_1[] PROGMEM = {
            /* 0: posZero   */ {20, {-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}},
            /* 2:           */ {10,  {-3.14,-28.86,19.64,-49.55,7.94,13.16,19.44,47.31,40.93,-6.01}},
            /* 1:    */        {0,  {6.99,-32.20,21.93,-47.62,-26.93,20.99,21.70,36.75,38.81,21.36}},
        };

        const int arrNum_1 = sizeof(motionData_1) / sizeof(motionData_1[0]);

        static const MotionStep motionData_2[] PROGMEM = {
            /* 1:    */        {20,  {6.99,-32.20,21.93,-47.62,-26.93,20.99,21.70,36.75,38.81,21.36}},
            /* 2:           */ {10,  {-3.14,-28.86,19.64,-49.55,7.94,13.16,19.44,47.31,40.93,-6.01}},
            /* 0: posZero   */ {0,   {-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}},
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
        MOTION_STATIC_RESET(current_angle, 0.0f);
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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep=0;
        MOTION_STATIC_RESET(currentStep, 0);

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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep=0;
        MOTION_STATIC_RESET(currentStep, 0);

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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep=0;
        MOTION_STATIC_RESET(currentStep, 0);

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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep = 0;
        MOTION_STATIC_RESET(currentStep, 0);
        static int arrIndex = 0;
        MOTION_STATIC_RESET(arrIndex, 0);

        const int joint_num = 4;

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[joint_num];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        // 角度は論理角[deg]（オフセット差し引き後）。旧データ(生角度)は既定オフセットで変換済み。
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

        MOTION_STATIC_RESET(taskPhase, 0);
        static bool getup_mode;
        MOTION_STATIC_RESET(getup_mode, 0);

        switch (taskPhase){
            case 0:
            {
                Serial.printf("getUp!\n");
                for(int i=0;i<19;i++){
                    ServoArray[i]->setSkip(true);
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
                }
                return false;
                break;
            }
        }
        return true;
    }

    bool getUp_prone::motionLoop(){
        static int taskPhase = 0;
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep = 0;
        MOTION_STATIC_RESET(currentStep, 0);
        static int arrIndex = 0;
        MOTION_STATIC_RESET(arrIndex, 0);

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        // 角度は論理角[deg]（オフセット差し引き後）。旧データ(生角度)は既定オフセットで変換済み。
        static const MotionStep motionData[] PROGMEM = {
            /* 0: posZero   */ {30, {0.00,4.000,80.00,-16.00,-130.00,00.00,70.00,33.000,-130.00,-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}},
            /* 1:    */        {20, {0.64,-101.55,3.37,90.21,-34.96,-83.23,-2.84,89.24,-10.73,-4.49,-27.54,67.46,-100.00,-20.05,14.78,25.58,83.22,84.07,13.34}},
            /*              */ {20, {0.64,-91.83,-5.16,-6.21,-4.42,-88.69,3.00,165.68,1.92,7.32,-7.43,64.26,-104.15,-3.38,18.06,4.25,83.56,86.50,1.72}},
            /*              */ {120, {-9.99,-85.35,71.45,-0.47,-4.76,-93.22,-65.75,165.34,1.62,6.99,-7.43,64.59,-104.49,-3.67,18.06,3.92,83.22,86.80,1.41}},
            /* 6: posZero   */ { 0, {0.00,4.000,80.00,-16.00,-130.00,00.00,70.00,33.000,-130.00,-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}}
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
        MOTION_STATIC_RESET(taskPhase, 0);
        static int currentStep = 0;
        MOTION_STATIC_RESET(currentStep, 0);
        static int arrIndex = 0;
        MOTION_STATIC_RESET(arrIndex, 0);

        // モーション構造体定義
        struct MotionStep {
            int frameNum;
            float pose[19];
        };

        // PROGMEM配置（関数呼び出し間で保持するため static const を付与）
        // 角度は論理角[deg]（オフセット差し引き後）。旧データ(生角度)は既定オフセットで変換済み。
        static const MotionStep motionData[] PROGMEM = {
            /* 0: posZero   */ {30, {0.00,4.000,80.00,-16.00,-130.00,00.00,70.00,33.000,-130.00,-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}},
            /* 1: posSupine */ {30, {0.00,-71.42,100.95,58.42,-90.82,61.66,90.45,-51.30,-85.22,2.77,-10.12,-15.39,-16.27,-7.62,14.48,3.27,5.16,6.34,6.24}},
            /*    支援      */ {25, {0.00,-4.86,102.53,35.98,-33.65,-3.14,91.70,-52.28,43.67,-3.14,-81.71,13.97,-106.08,-79.72,14.14,77.52,31.82,86.50,74.86}}, 
            /* 2: posTar1   */ {25, {0.00,-56.40,1.45,9.25,-129.74,32.30,-3.51,9.25,-119.31,23.93,-96.76,14.98,-109.28,-87.42,11.88,88.80,11.03,96.82,91.67}},
            /* 3: posTar2   */ //{25, {0.00,35.98,30.21,46.37,0.81,-19.81,39.05,-57.81,6.51,3.41,-99.02,-18.67,-100.31,-87.42,8.94,89.44,-0.14,86.80,91.33}},
            /* 4: posTar3   */ {120, {0.00,-75.60,21.16,28.15,-75.63,29.67,23.12,31.39,-57.95,-11.47,-100.31,30.57,-108.31,-96.39,23.25,90.08,49.61,91.05,83.91}},
            /* 5: posTar4   */ {120, {0.00,-25.41,33.45,9.59,21.67,4.93,36.48,22.61,-15.63,2.77,-6.07,68.75,-60.52,-5.00,11.21,27.84,69.39,56.12,24.61}},
            /* 6: posZero   */ { 0, {0.00,4.000,80.00,-16.00,-130.00,00.00,70.00,33.000,-130.00,-0.81,-10.77,24.94,-34.36,-7.29,15.46,7.80,39.72,22.41,3.34}}
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

        MOTION_STATIC_RESET(count, 0);
        if(count>G_fps*4){
            for (int i = 0; i < 19; i++) {
                ServoArray[i]->setStretch(axisStretch(i));
            }
            count=0;
        }else{
            count++;
        }

        Serial.printf(".");
        return false;
    }

    // デバッグモード（PSボタン）: ○で FREE ⇔ その場で保持 を切り替え、×で終了。
    // 旧実装は while + delay(1500) でメインループごと止めていた（BLE応答・IMU・ACKも停止）。
    // 1周期ずつ進めるように変更。表示する角度は論理角（オフセット差し引き後）で、
    // 出力される {...} はそのまま motion_sub 用のモーションテーブルに貼れる。
    bool LOCK_DebugMode::motionLoop(){
        static int phase = 0;
        MOTION_STATIC_RESET(phase, 0);
        static bool freeMode = true;
        MOTION_STATIC_RESET(freeMode, true);
        static bool prevCircle = false;
        MOTION_STATIC_RESET(prevCircle, false);
        static uint32_t lastPrintMs = 0;
        MOTION_STATIC_RESET(lastPrintMs, 0);

        switch (phase) {
        case 0:
            Serial.printf("DebugMode! comand!\n");
            freeMode = true;
            prevCircle = Dualshock4.data.button.circle;
            for (int i = 0; i < 19; i++) ServoArray[i]->setPosFree();
            lastPrintMs = millis();
            phase = 1;
            return true;
        case 1: {
            if (Dualshock4.data.button.cross) {
                phase = 0;
                return false;
            }
            const bool circle = Dualshock4.data.button.circle;
            if (!prevCircle && circle) {
                freeMode = !freeMode;
                for (int i = 0; i < 19; i++) {
                    long raw;
                    if (freeMode) ServoArray[i]->setPosFree();
                    else if (servoBus::getFb(i, raw, 500)) ServoArray[i]->setPos(raw, false);  // 今の位置で保持
                }
            }
            prevCircle = circle;
            servoBus::requestFb(ALL_AXIS_MASK);

            const uint32_t interval = freeMode ? 1500 : 1000;
            if (millis() - lastPrintMs >= interval) {
                lastPrintMs = millis();
                Serial.print(freeMode ? "DebugMode(FREE) {" : "DebugMode(HOLD) {");
                for (int i = 0; i < 19; i++) {
                    long raw;
                    if (i != 0) Serial.print(",");
                    if (servoBus::getFb(i, raw, 1000)) {
                        Serial.print(servoICS::fromIcs_toDeg(ServoArray[i]->toLogicalIcs(raw)));
                    } else {
                        Serial.print("nan");
                    }
                }
                Serial.println("}");
            }
            return true;
        }
        default:
            phase = 0;
            return false;
        }
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

        MOTION_STATIC_RESET(taskDelay, 0);

        static int taskPhase=0;

        MOTION_STATIC_RESET(taskPhase, 0);
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


    
    // ServoStateDeBug / TeachingMode は RobotController（テレメトリ）と下記の教示処理に置き換えた。

}

// ==============================================================
// 教示(TEACH / 同期)モード
//  - BLE callback: 最新値優先のメールボックスへ置くだけ（軸指令は未処理分をマージ）。
//  - loop: 指令を論理値で setPos() するだけ。滑らかさは ServoBus のスムージング（TEACH中ON）が担う。
//  - 座標指令はマイコン側でIK。到達不可/範囲外なら直前の姿勢を保持し、エラー軸として通知。
// ==============================================================
namespace {
struct TeachAxis { uint32_t cmd; uint32_t fb; uint16_t v[19]; };
struct TeachPose { float v[6]; uint8_t ik; };   // X,Y,Z[mm], Roll,Pitch,Yaw[rad]

portMUX_TYPE g_tMux = portMUX_INITIALIZER_UNLOCKED;
TeachAxis g_tAxis = {};
bool g_tAxisValid = false;
TeachPose g_tPose[2] = {};
bool g_tPoseValid[2] = {false, false};
uint32_t g_tFbMask = 0;
uint32_t g_tErr = 0;

void teachClear() {
    portENTER_CRITICAL(&g_tMux);
    g_tAxisValid = false;
    g_tPoseValid[0] = g_tPoseValid[1] = false;
    portEXIT_CRITICAL(&g_tMux);
    g_tFbMask = 0;
    g_tErr = 0;
}

bool teachApplyPose(int f, const TeachPose& c) {
    const FootController::Pose pose = {c.v[0], c.v[1], c.v[2], c.v[3], c.v[4], c.v[5]};
    long out[5];
    if (!footIk(f, pose, c.ik, out)) {
        g_tErr |= FOOT_MASK[f];   // 到達不可/範囲外。直前の姿勢を保持
        return false;
    }
    for (int k = 0; k < 5; ++k) ServoArray[FOOT_BASE[f] + k]->setPos(out[k], true);
    g_tErr &= ~FOOT_MASK[f];
    return true;
}
} // namespace

bool teachPostAxis(const uint8_t* p, size_t len) {
    if (p == nullptr || len != 44) return false;    // [cmd:3][fb:3][19*u16]
    const uint32_t cmd = rdU24(p) & ALL_AXIS_MASK;
    const uint32_t fb = rdU24(p + 3) & ALL_AXIS_MASK;
    portENTER_CRITICAL(&g_tMux);
    if (!g_tAxisValid) {
        g_tAxis.cmd = 0;
        g_tAxisValid = true;
    }
    for (int i = 0; i < 19; ++i) {
        if (cmd & (1UL << i)) g_tAxis.v[i] = rdU16(p + 6 + i * 2);
    }
    g_tAxis.cmd |= cmd;
    g_tAxis.fb = fb;
    portEXIT_CRITICAL(&g_tMux);
    return true;
}

bool teachPostPose(const uint8_t* p, size_t len) {
    if (p == nullptr || len != 26 || p[0] > 1) return false;   // [foot][ik][6*f32]
    TeachPose c;
    c.ik = p[1] ? 1 : 0;
    memcpy(c.v, p + 2, sizeof(c.v));
    for (int i = 0; i < 6; ++i) if (!std::isfinite(c.v[i])) return false;
    for (int i = 0; i < 3; ++i) if (fabsf(c.v[i]) > 400.0f) return false;
    if (fabsf(c.v[3]) > 1.6f || fabsf(c.v[5]) > 1.6f) return false;
    c.v[4] = 0.0f;   // Pitchは現行IKでは未使用
    portENTER_CRITICAL(&g_tMux);
    g_tPose[p[0]] = c;
    g_tPoseValid[p[0]] = true;
    portEXIT_CRITICAL(&g_tMux);
    return true;
}

void teachEnter() { teachClear(); }
void teachExit() { teachClear(); }
uint32_t teachErrorMask() { return g_tErr; }

bool teachNeutral(uint8_t f) {
    if (f > 1) return false;
    TeachPose n;
    // 通常の pose モーションと同じ値（左右差補正 footPoseDef を含む）
    n.v[0] = NeutralPoseY + (f == 0 ? footPoseDefLeft : footPoseDefRight);   // X(上下)
    n.v[1] = (f == 0) ? -NeutralSpac : NeutralSpac;                           // Y(左右)
    n.v[2] = NeutralPoseX;                                                    // Z(前後)
    n.v[3] = n.v[4] = n.v[5] = 0.0f;
    n.ik = 0;
    const bool ok = teachApplyPose(f, n);
    uint8_t e[25];
    e[0] = f;
    memcpy(e + 1, n.v, sizeof(n.v));
    sendPacket(proto::POSE_ECHO, e, sizeof(e));
    return ok;
}

void teachUpdate() {
    TeachAxis ax;
    bool hasAxis = false;
    TeachPose pose[2];
    bool hasPose[2] = {false, false};
    portENTER_CRITICAL(&g_tMux);
    if (g_tAxisValid) { ax = g_tAxis; hasAxis = true; g_tAxisValid = false; }
    for (int f = 0; f < 2; ++f) {
        if (g_tPoseValid[f]) { pose[f] = g_tPose[f]; hasPose[f] = true; g_tPoseValid[f] = false; }
    }
    portEXIT_CRITICAL(&g_tMux);

    if (hasAxis) {
        g_tFbMask = ax.fb;
        for (int i = 0; i < 19; ++i) {
            const uint32_t bit = (1UL << i);
            if (!(ax.cmd & bit)) continue;
            servoICS::Servo* s = ServoArray[i];
            if (s == nullptr) { g_tErr |= bit; continue; }
            if (ax.v[i] == 0) {
                s->setPosFree();
                g_tErr &= ~bit;
            } else {
                s->setPos(ax.v[i], true);   // 論理値。範囲外ならservoICSが拒否（直前の目標を保持）
                if (s->returnStatus().success) g_tErr &= ~bit; else g_tErr |= bit;
            }
        }
    }
    for (int f = 0; f < 2; ++f) {
        if (hasPose[f]) teachApplyPose(f, pose[f]);
    }
    servoBus::requestFb(g_tFbMask);
}
