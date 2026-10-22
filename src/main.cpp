#include "Declaration.h"
#include "AssistiveProgram.h"
#include "Config.h"
#include "Motion.h"
#include <Wire.h>

#include "BLEManager.h"

// Motion.cppで定義される教示制御状態。BLEの実装詳細はBLEManager.cppに隔離する。
extern volatile bool g_teachCmdPending;
extern volatile bool g_teachModeActive;
extern bool uploadedMotionOwnsRobot();
extern bool uploadedMotionEmergencyLatched();
extern void processUploadedMotion();

// MPU6886 I2C設定
#define MPU6886_ADDRESS 0x68
#define SDA_PIN 25
#define SCL_PIN 21

// ローパスフィルタの平滑化係数 (0.0 < LPF_ALPHA <= 1.0)
// 値を小さくするほど振動ノイズに強くなります
static const float LPF_ALPHA = 0.2f;

// 内部ヘルパー関数：I2Cレジスタ書き込み
static void writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6886_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

bool setupIMU() {
    Wire.begin(SDA_PIN, SCL_PIN, 400000); // 400kHz Fast Mode
    delay(50);

    // WHO_AM_I レジスタの検証 (0x75 -> 0x19)
    Wire.beginTransmission(MPU6886_ADDRESS);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6886_ADDRESS, (size_t)1);
    if (Wire.read() != 0x19) {
        return false; // 通信エラーまたはデバイス不一致
    }

    writeRegister(0x6B, 0x00); // スリープ解除
    delay(10);
    writeRegister(0x1C, 0x10); // ACCEL_CONFIG: ±8g (4096 LSB/g)
    writeRegister(0x1A, 0x03); // 内蔵DLPF (~42Hz) でハードウェア帯域制限

    return true;
}


void updateIMU() {
    Wire.beginTransmission(MPU6886_ADDRESS);
    Wire.write(0x3D);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6886_ADDRESS, (size_t)2);

    int16_t rawAccY = (Wire.read() << 8) | Wire.read();
    float rawG = (float)rawAccY / 4096.0f;

    // 1次ローパスフィルタ（LPF）演算
    g_lpfAcc = (1.0f - LPF_ALPHA) * g_lpfAcc + LPF_ALPHA * rawG;

    // ヒステリシス付き表裏判定
    if (g_lpfAcc > 0.1f) {
        g_isFaceUp = true;   // 表向き
    } else if (g_lpfAcc < -0.1f) {
        g_isFaceUp = false;  // 裏向き
    }
}


void taskManager();

void loop2(void *p);

void loop2_begin(){
    xTaskCreatePinnedToCore(
        loop2,           // タスク関数名
        "Loop2Task",     // タスク名（デバッグ用文字列）
        4096,            // スタックサイズ（バイト単位。必要に応じて調整）
        NULL,            // タスク引数
        1,               // 優先度（1〜24、数値が大きいほど優先）
        NULL,            // タスクハンドル
        0                // 割り当てるコア番号（0 または 1）
    );
}

inline void checkMemoryUsage(unsigned long intervalMs = 1000) {
    static unsigned long lastCheck = 0;
    unsigned long now = millis();

    // 指定インターバル（デフォルト1000ms）経過していない場合は即復帰（極小負荷）
    if (now - lastCheck < intervalMs) {
        return;
    }
    lastCheck = now;

    uint32_t total = ESP.getHeapSize();
    uint32_t free = ESP.getFreeHeap();
    uint32_t minFree = ESP.getMinFreeHeap();
    uint32_t used = total - free;
    float usageRatio = ((float)used / (float)total) * 100.0f;

    // 1行で要点をまとめた軽量シリアル出力
    Serial.printf("[RAM] Used: %u B (%.1f%%) | Free: %u B | MinFree: %u B\n", 
                  used, usageRatio, free, minFree);

    // メモリ危険域（20KB以下）の警告
    if (free < 20000) {
        Serial.printf("[WARN] Critical Low RAM! Free: %u B\n", free);
    }
}


static char g_usingPartsBit = 0;

bool runExclusiveTask(motionBase* motion) {
    if (motion->isActive) {
        // 実行中の場合：モーションを継続し、終了したら解放
        if (!motion->motionLoop()) {
            motion->isActive = false;
            g_usingPartsBit &= ~(motion->motionArrUsingBit);
        }
        return true;
    } 
    else if (motion->canStart() && ((g_usingPartsBit & motion->motionArrUsingBit) == 0)) {
        // 停止中で、リソースが空いている場合：起動条件を満たしていれば開始
        motion->isActive = true;
        g_usingPartsBit |= motion->motionArrUsingBit;
        
        // 初回実行
        if (!motion->motionLoop()) {
            motion->isActive = false;
            g_usingPartsBit &= ~(motion->motionArrUsingBit);
        }
        return true;
    }
    return false; // 他のタスクが占有中、または起動条件を満たさないため実行不可
}

void taskManager() {
    static bool wasTeachingMode = false;

    // PCから全フレーム受信済みのモーションを最優先で実行する。
    // 再生中は既存モーションを停止して全軸を占有し、BLEへ1フレームずつ送らない。
    if (uploadedMotionOwnsRobot()) {
        for (int i = 0; i < motionList::motionArrNum; ++i) {
            motionList::motionArr[i]->isActive = false;
        }
        g_usingPartsBit = 0;
        wasTeachingMode = false;
        processUploadedMotion();
        return;
    }

    // 緊急停止後は自律モーションをラッチ停止。PCが明示的にTX教示を開始すれば手動制御は許可する。
    if (uploadedMotionEmergencyLatched()) {
        for (int i = 0; i < motionList::motionArrNum; ++i) {
            motionList::motionArr[i]->isActive = false;
        }
        g_usingPartsBit = 0;
        wasTeachingMode = false;
        if (g_teachModeActive && deviceConnected) {
            motion::TeachingMode_inst.motionLoop();
        } else if (!g_teachModeActive && deviceConnected) {
            // 緊急停止ラッチ中でもRX表示用テレメトリは継続する。
            // 通常の自律モーションは再開しない。
            motion::ServoStateDeBug_inst.motionLoop();
        }
        return;
    }

    // 教示中は他のモーションを停止・抑止し、毎周期ノンブロッキングで教示処理を進める。
    if (g_teachModeActive && deviceConnected) {
        if (!wasTeachingMode) {
            for (int i = 0; i < motionList::motionArrNum; ++i) {
                motionList::motionArr[i]->isActive = false;
            }
            g_usingPartsBit = 0;
            wasTeachingMode = true;
        }

        // 指令の有無に関わらず毎周期呼ぶ（スムージング出力・FB・応答送信を継続するため）。
        motion::TeachingMode_inst.motionLoop();
        return; // この間は通常モーションと定期テレメトリを動かさない。
    }

    if (wasTeachingMode) {
        for (int i = 0; i < motionList::motionArrNum; ++i) {
            motionList::motionArr[i]->isActive = false;
        }
        g_usingPartsBit = 0;
        wasTeachingMode = false;
    }

    for (int i = 0; i < motionList::motionArrNum; ++i) {
        runExclusiveTask(motionList::motionArr[i]);
    }
}

FrameLimiter framelim;

void setup() {
    Serial.begin(serialPC_bps);
    Serial1.begin(serialServo_bps, SERIAL_8E1, rxPin, txPin);
    // 返り値がないサーボ1軸あたりの待ち時間。ICSの返信は1ms程度で来るので10msで十分。
    // （旧100msだと、応答しない軸のFBを読むたびに制御周期が100ms止まっていた）
    Serial1.setTimeout(10);

    if (!setupIMU()) {
        Serial.println("IMU Initialization Failed!");
        while (1) delay(100);
    }

    loop2_begin();
    delay(500);

    // ★ 1. 先に PS4 コントローラ（Classic BT）の接続待ち受けを開始
    Dualshock4.begin(Config::ControllerMac);
    bondReset();

    Serial.println("Waiting for PS4 Controller...");
    while (1) {
        if (Dualshock4.isConnected()) {
            Dualshock4.setLed(255, 255, 255);
            Dualshock4.setRumble(0, 255);
            Dualshock4.sendToController();
            resetRumble(500);
            break;
        }
        delay(10);
    }
    Serial.println("PS4 Controller Connected!");

    // ファイルシステムを先に初期化し、BLE接続直後にモーション転送が来ても
    // LittleFSが未マウントの状態でCOMMITされないようにする。
    LittleFS_ini();
    listFiles();

    // ★ 2. PS4接続とLittleFS初期化の完了後にBLE広告を開始
    setupBLE();

    #ifndef SIMULATION

        short ICS_senter[19] = {
            7500, // [0] hipServo
            7500, // [1] leftArmJ1
            7500, // [2] leftArmJ2
            7500, // [3] leftArmJ3
            7500, // [4] leftArmJ4
            7500, // [5] rightArmJ1
            7500, // [6] rightArmJ2
            7500, // [7] rightArmJ3
            7500, // [8] rightArmJ4
            7500, // [9] leftFoot1
            7500, // [10] leftFoot2
            7500, // [11] leftFoot3
            7500, // [12] leftFoot4
            7500, // [13] leftFoot5
            7500, // [14] rightFoot1
            7500, // [15] rightFoot2
            7500, // [16] rightFoot3
            7500, // [17] rightFoot4
            7500 // [18] rightFoot5
        };

    short ServoArray_pos[19] = {
        7500, // [0] hipServo
        7500, // [1] leftArmJ1
        7500, // [2] leftArmJ2
        7500, // [3] leftArmJ3
        7500, // [4] leftArmJ4
        7500, // [5] rightArmJ1
        7500, // [6] rightArmJ2
        7500, // [7] rightArmJ3
        7500, // [8] rightArmJ4
        7500, // [9] leftFoot1
        7676, // [10] leftFoot2
        7710, // [11] leftFoot3
        7777, // [12] leftFoot4
        7600, // [13] leftFoot5
        7500, // [14] rightFoot1
        7575, // [15] rightFoot2
        7299, // [16] rightFoot3
        7299, // [17] rightFoot4
        7720 // [18] rightFoot5
    };

        leftFoot.FootMotorInvert(1, 1, -1, -1, 1);
        rightFoot.FootMotorInvert(1, 1, 1, 1, 1);

        for (int i = 0; i < 19; i++) {
            ServoArray[i]->setOffsetIcs(ServoArray_pos[i]);
        }
        for (int i = 0; i < 19; i++) {
            ServoArray[i]->setStretch(stretch);
            delay(5);
        }
        ServoArray[13]->setStretch(10);
        delay(5);
        ServoArray[18]->setStretch(10);
        delay(5);
        ServoArray[0]->setSkip(true);
    #endif

    Dualshock4.setLed(255, 0, 0);
    Dualshock4.sendToController();

    framelim.setInterval(1000 / G_fps);
}

void loop() {
    taskManager();

    
    updateIMU();

    // グローバル変数を参照して動作分岐
    if (g_isFaceUp) {
        Serial.println(">Status:1");  // 表向き
    } else {
        Serial.println(">Status:-1"); // 裏向き
    }
    
    // BLE接続状態の監視と切断後のアドバタイズ再開はBLEモジュールに委譲。
    updateBLE();

    Serial.println(framelim.sync());
}

namespace loop2_task
{
    // resetRumble
    volatile unsigned long resetRumble_time=0;
    volatile unsigned long startTime = 0;
    volatile bool resetRumble_flag=false;
}

void resetRumble(long time_ms) {
    loop2_task::startTime = millis();
    loop2_task::resetRumble_time = time_ms;
    loop2_task::resetRumble_flag = true;
}

void loop2(void *p){
    while (true) {
        //[resetRumble]タスク
        // フラグが立っており、指定された時間が経過したか判定
        if (loop2_task::resetRumble_flag) {
            if (millis() - loop2_task::startTime >= loop2_task::resetRumble_time) {
                // 1. フラグを降ろす（2重実行防止）
                loop2_task::resetRumble_flag = false;

                // 2. コントローラーの振動をリセットして送信
                Dualshock4.setRumble(0, 0);
                Dualshock4.sendToController();
            }
        }
        checkMemoryUsage();

        delay(1);
    }
}