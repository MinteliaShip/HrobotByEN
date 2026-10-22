#include "BLEManager.h"
#include "Declaration.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

// Motion.cppで実装：BLE callbackから受け取った指令をバッファへ渡す。
// 外部リンケージを維持するため、匿名namespaceの外で宣言する。
extern bool queueTeachingCommand(const uint8_t* data, size_t length);
extern bool queueTeachingPose(const uint8_t* data, size_t length);
extern bool queueTeachingNeutral(uint8_t foot);
extern void startTeachingMode();
extern void stopTeachingMode();
extern bool handleUploadedMotionCommand(const uint8_t* data, size_t length);

// Nordic UART Service UUID
namespace {
constexpr char SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char CHARACTERISTIC_UUID_RX[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // PC -> ESP32
constexpr char CHARACTERISTIC_UUID_TX[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // ESP32 -> PC

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool oldDeviceConnected = false;
volatile bool offsetRequested = false; // 0x42受信 → updateBLE()でオフセット表を送る

// GATT callbackからnotify()を直接呼ばない。ACKをFIFOへ積み、
// loop()側のupdateBLE()から通常テレメトリと同じ実行コンテキストで送る。
// 緊急停止がデータ転送ACKと重なっても、後から来たACKで前のACKを上書きしない。
static portMUX_TYPE ackMux = portMUX_INITIALIZER_UNLOCKED;
static constexpr uint8_t ACK_QUEUE_CAPACITY = 16;
struct MotionAckRecord {
    uint8_t opcode;
    uint8_t accepted;
    uint16_t token;
};
static MotionAckRecord ackQueue[ACK_QUEUE_CAPACITY] = {};
static uint8_t ackHead = 0;
static uint8_t ackTail = 0;
static uint8_t ackCount = 0;

static void queueMotionAck(uint8_t opcode, bool accepted, uint16_t token) {
    portENTER_CRITICAL(&ackMux);
    if (ackCount < ACK_QUEUE_CAPACITY) {
        ackQueue[ackTail].opcode = opcode;
        ackQueue[ackTail].accepted = accepted ? 1 : 0;
        ackQueue[ackTail].token = token;
        ackTail = static_cast<uint8_t>((ackTail + 1) % ACK_QUEUE_CAPACITY);
        ++ackCount;
    } else {
        // FIFO満杯時は末尾レコードを最新結果で更新する。
        // 通常のPC実装はACK確認ごとに次パケットを送るため、非常時のみ到達する。
        const uint8_t last = static_cast<uint8_t>((ackTail + ACK_QUEUE_CAPACITY - 1) % ACK_QUEUE_CAPACITY);
        ackQueue[last].opcode = opcode;
        ackQueue[last].accepted = accepted ? 1 : 0;
        ackQueue[last].token = token;
    }
    portEXIT_CRITICAL(&ackMux);
}

class ServerCallbacks final : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* server) override {
        (void)server;
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* server) override {
        (void)server;
        deviceConnected = false;
        stopTeachingMode();
        portENTER_CRITICAL(&ackMux);
        ackHead = 0;
        ackTail = 0;
        ackCount = 0;
        portEXIT_CRITICAL(&ackMux);
    }
};

// 拒否ログは連打されるので1秒に1回まで。
static void logRejected(const char* msg) {
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if (now - lastMs >= 1000) {
        lastMs = now;
        Serial.printf("[BLE RX] %s\n", msg);
    }
}

// BLE callback内ではサーボUARTを操作しない。受信フレームをMotion側へ渡すだけ。
class RxCallbacks final : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* characteristic) override {
        const std::string rxValue = characteristic->getValue();
        if (rxValue.empty()) return;

        // RX/TXモードは明示的なコマンド＋論理ACKで切り替える。
        // 0x02=TX解除(RXへ)、0x03=TX開始。
        if (rxValue.length() == 1 && static_cast<uint8_t>(rxValue[0]) == 0x02) {
            stopTeachingMode();
            queueMotionAck(0x02, true, 0);
            Serial.println("[BLE RX] TX teaching mode released by PC.");
            return;
        }
        if (rxValue.length() == 1 && static_cast<uint8_t>(rxValue[0]) == 0x03) {
            startTeachingMode();
            queueMotionAck(0x03, true, 0);
            Serial.println("[BLE RX] TX teaching mode enabled by PC.");
            return;
        }

        // 事前転送モーションプロトコル (0x30..0x34)。動作中のサーボ制御はmain loop側で実行。
        const uint8_t opcode = static_cast<uint8_t>(rxValue[0]);
        if (opcode >= 0x30 && opcode <= 0x34) {
            const bool accepted = handleUploadedMotionCommand(
                reinterpret_cast<const uint8_t*>(rxValue.data()), rxValue.length());
            // DATAチャンクごとの大量ログはBLE転送を遅くするため、通常はログを省略。
            if (!accepted) {
                Serial.printf("[BLE RX] Uploaded-motion command 0x%02X rejected.\n", opcode);
            } else if (opcode != 0x31) {
                Serial.printf("[BLE RX] Uploaded-motion command 0x%02X accepted (%u bytes).\n", opcode, static_cast<unsigned>(rxValue.length()));
            }

            // ATT write success is not enough: the PC also needs the firmware's logical result.
            uint16_t token = 0;
            // 0x30はフレーム数、0x31はデータオフセットをACK tokenに含める。
            // 0x34の3byte軸マスクはtokenではない。
            if ((opcode == 0x30 || opcode == 0x31) && rxValue.length() >= 3) {
                token = static_cast<uint16_t>(static_cast<uint8_t>(rxValue[1]) |
                    (static_cast<uint16_t>(static_cast<uint8_t>(rxValue[2])) << 8));
            }
            queueMotionAck(opcode, accepted, token);
            return;
        }

        // 現行プロトコル45B: [0x01][CommandMask:3B][FeedbackMask:3B][Angles:19*2B]
        // 高頻度に届くため、受理時のログは出さない（BLEタスクの遅延を避ける）。
        if (rxValue.length() == 45 && static_cast<uint8_t>(rxValue[0]) == 0x01) {
            if (!queueTeachingCommand(
                    reinterpret_cast<const uint8_t*>(rxValue.data()), rxValue.length())) {
                logRejected("Teaching frame rejected: not in TX mode.");
            }
            return;
        }

        // 座標(IK)指令: [0x40][foot][mode][X,Y,Z,Roll,Pitch,Yaw: float32 LE x6] = 27B
        if (rxValue.length() == 27 && static_cast<uint8_t>(rxValue[0]) == 0x40) {
            if (!queueTeachingPose(
                    reinterpret_cast<const uint8_t*>(rxValue.data()), rxValue.length())) {
                logRejected("Pose command rejected: not in TX mode or invalid value.");
            }
            return;
        }

        // オフセット表要求: [0x42] → [0x39][19*int16 LE]（各サーボの setOffset 値）
        if (rxValue.length() == 1 && static_cast<uint8_t>(rxValue[0]) == 0x42) {
            offsetRequested = true;
            return;
        }

        // ニュートラル座標要求: [0x41][foot]
        if (rxValue.length() == 2 && static_cast<uint8_t>(rxValue[0]) == 0x41) {
            if (!queueTeachingNeutral(static_cast<uint8_t>(rxValue[1]))) {
                logRejected("Neutral request rejected: not in TX mode.");
            }
            return;
        }

        // 旧形式3B単軸指令も、45Bフレームへ変換して共通処理に渡す。
        if (rxValue.length() == 3) {
            const uint8_t id = static_cast<uint8_t>(rxValue[0]);
            if (id >= 19) {
                Serial.println("[BLE RX] Invalid legacy servo ID.");
                return;
            }

            uint8_t frame[45] = {0};
            frame[0] = 0x01;
            frame[1 + (id / 8)] = static_cast<uint8_t>(1U << (id % 8));
            const uint16_t ics = static_cast<uint8_t>(rxValue[1]) |
                (static_cast<uint16_t>(static_cast<uint8_t>(rxValue[2])) << 8);
            const size_t angleOffset = 7 + (static_cast<size_t>(id) * 2);
            frame[angleOffset] = static_cast<uint8_t>(ics & 0xFF);
            frame[angleOffset + 1] = static_cast<uint8_t>((ics >> 8) & 0xFF);

            if (!queueTeachingCommand(frame, sizeof(frame))) {
                logRejected("Legacy command rejected: not in TX mode.");
            }
            return;
        }

        Serial.printf("[BLE RX] Unsupported packet length: %u\n",
                      static_cast<unsigned>(rxValue.length()));
    }
};
} // namespace

volatile bool deviceConnected = false;

void setupBLE() {
    BLEDevice::init("ESP32_Robot");
    BLEDevice::setMTU(64);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* service = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic* rxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rxCharacteristic->setCallbacks(new RxCallbacks());

    service->start();
    pServer->getAdvertising()->start();
    oldDeviceConnected = false;
    Serial.println("BLE UART Ready.");
}

void sendBLE(uint8_t* data, size_t length) {
    if (!deviceConnected || pTxCharacteristic == nullptr || data == nullptr) return;
    // ATT MTU 64の場合のNotificationペイロード上限(64 - 3)。
    if (length == 0 || length > 61) return;

    pTxCharacteristic->setValue(data, length);
    pTxCharacteristic->notify();
}

void updateBLE() {
    // Motionコマンドの論理ACKをFIFOから1件ずつ送る。
    uint8_t ack[5] = {0x35, 0, 0, 0, 0};
    bool hasAck = false;
    portENTER_CRITICAL(&ackMux);
    if (ackCount > 0) {
        const MotionAckRecord record = ackQueue[ackHead];
        ackHead = static_cast<uint8_t>((ackHead + 1) % ACK_QUEUE_CAPACITY);
        --ackCount;
        ack[1] = record.opcode;
        ack[2] = record.accepted;
        ack[3] = static_cast<uint8_t>(record.token & 0xFF);
        ack[4] = static_cast<uint8_t>((record.token >> 8) & 0xFF);
        hasAck = true;
    }
    portEXIT_CRITICAL(&ackMux);
    if (hasAck) sendBLE(ack, sizeof(ack));

    if (offsetRequested && deviceConnected) {
        offsetRequested = false;
        uint8_t pkt[39];
        pkt[0] = 0x39;
        for (int i = 0; i < 19; ++i) {
            const long ofs = (ServoArray[i] != nullptr) ? ServoArray[i]->getOffset().value : 7500;
            pkt[1 + i * 2] = static_cast<uint8_t>(ofs & 0xFF);
            pkt[2 + i * 2] = static_cast<uint8_t>((ofs >> 8) & 0xFF);
        }
        sendBLE(pkt, sizeof(pkt));
    }

    // BLE切断後に再広告し、再接続できるようにする。
    if (!deviceConnected && oldDeviceConnected) {
        delay(100);
        if (pServer != nullptr) pServer->startAdvertising();
        Serial.println("[BLE] Advertising restarted.");
        oldDeviceConnected = false;
    } else if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true;
        Serial.println("[BLE] Client connected.");
    }
}
