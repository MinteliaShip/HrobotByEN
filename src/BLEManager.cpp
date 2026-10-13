#include "BLEManager.h"

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

// GATT callbackからnotify()を直接呼ばない。ACKを短いメールボックスに保存し、
// loop()側のupdateBLE()から通常テレメトリと同じ実行コンテキストで送る。
static portMUX_TYPE ackMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool ackPending = false;
static uint8_t ackOpcode = 0;
static uint8_t ackAccepted = 0;
static uint16_t ackToken = 0;

static void queueMotionAck(uint8_t opcode, bool accepted, uint16_t token) {
    portENTER_CRITICAL(&ackMux);
    ackOpcode = opcode;
    ackAccepted = accepted ? 1 : 0;
    ackToken = token;
    ackPending = true;
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
    }
};

// BLE callback内ではサーボUARTを操作しない。受信フレームをMotion側へ渡すだけ。
class RxCallbacks final : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* characteristic) override {
        const std::string rxValue = characteristic->getValue();
        if (rxValue.empty()) return;

        // 0x02: 教示モード解除。
        if (rxValue.length() == 1 && static_cast<uint8_t>(rxValue[0]) == 0x02) {
            stopTeachingMode();
            Serial.println("[BLE RX] Teaching mode released by PC.");
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
            const uint16_t token = (rxValue.length() >= 3)
                ? static_cast<uint16_t>(static_cast<uint8_t>(rxValue[1]) |
                    (static_cast<uint16_t>(static_cast<uint8_t>(rxValue[2])) << 8))
                : 0;
            queueMotionAck(opcode, accepted, token);
            return;
        }

        // 現行プロトコル45B: [0x01][CommandMask:3B][FeedbackMask:3B][Angles:19*2B]
        if (rxValue.length() == 45 && static_cast<uint8_t>(rxValue[0]) == 0x01) {
            if (queueTeachingCommand(
                    reinterpret_cast<const uint8_t*>(rxValue.data()), rxValue.length())) {
                Serial.println("[BLE RX] Teaching frame queued (45 bytes).");
            } else {
                Serial.println("[BLE RX] Teaching frame rejected: previous frame is busy.");
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

            if (queueTeachingCommand(frame, sizeof(frame))) {
                Serial.printf("[BLE RX] Legacy servo %u command queued.\n", id);
            } else {
                Serial.println("[BLE RX] Legacy command rejected: previous frame is busy.");
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
    // Motionコマンドの論理ACKをメインループから送信する。
    uint8_t ack[5] = {0x35, 0, 0, 0, 0};
    bool hasAck = false;
    portENTER_CRITICAL(&ackMux);
    if (ackPending) {
        ack[1] = ackOpcode;
        ack[2] = ackAccepted;
        ack[3] = static_cast<uint8_t>(ackToken & 0xFF);
        ack[4] = static_cast<uint8_t>((ackToken >> 8) & 0xFF);
        ackPending = false;
        hasAck = true;
    }
    portEXIT_CRITICAL(&ackMux);
    if (hasAck) sendBLE(ack, sizeof(ack));

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
