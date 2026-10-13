#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// BLE接続状態。Motion.cpp / main.cpp から参照する。
extern volatile bool deviceConnected;

// ESP32_Robot のNordic UART Serviceを初期化する。
void setupBLE();

// loop()から毎回呼び出し、切断後の再広告などを処理する。
void updateBLE();

// ESP32 -> PCへのNotification送信。
void sendBLE(uint8_t* data, size_t length);
