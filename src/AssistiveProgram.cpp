#include "AssistiveProgram.h"

int map_controller(int in, int ineRange, int inMin, int inMax, int outMin, int outMax) {
    int in_abs = abs(in);
    int in_pn = (in_abs != 0) ? (in / in_abs) : 0;

    int in_cut;
    if (in_abs < ineRange) {
        in_cut = 0;
    } else {
        in_cut = in - ineRange * in_pn;
    }

    return map(in_cut, inMin + ineRange, inMax - ineRange, outMin, outMax);
}

void bondReset() {
    int dev_num = esp_bt_gap_get_bond_device_num();

    if (dev_num > 0) {
        esp_bd_addr_t dev_list[dev_num];
        esp_bt_gap_get_bond_device_list(&dev_num, dev_list);
        
        for (int i = 0; i < dev_num; i++) {
            esp_bt_gap_remove_bond_device(dev_list[i]);
        }
    }
}

void FrameLimiter::setInterval(unsigned long interval) {
    interval_ms = interval;
    next_target_ms = millis() + interval_ms;
}

void FrameLimiter::wait() {
    unsigned long now = millis();
    if ((long)(next_target_ms - now) > 0) {
        delay(next_target_ms - now);
    }
}

void FrameLimiter::update() {
    next_target_ms += interval_ms;
    if ((long)(millis() - next_target_ms) > 0) {
        next_target_ms = millis() + interval_ms;
    }
}

long FrameLimiter::sync() {
    unsigned long now = millis();
    long overrun = (long)(now - next_target_ms);

    wait();
    update();

    return (overrun > 0) ? overrun : 0;
}

void LittleFS_ini() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }
    Serial.println("LittleFS Mounted Successfully");
}

void listFiles() {
    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) {
        Serial.println("Failed to open root directory");
        return;
    }

    Serial.println("--- File List ---");
    File file = root.openNextFile();
    bool found = false;
    
    while (file) {
        found = true;
        Serial.printf("Name: %-15s | Size: %u bytes\n", file.name(), file.size());
        file = root.openNextFile();
    }
    
    if (!found) {
        Serial.println("No files found.");
    }
    Serial.println("-----------------");
}