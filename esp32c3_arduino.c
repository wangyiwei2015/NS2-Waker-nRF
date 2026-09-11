#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <esp_mac.h>
#include <esp_sleep.h>

#define DEVICE_NAME "NS2_Waker"
// 目标JC蓝牙MAC地址 (Base MAC is BT MAC 的末位 - 2)
const uint8_t baseMac[6] = {0x78, 0x81, 0x8c, 0xd6, 0xb7, 0x38-2};
const uint8_t mfgData[] = {
    0x53, 0x05,
    0x01, 0x00, 0x03, 0x7E,
    0x05, 0x66, 0x20, 0x00,
    0x01, 0x81, 0x4F, 0xF4,
    0xE0, 0x8C, 0x81, 0x78,
    0x0F, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};
BLEAdvertising *pAdvertising;

void loop() {} // No loops~

void setup() {
    esp_base_mac_addr_set(baseMac);
    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    pAdvertising = BLEDevice::getAdvertising();
    BLEAdvertisementData advData;
    advData.setFlags(0x06);
    advData.setManufacturerData(String((const char*)mfgData, sizeof(mfgData)));
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setMinInterval(0x0020);
    pAdvertising->setMaxInterval(0x0040);
    pAdvertising->start();
    delay(1500);
    pAdvertising->stop();
    delay(500);
    esp_deep_sleep_start(); // 深度休眠超低功耗
}
