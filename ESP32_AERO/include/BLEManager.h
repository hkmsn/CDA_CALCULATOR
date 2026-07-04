#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include "sdkconfig.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h" // Must be included before other BLE headers to define GAP types
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include <atomic>
#include "SystemConfig.h"

// Ground Speed (Bicycle) BLE Sensor Definitions
#define WHEEL_CIRCUMFERENCE 2.095f // Standard 700x23c

// BLE Service and Characteristic UUIDs for ESP32_Aero
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME "ESP32_Aero"

extern unsigned long g_lastWheelUpdateMillis; // Global timestamp for last wheel update
extern std::atomic<float> g_ble_ground_speed; // Global atomic for ground speed

class BLEManager {
public:
    BLEManager();
    void begin(SystemConfig* config);
    void handleClientConnection();
    void startScanningForCSC();
    void notifyCharacteristic(const char* data, size_t len);

    bool isDeviceConnected() const { return deviceConnected; }
    bool isSensorClientConnected() const { return sensorClientConnected; }

    // Public for main.cpp to access for logging/display
    BLEServer *pServer = NULL; 
    unsigned long lastBleNotifyTime = 0;
    bool oldDeviceConnected = false;

    // CSC Wheel State
    uint32_t lastWheelRevs = 0;
    uint16_t lastWheelTime = 0;
    bool firstWheelData = true;

    // Move callback classes to public so the static notifyCallback can access the types
    class MyServerCallbacks;
    class MyCharacteristicCallbacks;
    class MyAdvertisedDeviceCallbacks;
    class MyClientCallback;

private:
    SystemConfig* sysConfig = nullptr;
    BLECharacteristic *pCharacteristic = NULL;
    volatile bool deviceConnected = false;

    // CSC Client specific
    BLEUUID cscServiceUUID = BLEUUID((uint16_t)0x1816);
    BLEUUID cscCharUUID    = BLEUUID((uint16_t)0x2A5B);
    bool doConnect = false;
    bool sensorClientConnected = false;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
    BLEAdvertisedDevice* myDevice = nullptr;
};

#endif // BLE_MANAGER_H