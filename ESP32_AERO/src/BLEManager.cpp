#include "BLEManager.h"
#include "esp_log.h" // For ESP_LOGx macros
#include <LittleFS.h> // For logging commands
#include <SD.h>       // For logging commands
#include <cmath>      // For NAN

static const char* TAG = "BLEManager";

// Global variables (declared extern in BLEManager.h)
std::atomic<float> g_ble_ground_speed{NAN};
unsigned long g_lastWheelUpdateMillis = 0;

// --- BLE Callbacks Implementations ---

// Access the global manager instance from main.cpp. 
// The standard ESP32 BLE library does not allow passing context to notification callbacks,
// and lacks a getter for client callbacks, so we must reference the global instance
// to update the wheel state variables.

extern BLEManager bleManager;

// BLE Client Callbacks
class BLEManager::MyClientCallback : public BLEClientCallbacks {
public:
    BLEManager* _bleManager;
    MyClientCallback(BLEManager* bm) : _bleManager(bm) {}
    void onConnect(BLEClient* pclient) override { _bleManager->sensorClientConnected = true; }
    void onDisconnect(BLEClient* pclient) override {
        _bleManager->sensorClientConnected = false;
        g_ble_ground_speed.store(NAN);
        _bleManager->firstWheelData = true;
        ESP_LOGI(TAG, "CSC Sensor disconnected. Restarting scan.");
        BLEDevice::getScan()->start(5, false); // Restart scan on disconnect
    }
    void* getCustomData() { return _bleManager; } // Allow notifyCallback to get BLEManager instance
};

// BLE Server Characteristic Callbacks
class BLEManager::MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    SystemConfig* _config;
    MyCharacteristicCallbacks(SystemConfig* cfg) : _config(cfg) {}

    void onWrite(BLECharacteristic* pChar) override {
        uint8_t* pData = pChar->getData();
        size_t len = pChar->getLength();
        if (len > 0) {
            char cmd = (char)pData[0];
            if (cmd == 's' || cmd == 'S') {
                _config->loggingEnabled = !_config->loggingEnabled;
                ESP_LOGI(TAG, "BLE COMMAND: LOGGING %s", _config->loggingEnabled ? "ENABLED" : "DISABLED");
            } else if (cmd == 'c' || cmd == 'C') {
                // NOTE: Calling deleteLogFile() here performs SD I/O inside a BLE callback.
                // This can block the stack and cause disconnects. 
                extern bool deleteLogFile();
                if (deleteLogFile()) {
                    ESP_LOGI(TAG, "BLE COMMAND: LOG FILE DELETED");
                } else {
                    ESP_LOGE(TAG, "BLE COMMAND: ERROR DELETING LOG (Storage Busy)");
                }
            }
        }
    }
};

// BLE Advertised Device Callbacks
class BLEManager::MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
public:
    BLEManager* _bleManager;
    MyAdvertisedDeviceCallbacks(BLEManager* bm) : _bleManager(bm) {}
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(_bleManager->cscServiceUUID)) {
            BLEDevice::getScan()->stop();
            if (_bleManager->myDevice == nullptr) {
                _bleManager->myDevice = new BLEAdvertisedDevice(advertisedDevice);
            } else {
                *_bleManager->myDevice = advertisedDevice;
            }
            _bleManager->doConnect = true;
            ESP_LOGI(TAG, "Found CSC sensor: %s", advertisedDevice.toString().c_str());
        }
    }
};

// BLE Server Callbacks
class BLEManager::MyServerCallbacks : public BLEServerCallbacks {
public:
    BLEManager* _bleManager;
    MyServerCallbacks(BLEManager* bm) : _bleManager(bm) {}
    void onConnect(BLEServer *pServer) override { 
        _bleManager->deviceConnected = true; 
        _bleManager->oldDeviceConnected = true;
        ESP_LOGI(TAG, "BLE Client connected."); 
    };
    void onDisconnect(BLEServer *pServer) override {
        _bleManager->deviceConnected = false;
        _bleManager->oldDeviceConnected = false;
        ESP_LOGI(TAG, "BLE Client disconnected. Restarting advertising.");
        _bleManager->pServer->getAdvertising()->start();
        _bleManager->lastBleNotifyTime = 0; // Ensure fresh data on reconnect
    }
};

// CSC Client Notify Callback
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length < 1) return;
    uint8_t flags = pData[0];
    int offset = 1;
    
    if (flags & 0x01) { // Wheel Revolution Data Present
        if (length < 7) return;
        uint32_t wheelRevs = pData[offset] | (pData[offset+1] << 8) | (pData[offset+2] << 16) | (pData[offset+3] << 24);
        offset += 4;
        uint16_t wheelTime = pData[offset] | (pData[offset+1] << 8);
        
        if (!bleManager.firstWheelData) {
            uint32_t revDelta = wheelRevs - bleManager.lastWheelRevs;
            uint16_t timeDelta = wheelTime - bleManager.lastWheelTime; // uint16 handles rollover automatically
            
            if (timeDelta > 0 && revDelta > 0) {
                g_ble_ground_speed.store(((float)revDelta * WHEEL_CIRCUMFERENCE) / ((float)timeDelta / 1024.0f));
                g_lastWheelUpdateMillis = millis(); // Update global timestamp
            }
        }
        bleManager.lastWheelRevs = wheelRevs;
        bleManager.lastWheelTime = wheelTime;
        bleManager.firstWheelData = false;
    }
}

// --- BLEManager Class Implementations ---

BLEManager::BLEManager() {}

void BLEManager::begin(SystemConfig* config) {
    sysConfig = config;
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(this));

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | 
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks(sysConfig));

    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    
    // Optimized for Garmin/iOS: Primary packet contains Flags and Service UUID.
    // 128-bit UUID (16 bytes) + Flags (3 bytes) fits well in the 31-byte limit.
    BLEAdvertisementData oAdvertisementData;
    oAdvertisementData.setFlags(0x06);
    oAdvertisementData.setCompleteServices(BLEUUID(SERVICE_UUID));
    pAdvertising->setAdvertisementData(oAdvertisementData);

    pAdvertising->setAppearance(0x0480);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    pAdvertising->setMaxPreferred(0x12); // Corrected from setMinPreferred
    pAdvertising->start();

    // Start CSC scanning after a 5-second delay. This ensures the radio is 
    // available for the Garmin's initial connection and GATT discovery phase.
    xTaskCreate([](void* obj){ vTaskDelay(pdMS_TO_TICKS(5000)); ((BLEManager*)obj)->startScanningForCSC(); vTaskDelete(NULL); }, "CSCStart", 2048, this, 1, NULL);
}

void BLEManager::startScanningForCSC() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(this));
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false); // Scan for 5 seconds, then stop
}

void BLEManager::handleClientConnection() {
    if (doConnect == true) {
        if (myDevice == nullptr) {
            ESP_LOGE(TAG, "Attempted to connect to null device.");
            doConnect = false;
            return;
        }

        static BLEClient* pClient = nullptr;
        if (pClient == nullptr) {
            pClient = BLEDevice::createClient();
            pClient->setClientCallbacks(new MyClientCallback(this));
        }

        if (!pClient->isConnected()) {
            if (!pClient->connect(myDevice)) {
                ESP_LOGE(TAG, "Failed to connect to CSC sensor.");
                startScanningForCSC(); // Restart scan if connection fails
                doConnect = false;
                return;
            }
        }

        // Negotiate MTU for Garmin compatibility (23 bytes MTU = 20 bytes payload)
        pClient->setMTU(23);
        
        // Give the BLE stack and the sensor a moment to stabilize before discovery
        delay(200); 

        // Explicitly trigger discovery to fill the GATT table
        ESP_LOGI(TAG, "Discovering services...");
        pClient->getServices(); 

        BLERemoteService* pRemoteService = pClient->getService(cscServiceUUID);
        if (pRemoteService == nullptr) {
            ESP_LOGE(TAG, "Failed to find CSC service.");
            pClient->disconnect();
            startScanningForCSC();
            doConnect = false;
            return;
        }
        
        pRemoteCharacteristic = pRemoteService->getCharacteristic(cscCharUUID);
        if (pRemoteCharacteristic == nullptr) {
            ESP_LOGE(TAG, "Failed to find CSC characteristic.");
            pClient->disconnect();
            startScanningForCSC();
            doConnect = false;
            return;
        }
        
        if (pRemoteCharacteristic->canNotify()) {
            pRemoteCharacteristic->registerForNotify(notifyCallback);
            sensorClientConnected = true;
            ESP_LOGI(TAG, "Connected to CSC sensor and subscribed to notifications.");
        } else {
            ESP_LOGW(TAG, "CSC characteristic does not support notifications.");
            pClient->disconnect();
            startScanningForCSC();
        }
        doConnect = false;
    }
}

void BLEManager::notifyCharacteristic(const char* data, size_t len) {
    // Only attempt to notify if a device is connected AND the client has enabled notifications
    if (deviceConnected && pCharacteristic != nullptr) {
        BLEDescriptor* pDesc = pCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));

        // --- Debug check: Ensure no newline characters are sent over BLE ---
        #ifdef DEBUG_BLE_NOTIFICATIONS
        for (size_t i = 0; i < len; ++i) {
            if (data[i] == '\n' || data[i] == '\r') {
                ESP_LOGE(TAG, "CRITICAL ERROR: Newline character detected in BLE notification data at index %d! Data: '%.*s'", i, len, data);
                // Consider adding an assert here if this condition should never be met in production
                break; 
            }
        }
        #endif

        // Check if CCCD (0x2902) exists and notifications (bit 0) are enabled
        if (pDesc != nullptr && pDesc->getLength() > 0 && (pDesc->getValue()[0] & 0x01)) {
            // The client has a 20-byte MTU limit. We send the data in raw 20-byte chunks.
            // We do NOT add newlines here, as that causes the client to parse partial messages.
            const size_t MAX_MTU = 20;
            for (size_t i = 0; i < len; i += MAX_MTU) {
                size_t chunkLen = (len - i > MAX_MTU) ? MAX_MTU : (len - i);
                
                pCharacteristic->setValue((uint8_t*)&data[i], chunkLen);
                pCharacteristic->notify();
                
                // Delay to prevent saturating the BLE stack buffers and allow client processing
                delay(50);
            }
        }
    }
}