/**
 * @file main.cpp
 * @brief Main application for ESP32_AERO project.
 * Orchestrates sensor data acquisition, BLE communication, and logging.
 */

#include <LittleFS.h> // For internal flash logging
#include <SPI.h>      // For SD card SPI communication
#include <SD.h>       // For SD card filesystem
#include <cmath>      // For std::isnan
#include "esp_log.h"

// For multi-core support
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Custom Modules
#include "BLEManager.h"
#include "SensorManager.h"
#include "Utilities.h"
#include "SystemConfig.h"

// --- Global Variables ---
SystemConfig sysConfig;

QueueHandle_t sensorQueue     = NULL;
TaskHandle_t sensorTaskHandle = NULL;

// --- Module Instances ---
SensorManager sensorManager;
BLEManager    bleManager;

// Use the task defined inside SensorManager.cpp
extern void sensorTask(void *pvParameters); 

// Prototype for the reduced BLE telemetry format (Temperature removed)
extern void serializeBLETelemetry(char* outBuf, size_t sz, float as, float rho, float alt, float alt_ch, bool as_ok, bool bmp_ok);
extern void printDashboardHeader();

/**
 * @brief Standard Arduino setup function.
 */
void setup() {
  // 1. Move Serial initialization to the ABSOLUTE top.
  Serial.begin(115200);
  
  // S3 Native USB stabilization delays
  delay(3000); 
  while(!Serial && millis() < 5000); 
  delay(500);

  esp_log_level_set("BLEScan", ESP_LOG_ERROR);

  Serial.println("\n[SYS] Booting ESP32-S3...");
  listPartitions();

  // 2. CRITICAL FIX: Instantiate FreeRTOS Queue BEFORE sensors start up.
  // This completely eliminates null-pointer exceptions if any sensor queries it on initialization.
  sensorQueue = xQueueCreate(100, sizeof(QueueData));
  if (sensorQueue == NULL) {
    Serial.println("[ERROR] Failed to create sensorQueue! Freezing...");
    while(1);
  }

  // 3. Initialize Sensors
  sensorManager.begin(&sysConfig);

  // 4. Initialize SD Card (SPI Peripheral Setup)
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sysConfig.sdAvailable = SD.begin(SD_CS_PIN);

  // 5. Initialize LittleFS for fallback flash logging
  sysConfig.littleFsAvailable = LittleFS.begin(true, "/littlefs", 10, "spiffs");

  // 6. Initialize remaining infrastructure modules
  initUtilities(&sysConfig);
  bleManager.begin(&sysConfig);

  // File system checks
  writeLogHeader();
  printMenu();
  delay(500); 

  if (sysConfig.serialOutputEnabled) {
    printDashboardHeader();
  }

  // 7. Spawn the background data aquisition engine on Core 0
  if (ESP.getChipCores() > 1) {
    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        8192,               // Ample stack headroom to prevent RTC runtime resetting
        &sensorManager,     // Context pointer passed safely
        1,                  // Scheduled Priority
        &sensorTaskHandle,
        0                   // Affined specifically to Core 0
    );
    Serial.println("[SYS] Multi-Core Sensor Task started successfully on Core 0.");
  } else {
    Serial.println("[SYS] Single core environment detected. Falling back to poll injection.");
  }
}

/**
 * @brief Standard Arduino loop function (Runs continuously on Core 1).
 */
void loop() {
  // Process incoming UI commands via serial monitor interface
  if (Serial.available()) {
    handleSerialCommand(Serial.read());
  }

  unsigned long now = millis();
  bleManager.handleClientConnection();

  // --- Telemetry Sinking State Machine ---
  if (sensorTaskHandle != NULL && sensorQueue != NULL) {
    QueueData data;
    // Safely drain all accumulated elements out of the transactional FIFO buffer
    while (xQueueReceive(sensorQueue, &data, 0) == pdTRUE) {
      sensorManager.recordSample(data);
    }
  } else {
    // SINGLE-CORE FALLBACK MODE
    // Throttle data execution updates match the expected telemetry interval rate.
    static unsigned long lastSingleCorePoll = 0;
    if (now - lastSingleCorePoll >= 20) { // Poll matching 50Hz frequency constraints
        lastSingleCorePoll = now;
        QueueData localData;
        if (sensorManager.updateSingleCore(localData)) {
            sensorManager.recordSample(localData);
        }
    }
  }
  
  // --- Periodic Aggregation and Broadcasting Engine ---
  if (now - bleManager.lastBleNotifyTime >= BLE_PUBLISH_INTERVAL) {
    bleManager.lastBleNotifyTime = now; 

    float alt_change = 0.0f;
    QueueData avgData;
    
    // Process math averages from the accumulated records collected over the 1s window
    sensorManager.getAveragedTelemetry(avgData, alt_change);

    // Build the dynamic primary output buffer string
    char fullStr[128];
    serializeTelemetry(
        fullStr, sizeof(fullStr), 
        avgData.airspeed, avgData.air_rho, avgData.temp, avgData.pressure, avgData.altitude, alt_change, avgData.ground_speed,
        sensorManager.isAirspeedSensorAvailable(),
        sensorManager.isBMPAvailable() || sensorManager.isBMP280Available(),
        bleManager.isSensorClientConnected()
    );

    // Commit telemetry string changes permanently to storage (SD/Flash)
    processLoggingAndDashboard(now, fullStr);

    // Handle Active Bluetooth Client Notifications
    if (bleManager.isDeviceConnected()) {
      char bleStr[128];
      serializeBLETelemetry(
          bleStr, sizeof(bleStr), 
          avgData.airspeed, avgData.air_rho, avgData.altitude, alt_change,
          sensorManager.isAirspeedSensorAvailable(),
          sensorManager.isBMPAvailable() || sensorManager.isBMP280Available()
      );
      bleManager.notifyCharacteristic(bleStr, strlen(bleStr));
    }
  }

  // Essential yield to satisfy the FreeRTOS Scheduler and kick the System Watchdogs (IDLE Task breathing room)
  vTaskDelay(pdMS_TO_TICKS(1));
}
