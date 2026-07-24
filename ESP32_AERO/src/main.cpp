/**
 * @file main.cpp
 * @brief Main application for ESP32_AERO project.
 * Orchestrates sensor data acquisition, BLE communication, and logging.
 */

#include <LittleFS.h> // For internal flash logging
#include <SPI.h>      // For SD card SPI communication
#include <SD.h>       // For SD card filesystem
#include <cmath>      // For std::isnan
#include <cstring>

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
#ifdef M5STACK_CAPSULE
  // Capsule v1.1 must assert HOLD immediately after wake to remain powered.
  pinMode(CAPSULE_POWER_HOLD_PIN, OUTPUT);
  digitalWrite(CAPSULE_POWER_HOLD_PIN, HIGH);
#endif

  // 1. Move Serial initialization to the ABSOLUTE top.
  Serial.begin(115200);
  
  // S3 Native USB stabilization delays
  delay(3000); 
  while(!Serial && millis() < 5000); 
  delay(500);

  Serial.println("\n[SYS] Booting ESP32-S3...");
  listPartitions();

  // 2. CRITICAL FIX: Instantiate FreeRTOS Queue BEFORE sensors start up.
  // This completely eliminates null-pointer exceptions if any sensor queries it on initialization.
  sensorQueue = xQueueCreate(100, sizeof(QueueData));
  if (sensorQueue == NULL) {
    Serial.println("[ERROR] Failed to create sensorQueue; restarting...");
    delay(1000);
    ESP.restart();
    return;
  }

  // 3. Initialize Sensors
  sensorManager.begin(&sysConfig);

#ifdef M5STACK_CAPSULE
  // Initialize the Capsule BM8563 RTC on the same internal bus as the BMI270.
  bool rtcReady = M5.Rtc.begin(&M5.In_I2C, m5::board_t::board_M5Capsule);
  m5::rtc_datetime_t rtcNow;
  bool rtcValid = rtcReady && M5.Rtc.getDateTime(&rtcNow) &&
                  !M5.Rtc.getVoltLow() && rtcNow.date.year >= 2024;
  if (rtcReady && !rtcValid) {
    // Seed an invalid/low-voltage RTC from local firmware compile time.
    static const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char monthText[4] = {};
    int day = 1, year = 2024, hour = 0, minute = 0, second = 0;
    sscanf(__DATE__, "%3s %d %d", monthText, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);
    const char* monthPos = strstr(months, monthText);
    const int month = monthPos ? static_cast<int>((monthPos - months) / 3) + 1 : 1;
    m5::rtc_date_t date(year, month, day, -1);
    m5::rtc_time_t time(hour, minute, second);
    M5.Rtc.setDateTime(&date, &time);
    rtcValid = M5.Rtc.getDateTime(&rtcNow);
    Serial.println("[RTC] Invalid clock seeded from firmware compile time.");
  }
  sysConfig.rtcAvailable.store(rtcValid);
  Serial.printf("[RTC] BM8563: %s\n", rtcValid ? "ready" : "unavailable; using uptime");
#endif

  // 4. Initialize SD Card (SPI Peripheral Setup)
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sysConfig.sdAvailable = SD.begin(SD_CS_PIN);

  // 5. Initialize LittleFS for fallback flash logging
  // Never auto-format on mount failure: preserve recoverable logs.
  sysConfig.littleFsAvailable = LittleFS.begin(false, "/littlefs", 10, "spiffs");

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
    const BaseType_t taskResult = xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        8192,               // Ample stack headroom to prevent RTC runtime resetting
        &sensorManager,     // Context pointer passed safely
        1,                  // Scheduled Priority
        &sensorTaskHandle,
        0                   // Affined specifically to Core 0
    );
    if (taskResult == pdPASS) {
      Serial.println("[SYS] Multi-Core Sensor Task started successfully on Core 0.");
    } else {
      sensorTaskHandle = NULL;
      Serial.println("[WARN] Sensor task creation failed; using loop polling fallback.");
    }
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
  serviceUtilities();

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
    char fullStr[256];
    serializeTelemetry(
        fullStr, sizeof(fullStr), 
        avgData.airspeed, avgData.air_rho, avgData.temp, avgData.pressure,
        avgData.altitude, alt_change, avgData.ground_speed,
        avgData.climb_rate, avgData.forward_accel,
        avgData.accel_x, avgData.accel_y, avgData.accel_z,
        sensorManager.isAirspeedSensorAvailable(),
        sensorManager.isAHTAvailable() || sensorManager.isBMPAvailable() ||
            sensorManager.isBMP280Available(),
        sensorManager.isBMPAvailable() || sensorManager.isBMP280Available(),
        bleManager.isSensorClientConnected(),
        sensorManager.isIMUAvailable()
    );

    // Commit only complete telemetry records.
    if (fullStr[0] != '\0') processLoggingAndDashboard(now, fullStr);

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
