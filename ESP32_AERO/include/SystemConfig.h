#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <atomic>

// I2C Pins
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 15

// M5Stack Capsule internal BMI270 bus (separate from external sensors).
#define CAPSULE_IMU_SDA_PIN 8
#define CAPSULE_IMU_SCL_PIN 10
#define CAPSULE_POWER_HOLD_PIN 46

// SD Card SPI pins. Capsule v1.1 uses its onboard microSD wiring; generic
// ESP32-S3 builds retain the external-card defaults and may override these
// definitions with build flags when using different hardware.
#ifdef M5STACK_CAPSULE
#define SD_CS_PIN   11
#define SD_MOSI_PIN 12
#define SD_MISO_PIN 39
#define SD_SCK_PIN  14
#else
#ifndef SD_CS_PIN
#define SD_CS_PIN   4
#endif
#ifndef SD_MOSI_PIN
#define SD_MOSI_PIN 5
#endif
#ifndef SD_MISO_PIN
#define SD_MISO_PIN 6
#endif
#ifndef SD_SCK_PIN
#define SD_SCK_PIN  7
#endif
#endif

// MS4525DO Airspeed Sensor Definitions
#define MS4525DO_ADDR   0x28     // Default I2C address
#define RHO_AIR_DENSITY 1.184f // kg/m^3 at 25c 1.225f // kg/m^3, standard air density at sea level
#define I2C_MAX_ERRORS  5

// Global reference
constexpr float         SEA_LEVEL_PRESSURE_PA  = 101325.0f;
constexpr unsigned long BLE_PUBLISH_INTERVAL   = 250; // 4HZ   // Publish to BLE every 1000ms 1 sec
constexpr float         ALT_CHANGE_DEADZONE    = 0.01f;  // Ignore altitude changes less than 1cm in BLE_PUBLISH_INTERVAL 250ms
constexpr unsigned long ESP32_LOGGING_INTERVAL = 2000;    // Log and update dashboard every 2 seconds
constexpr unsigned long BMP_READ_INTERVAL      = 75;      // Max sensitivity: fits 32x pressure oversampling (68ms)
constexpr unsigned long AIRSPEED_READ_INTERVAL = 20;      // Read airspeed every 20 ms (50 Hz)
constexpr unsigned long AHT_READ_INTERVAL      = 500;     // Polling rate for AHT20 temperature/humidity
constexpr unsigned long IMU_READ_INTERVAL      = 20;      // Built-in BMI270 sample rate (50 Hz)

struct SystemConfig {
    std::atomic<bool> littleFsAvailable{false};
    std::atomic<bool> sdAvailable{false};
    std::atomic<bool> loggingEnabled{true};
    std::atomic<bool> rtcAvailable{false};
    bool debug                   = true;
    std::atomic<bool> serialOutputEnabled{true};
};

#endif // SYSTEM_CONFIG_H
