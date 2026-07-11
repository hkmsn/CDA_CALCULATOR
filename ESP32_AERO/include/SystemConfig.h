#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// I2C Pins
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 15

// SD Card SPI Pins (Safe for S3: Avoid 10-14)
#define SD_CS_PIN   4
#define SD_MOSI_PIN 5
#define SD_MISO_PIN 6
#define SD_SCK_PIN  7

// MS4525DO Airspeed Sensor Definitions
#define MS4525DO_ADDR   0x28     // Default I2C address
#define RHO_AIR_DENSITY 1.184f // kg/m^3 at 25c 1.225f // kg/m^3, standard air density at sea level
#define I2C_MAX_ERRORS  5

// MS4525DO transfer function: 10% to 90% counts over the configured pressure span.
// 4525DAI 001D is a +/-1 psi differential pressure sensor.
constexpr float MS4525DO_FULL_SCALE_PRESSURE_PSI = 1.0f;
constexpr float MS4525DO_PRESSURE_MIN_PSI = -MS4525DO_FULL_SCALE_PRESSURE_PSI;
constexpr float MS4525DO_PRESSURE_MAX_PSI =  MS4525DO_FULL_SCALE_PRESSURE_PSI;
constexpr float MS4525DO_COUNTS_MIN       = 1638.4f;
constexpr float MS4525DO_COUNTS_MAX       = 14745.6f;
constexpr float PSI_TO_PA                 = 6894.76f;
constexpr float AIRSPEED_PRESSURE_DEADBAND_PA = 1.0f;
constexpr int   AIRSPEED_ZERO_CONFIRM_SAMPLES = 8;

// Global reference
constexpr float         SEA_LEVEL_PRESSURE_PA  = 101325.0f;
constexpr unsigned long BLE_PUBLISH_INTERVAL   = 250; // 4HZ   // Publish to BLE every 1000ms 1 sec
constexpr float         ALT_CHANGE_DEADZONE    = 0.10f;  // Ignore road/barometer noise under 10cm per BLE publish interval
constexpr unsigned long ESP32_LOGGING_INTERVAL = 2000;    // Log and update dashboard every 2 seconds
constexpr unsigned long BMP_READ_INTERVAL      = 75;      // Max sensitivity: fits 32x pressure oversampling (68ms)
constexpr unsigned long AIRSPEED_READ_INTERVAL = 50;      // Increased frequency: Read Airspeed every 20ms (50Hz)
constexpr unsigned long AHT_READ_INTERVAL      = 500;     // Polling rate for AHT20 temperature/humidity

struct SystemConfig {
    bool littleFsAvailable       = false;
    bool sdAvailable             = false;
    volatile bool loggingEnabled = true;
    bool debug                   = true;
    bool serialOutputEnabled     = true;
};

#endif // SYSTEM_CONFIG_H
