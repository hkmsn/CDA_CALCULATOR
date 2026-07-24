#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h> // For I2C communication
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <atomic> // For std::atomic
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "KalmanFilter.h"
#include "SystemConfig.h"
#ifdef M5STACK_CAPSULE
#include <M5Unified.h>
#endif

// Queue for sending sensor data between cores
struct QueueData {
    float temp;
    float pressure; // in Pa
    float airspeed; // in m/s
    float altitude; // in meters
    float air_rho;
    float ground_speed;
    float climb_rate; // fused vertical speed in m/s
    float forward_accel; // gravity-compensated acceleration toward USB in m/s^2
    float accel_x; // Capsule BMI270 acceleration in g
    float accel_y;
    float accel_z;
};

extern QueueHandle_t sensorQueue;
extern TaskHandle_t sensorTaskHandle;
extern std::atomic<float> g_ble_ground_speed; // From BLEManager
extern std::atomic<uint32_t> g_sensorQueueDrops;
extern std::atomic<uint32_t> g_i2cRecoveries;
extern std::atomic<uint32_t> g_imuRecoveries;

class SensorManager {
public:
    SensorManager();

    // Grant the background task access to private persistent state and reading helpers
    friend void sensorTask(void *pvParameters);

    void begin(SystemConfig* config = nullptr);
    void calibrateSensors();
    void recoverI2CBus();
    
    void getAirspeedData(float measurement, float &smoothed_as);

    // Helper functions
    float calculateAltitude(float currentPressure, float referencePressure);
    float calculateAirDensity(float tempC, float pressurePa = SEA_LEVEL_PRESSURE_PA);
    // Fuses vertical acceleration with the internally selected BMP390/BMP280
    // altitude. If neither exists, it continues acceleration-only (with drift).
    float estimateClimbRate(float verticalAccelerationMps2,
                            unsigned long timestampMs);
    void resetClimbRateEstimator();
    bool updateSingleCore(QueueData &outData);

    // Telemetry processing (Averaging over publish intervals)
    void recordSample(const QueueData &data);
    void getAveragedTelemetry(QueueData &outData, float &altChange);
    
    // Sensor reading helpers used by the FreeRTOS task
    bool _readBmpData();
    bool _readAirspeedData();
    bool _readAhtData();
    bool _readImuData();

    // Status flags
    bool isAirspeedSensorAvailable() const { return asAvailable; }
    bool isBMPAvailable() const    { return bmpAvailable; }
    bool isBMP280Available() const { return bmp280Available; }
    bool isAHTAvailable() const    { return ahtAvailable; }
    bool isIMUAvailable() const    { return imuAvailable; }

    // Public access to Kalman filter for potential external adjustments or state retrieval
    float getSeaLevelPressure() const { return seaLevelPressure; }
    KalmanFilter1D& getAirspeedKalmanFilter() { return as_filter; }

    // Sensor instances
    Adafruit_BMP3XX bmp;
    Adafruit_BMP280 bmp280;
    Adafruit_AHTX0 aht;

private:
    bool _i2cInitialized;              // Tracks if Wire.begin() has been called
    void _detectAndConfigureSensors(); // Declaration for the helper function
    SystemConfig* sysConfig = nullptr;
    std::atomic<bool> asAvailable{false};
    std::atomic<bool> bmpAvailable{false}; // BMP390
    std::atomic<bool> bmp280Available{false};
    std::atomic<bool> ahtAvailable{false};
    std::atomic<bool> imuAvailable{false};

    float as_offset_pa     = 0.0f;     // Zero-pressure offset calibration
    float seaLevelPressure = SEA_LEVEL_PRESSURE_PA;
    uint8_t bmpI2CAddr     = 0x77;

    KalmanFilter1D as_filter;
    KalmanFilter1D imu_x_filter;
    KalmanFilter1D imu_y_filter;
    KalmanFilter1D imu_z_filter;
    bool _imuFiltersInitialized = false;

    // Private sensor reading helpers
    bool _readRawAirspeedSensor(float &raw_airspeed_diff_press_pa, float &internal_sensor_temp_c, float ambient_pressure_pa); // No default argument here

    // Interval accumulators and counters
    float _p_acc = 0, _t_acc = 0, _alt_acc = 0, _as_acc = 0, _rho_acc = 0, _gs_acc = 0;
    float _climb_rate_acc = 0, _forward_accel_acc = 0, _imu_x_acc = 0, _imu_y_acc = 0, _imu_z_acc = 0;
    int   _p_n = 0,   _t_n = 0,   _alt_n = 0,   _as_n = 0,   _rho_n = 0,   _gs_n = 0;
    int   _climb_rate_n = 0, _forward_accel_n = 0, _imu_n = 0;
    float _alt_diff_acc = 0.0f;
    float _last_recorded_alt = NAN;

    // Persistent filter and last-read state
    float _ema_alt  = NAN;
    float _t_last   = NAN;
    float _p_last   = NAN;
    float _alt_last = NAN;
    float _as_last  = NAN;
    float _rho_last = NAN;
    float _gs_last  = NAN;
    float _imu_x_last = NAN;
    float _imu_y_last = NAN;
    float _imu_z_last = NAN;
    float _forward_accel_mps2 = NAN;
    float _gravity_x_g = 0.0f;
    float _gravity_y_g = 0.0f;
    float _gravity_z_g = 1.0f;
    unsigned long _imu_attitude_last_ms = 0;
    bool _gravityInitialized = false;

    // Vertical-speed estimator state. The Capsule USB-facing direction is -Y,
    // but verticalAccelerationMps2 must already be rotated into the earth frame.
    float _climb_rate_mps = 0.0f;
    float _climb_baro_rate_mps = 0.0f;
    float _climb_last_alt_m = NAN;
    unsigned long _climb_last_ms = 0;
    unsigned long _climb_last_baro_ms = 0;

    const float _alt_alpha = 0.2f;

    SemaphoreHandle_t _i2cMutex = nullptr;
};

void sensorTask(void *pvParameters); // FreeRTOS task function

#endif // SENSOR_MANAGER_H
