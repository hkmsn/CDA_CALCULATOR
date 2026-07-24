/**
 * @file SensorManager.cpp
 * @brief Manages connected sensors with optimized I2C bus sequencing
 */

#include "SensorManager.h"
#include "esp_log.h" // For ESP_LOGx macros
#include <cmath> // For std::isnan
#include <cstring> // For memset

static const char* TAG = "SensorManager";

// Global atomic for ground speed (declared extern in BLEManager.h)
extern std::atomic<float> g_ble_ground_speed;
// Global timestamp for last wheel update (declared extern in BLEManager.h)
extern std::atomic<unsigned long> g_lastWheelUpdateMillis;
std::atomic<uint32_t> g_sensorQueueDrops{0};
std::atomic<uint32_t> g_i2cRecoveries{0};
std::atomic<uint32_t> g_imuRecoveries{0};
 
SensorManager::SensorManager() 
    : as_filter(0.25f, 0.5f, 0.0f),
      imu_x_filter(0.01f, 0.1f, 0.0f),
      imu_y_filter(0.01f, 0.1f, 0.0f),
      imu_z_filter(0.01f, 0.1f, 0.0f),
      _t_last(NAN), _p_last(NAN), _alt_last(NAN), _as_last(NAN), _rho_last(NAN), _gs_last(NAN),
      _ema_alt(NAN), _alt_alpha(0.15f),
      _alt_diff_acc(0.0f), _last_recorded_alt(NAN),
      _p_acc(0.0f), _t_acc(0.0f), _alt_acc(0.0f), _as_acc(0.0f), _rho_acc(0.0f), _gs_acc(0.0f),
      _p_n(0), _t_n(0), _alt_n(0), _as_n(0), _rho_n(0), _gs_n(0),
      _i2cInitialized(false), seaLevelPressure(101325.0f), as_offset_pa(0.0f), _i2cMutex(nullptr) {}

void SensorManager::begin(SystemConfig* config) {
    if (config != nullptr) {
        sysConfig = config;
    }
    
    if (_i2cMutex == nullptr) {
        _i2cMutex = xSemaphoreCreateRecursiveMutex();
        if (_i2cMutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create I2C mutex; external sensors disabled");
            return;
        }
    }

#ifdef M5STACK_CAPSULE
    const bool imuBusReady = M5.In_I2C.begin(
        I2C_NUM_1, CAPSULE_IMU_SDA_PIN, CAPSULE_IMU_SCL_PIN);
    imuAvailable = imuBusReady &&
        M5.Imu.begin(&M5.In_I2C, m5::board_t::board_M5Capsule);
    ESP_LOGI(TAG, "Capsule BMI270 IMU: %s", imuAvailable ? "ready" : "not found");
#else
    imuAvailable = false;
    ESP_LOGI(TAG, "Built-in IMU disabled for non-Capsule build");
#endif

    ESP_LOGI(TAG, "Initializing I2C Bus on SDA:%d, SCL:%d", I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(200); // Prevent hanging if bus is physically stuck, increased for robustness
    _i2cInitialized = true;

    // Mutex is deliberately omitted during discovery to prevent deadlocks 
    // with asynchronous underlying third-party library constructors.
    _detectAndConfigureSensors();
    
    xSemaphoreTakeRecursive(_i2cMutex, portMAX_DELAY);
    calibrateSensors();
    xSemaphoreGiveRecursive(_i2cMutex);
}

// Helper function to detect and configure sensors after I2C bus is initialized
void SensorManager::_detectAndConfigureSensors() {
    ESP_LOGI(TAG, "Starting I2C sensor discovery...");
    delay(200); // Increased stabilization delay for sensor IC power-on

    // Helper lambda to check for device presence without full driver initialization
    auto pingDevice = [](uint8_t addr) {
        Wire.beginTransmission(addr);
        return Wire.endTransmission() == 0;
    };

    // 1. MS4525DO Airspeed sensor check
    if (pingDevice(MS4525DO_ADDR)) {
        asAvailable = true;
        ESP_LOGI(TAG, "MS4525DO found at 0x%02X", MS4525DO_ADDR);
    } else {
        asAvailable = false;
        ESP_LOGW(TAG, "MS4525DO NOT found at 0x%02X", MS4525DO_ADDR);
    }

    // Force explicitly clean starting defaults
    bmpAvailable = false;
    bmp280Available = false;

    // 2. Try identifying BMP390 Barometer via light-weight ping
    bool bmp_physically_present = false;
    if (pingDevice(0x77)) {
        bmpI2CAddr = 0x77;
        bmp_physically_present = true;
    } else if (pingDevice(0x76)) {
        bmpI2CAddr = 0x76;
        bmp_physically_present = true;
    }

    bool bmp_found = false;
    if (bmp_physically_present) {
        bmp_found = bmp.begin_I2C(bmpI2CAddr, &Wire);
    }

    if (bmp_found) {
        bmpAvailable = true;
        ESP_LOGI(TAG, "BMP390 found at 0x%02X", bmpI2CAddr);
    } else {
        // Fallback to BMP280 detection if BMP390 isn't available
        ESP_LOGI(TAG, "BMP390 not found, checking for BMP280...");
        if (bmp280.begin(0x76) || bmp280.begin(0x77)) {
            bmp280Available = true;
            ESP_LOGI(TAG, "BMP280 found.");
        } else {
            ESP_LOGW(TAG, "BMP280 NOT found.");
        }
    }
    
    // 3. Initialize AHT20 Ambient sensor
    ahtAvailable = aht.begin(&Wire);
    if (!ahtAvailable) {
        ESP_LOGW(TAG, "AHT20 Humidity/Temp sensor NOT FOUND!");
    } else {
        ESP_LOGI(TAG, "AHT20 found.");
    }

    if (!asAvailable && !bmpAvailable && !bmp280Available && !ahtAvailable) {
        ESP_LOGE(TAG, "TOTAL I2C SENSOR FAILURE: Verify hardware connections on SDA:%d SCL:%d.", I2C_SDA_PIN, I2C_SCL_PIN);
    }
}

void SensorManager::calibrateSensors() {
    ESP_LOGI(TAG, "Beginning sensor calibration...");
    // Calibrate BMP (either 390 or 280)
    if (!bmpAvailable && !bmp280Available) {
        ESP_LOGW(TAG, "Calibration: BMP sensor not found, skipping.");
    } else {
        float sum = 0;
        const int samples = 20;
        int validSamples = 0;

        // Initial dummy readings to wake up the sensor IC
        for (int i = 0; i < 3; i++) {
            if (bmpAvailable) bmp.performReading();
            else bmp280.readPressure();
            delay(20);
        }

        for (int i = 0; i < samples; i++) {
            if (bmpAvailable) {
                if (bmp.performReading()) {
                    sum += bmp.pressure;
                    validSamples++;
                }
            } else {
                const float pressure = bmp280.readPressure();
                if (std::isfinite(pressure) && pressure > 0.0f) {
                    sum += pressure;
                    validSamples++;
                }
            }
            delay(50);
        }
        if (validSamples > 0) {
            seaLevelPressure = sum / static_cast<float>(validSamples);
            ESP_LOGI(TAG, "BMP calibrated. Sea level pressure: %.2f Pa", seaLevelPressure);
        } else {
            ESP_LOGW(TAG, "BMP calibration failed: No valid samples obtained.");
        }
    }

    // Calibrate Airspeed
    if (!asAvailable) {
        ESP_LOGW(TAG, "Calibration: Airspeed sensor not found, skipping.");
        return;
    }
    
    float sum = 0;
    const int as_samples = 40;
    int validSamples = 0;

    for (int i = 0; i < as_samples; i++) {
        if (Wire.requestFrom(MS4525DO_ADDR, 2) == 2) {
            uint8_t b0 = Wire.read();
            uint8_t b1 = Wire.read();
            if (((b0 >> 6) & 0x03) != 0) {
                delay(20);
                continue;
            }
            int16_t p_counts = ((b0 & 0x3F) << 8) | b1;
            float diff_press_psi = ((((float)p_counts - 1638.4f) * 2.0f) / 13107.2f) - 1.0f;
            sum += (diff_press_psi * 6894.76f);
            validSamples++;
        }
        delay(20);
    }
    if (validSamples > 0) {
        as_offset_pa = sum / (float)validSamples;
        ESP_LOGI(TAG, "Airspeed sensor calibrated. Offset: %.2f Pa", as_offset_pa);
    } else {
        ESP_LOGW(TAG, "Airspeed sensor calibration failed: No valid samples obtained.");
    }
}

void SensorManager::recoverI2CBus() {
    if (_i2cMutex != nullptr) xSemaphoreTakeRecursive(_i2cMutex, portMAX_DELAY);

    ESP_LOGW(TAG, "Attempting I2C bus recovery...");
    g_i2cRecoveries.fetch_add(1);
    Wire.end();
    
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
    delay(1);

    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT);
    
    // Toggle SCL to force slaves to release SDA if they are stuck mid-byte
    for (int i = 0; i < 20; i++) {
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(10);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(10);
        // If SDA goes high, the bus is likely clear
        if (digitalRead(I2C_SDA_PIN) == HIGH) break;
    }

    pinMode(I2C_SCL_PIN, INPUT_PULLUP); // Release SCL before Wire takes over
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(200); // Standardize timeout with initial begin()
    delay(150); // Allow bus to stabilize after re-initialization
    
    _detectAndConfigureSensors(); // Re-run discovery to reset sensor IC states
    _i2cInitialized = true; 
    ESP_LOGI(TAG, "I2C bus recovery complete. Sensors re-initialized.");

    if (_i2cMutex != nullptr) xSemaphoreGiveRecursive(_i2cMutex);
}

float SensorManager::calculateAltitude(float currentPressure, float referencePressure) {
    if (referencePressure <= 0 || std::isnan(currentPressure)) return NAN;
    return 44330.0f * (1.0f - powf(currentPressure / referencePressure, 0.190263f));
}

float SensorManager::calculateAirDensity(float tempC, float pressurePa) {
    const float GAS_CONSTANT = 287.05f; // J/(kg*K)
    const float tempK = tempC + 273.15f;
    if (!std::isfinite(tempK) || tempK <= 0.0f ||
        !std::isfinite(pressurePa) || pressurePa <= 0.0f) {
        return RHO_AIR_DENSITY;
    }
    return pressurePa / (GAS_CONSTANT * tempK);
}

float SensorManager::estimateClimbRate(float verticalAccelerationMps2,
                                       unsigned long timestampMs) {
    // _alt_last is populated by either BMP390 or BMP280 and remains NAN when
    // no supported barometer is available.
    const float barometricAltitudeM = _alt_last;

    if (_climb_last_ms == 0) {
        _climb_last_ms = timestampMs;
        _climb_last_alt_m = barometricAltitudeM;
        _climb_last_baro_ms = timestampMs;
        return _climb_rate_mps;
    }

    const float dt = (timestampMs - _climb_last_ms) * 0.001f;
    _climb_last_ms = timestampMs;

    // Reject timing gaps so a pause or rollover cannot create a velocity jump.
    if (dt <= 0.0f || dt > 0.25f) {
        _climb_last_alt_m = barometricAltitudeM;
        _climb_last_baro_ms = timestampMs;
        return _climb_rate_mps;
    }

    if (std::isfinite(verticalAccelerationMps2)) {
        // Ignore residual IMU noise/gravity-removal error near zero.
        constexpr float ACCEL_DEADZONE_MPS2 = 0.05f;
        const float a = fabsf(verticalAccelerationMps2) < ACCEL_DEADZONE_MPS2
                            ? 0.0f
                            : verticalAccelerationMps2;
        _climb_rate_mps += a * dt;
    }

    if (std::isfinite(barometricAltitudeM)) {
        const bool altitudeChanged = !std::isfinite(_climb_last_alt_m) ||
            fabsf(barometricAltitudeM - _climb_last_alt_m) > 0.0001f;
        if (altitudeChanged && std::isfinite(_climb_last_alt_m)) {
            const float baroDt = (timestampMs - _climb_last_baro_ms) * 0.001f;
            const float rawBaroRate = baroDt > 0.0f
                ? (barometricAltitudeM - _climb_last_alt_m) / baroDt
                : 0.0f;

            // Reject implausible single-sample pressure spikes, then low-pass
            // the BMP390- or BMP280-derived rate to arrest integration drift.
            if (fabsf(rawBaroRate) <= 20.0f) {
                constexpr float BARO_RATE_ALPHA = 0.08f;
                _climb_baro_rate_mps +=
                    BARO_RATE_ALPHA * (rawBaroRate - _climb_baro_rate_mps);

            }
        }
        if (altitudeChanged) {
            _climb_last_alt_m = barometricAltitudeM;
            _climb_last_baro_ms = timestampMs;
        }

        // Apply the latest barometric correction continuously between the
        // slower pressure-sensor updates.
        constexpr float BARO_CORRECTION = 0.02f;
        _climb_rate_mps += BARO_CORRECTION *
                           (_climb_baro_rate_mps - _climb_rate_mps);
    }

    return _climb_rate_mps;
}

void SensorManager::resetClimbRateEstimator() {
    _climb_rate_mps = 0.0f;
    _climb_baro_rate_mps = 0.0f;
    _climb_last_alt_m = NAN;
    _climb_last_ms = 0;
    _climb_last_baro_ms = 0;
}

bool SensorManager::_readRawAirspeedSensor(float &raw_airspeed_diff_press_pa, float &internal_sensor_temp_c, float ambient_pressure_pa) {
    if (_i2cMutex == nullptr || xSemaphoreTakeRecursive(_i2cMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }
    uint8_t buffer[4];
    uint8_t bytesRead = Wire.requestFrom(MS4525DO_ADDR, 4); 
    if (bytesRead != 4) {
        xSemaphoreGiveRecursive(_i2cMutex);
        return false; 
    }
    for (int i = 0; i < 4; i++) {
        buffer[i] = Wire.read();
    }

    uint8_t status = (buffer[0] >> 6) & 0x03;
    // MS4525DO status 2 means the returned measurement is valid but stale.
    // This is expected when polling faster than a new conversion is produced
    // and must not be treated as an I2C bus failure. Status 1 is command mode
    // and status 3 indicates a diagnostic fault.
    if (status == 1 || status == 3) {
        xSemaphoreGiveRecursive(_i2cMutex);
        return false; 
    }

    int16_t p_counts = ((buffer[0] & 0x3F) << 8) | buffer[1];
    float diff_press_psi = ((((float)p_counts - 1638.4f) * 2.0f) / 13107.2f) - 1.0f;
    raw_airspeed_diff_press_pa = (diff_press_psi * 6894.76f) - as_offset_pa;

    int16_t t_counts = (buffer[2] << 8 | buffer[3]) >> 5;
    internal_sensor_temp_c      = ((float)t_counts * 200.0f / 2047.0f) - 50.0f; 

    xSemaphoreGiveRecursive(_i2cMutex);
    return true;
}

bool SensorManager::_readAhtData() {
    if (!ahtAvailable) return false;
    if (_i2cMutex == nullptr || xSemaphoreTakeRecursive(_i2cMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;

    sensors_event_t humidity, temp;
    bool success = aht.getEvent(&humidity, &temp);
    if (success) {
        _t_last = temp.temperature;
    }

    xSemaphoreGiveRecursive(_i2cMutex);
    return success;
}

bool SensorManager::_readBmpData() {
    bool success = false;

    if (bmpAvailable) {
        if (_i2cMutex == nullptr || xSemaphoreTakeRecursive(_i2cMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
        if (bmp.performReading()) {
            if (!std::isfinite(bmp.temperature) || bmp.temperature <= -80.0f ||
                bmp.temperature >= 100.0f || !std::isfinite(bmp.pressure) ||
                bmp.pressure <= 10000.0f || bmp.pressure >= 120000.0f) {
                xSemaphoreGiveRecursive(_i2cMutex);
                return false;
            }
            _t_last = bmp.temperature;
            _p_last = bmp.pressure;
            _rho_last = calculateAirDensity(_t_last, _p_last);
            float raw_alt = calculateAltitude(_p_last, getSeaLevelPressure());

            if (std::isnan(_ema_alt)) _ema_alt = raw_alt;
            else _ema_alt = (_alt_alpha * raw_alt) + ((1.0f - _alt_alpha) * _ema_alt);
            _alt_last = _ema_alt;
            
            success = true;
        } 
        xSemaphoreGiveRecursive(_i2cMutex);
    } else if (bmp280Available) {
        if (_i2cMutex == nullptr || xSemaphoreTakeRecursive(_i2cMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
        const float temp = bmp280.readTemperature();
        const float pressure = bmp280.readPressure();
        if (std::isfinite(temp) && temp > -80.0f && temp < 100.0f &&
            std::isfinite(pressure) && pressure > 10000.0f && pressure < 120000.0f) {
            _t_last = temp;
            _p_last = pressure;
            _rho_last = calculateAirDensity(_t_last, _p_last);
            const float raw_alt = calculateAltitude(_p_last, getSeaLevelPressure());
            if (std::isnan(_ema_alt)) _ema_alt = raw_alt;
            else _ema_alt = (_alt_alpha * raw_alt) + ((1.0f - _alt_alpha) * _ema_alt);
            _alt_last = _ema_alt;
            success = true;
        }
        xSemaphoreGiveRecursive(_i2cMutex);
    } else {
        _p_last = _alt_last = NAN;
        _rho_last = NAN;
        if (!ahtAvailable) _t_last = NAN;
        success = true; 
    }
    return success;
}

bool SensorManager::_readImuData() {
#ifdef M5STACK_CAPSULE
    if (!imuAvailable || !M5.Imu.update()) return false;

    const m5::imu_data_t data = M5.Imu.getImuData();
    if (!_imuFiltersInitialized) {
        // Seed each filter with the first sample to avoid a startup ramp from zero.
        imu_x_filter.setState(data.accel.x);
        imu_y_filter.setState(data.accel.y);
        imu_z_filter.setState(data.accel.z);
        _imuFiltersInitialized = true;
    }

    _imu_x_last = imu_x_filter.update(data.accel.x);
    _imu_y_last = imu_y_filter.update(data.accel.y);
    _imu_z_last = imu_z_filter.update(data.accel.z);

    // Track the gravity direction in device coordinates. Gyroscope integration
    // supplies fast pitch/roll response; a slow accelerometer correction bounds
    // drift when the measured magnitude is reasonably close to 1 g.
    constexpr float DEG_TO_RAD_F = 0.01745329252f;
    constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
    const unsigned long attitudeNow = millis();
    const float accelMagnitudeG = sqrtf(
        _imu_x_last * _imu_x_last +
        _imu_y_last * _imu_y_last +
        _imu_z_last * _imu_z_last);

    if (!_gravityInitialized && accelMagnitudeG > 0.1f) {
        _gravity_x_g = _imu_x_last / accelMagnitudeG;
        _gravity_y_g = _imu_y_last / accelMagnitudeG;
        _gravity_z_g = _imu_z_last / accelMagnitudeG;
        _gravityInitialized = true;
    } else if (_gravityInitialized) {
        const float dt = (attitudeNow - _imu_attitude_last_ms) * 0.001f;
        if (dt > 0.0f && dt <= 0.25f) {
            const float wx = data.gyro.x * DEG_TO_RAD_F;
            const float wy = data.gyro.y * DEG_TO_RAD_F;
            const float wz = data.gyro.z * DEG_TO_RAD_F;

            const float oldGx = _gravity_x_g;
            const float oldGy = _gravity_y_g;
            const float oldGz = _gravity_z_g;
            _gravity_x_g += (oldGy * wz - oldGz * wy) * dt;
            _gravity_y_g += (oldGz * wx - oldGx * wz) * dt;
            _gravity_z_g += (oldGx * wy - oldGy * wx) * dt;

            if (accelMagnitudeG > 0.9f && accelMagnitudeG < 1.1f) {
                constexpr float ACCEL_GRAVITY_CORRECTION = 0.005f;
                _gravity_x_g += ACCEL_GRAVITY_CORRECTION *
                    ((_imu_x_last / accelMagnitudeG) - _gravity_x_g);
                _gravity_y_g += ACCEL_GRAVITY_CORRECTION *
                    ((_imu_y_last / accelMagnitudeG) - _gravity_y_g);
                _gravity_z_g += ACCEL_GRAVITY_CORRECTION *
                    ((_imu_z_last / accelMagnitudeG) - _gravity_z_g);
            }

            const float gravityNorm = sqrtf(
                _gravity_x_g * _gravity_x_g +
                _gravity_y_g * _gravity_y_g +
                _gravity_z_g * _gravity_z_g);
            if (gravityNorm > 0.1f) {
                _gravity_x_g /= gravityNorm;
                _gravity_y_g /= gravityNorm;
                _gravity_z_g /= gravityNorm;
            }
        }
    }
    _imu_attitude_last_ms = attitudeNow;

    const float linearXG = _imu_x_last - _gravity_x_g;
    const float linearYG = _imu_y_last - _gravity_y_g;
    const float linearZG = _imu_z_last - _gravity_z_g;

    // The Capsule's USB-facing direction is -Y.
    _forward_accel_mps2 = -linearYG * STANDARD_GRAVITY_MPS2;

    // Project linear acceleration onto earth-up (the estimated gravity vector).
    const float verticalAccelerationMps2 =
        (linearXG * _gravity_x_g + linearYG * _gravity_y_g +
         linearZG * _gravity_z_g) * STANDARD_GRAVITY_MPS2;
    _climb_rate_mps = estimateClimbRate(verticalAccelerationMps2, millis());
    return true;
#else
    _imu_x_last = _imu_y_last = _imu_z_last = NAN;
    return false;
#endif
}

bool SensorManager::_readAirspeedData() {
    float raw_diff_press_pa = 0;
    float internal_sensor_temp_c = NAN;
    float raw_as = 0;
    bool success = false; 

    if (asAvailable) {
        if (_readRawAirspeedSensor(raw_diff_press_pa, internal_sensor_temp_c, _p_last)) {
            float calc_temp = std::isnan(_t_last) ? internal_sensor_temp_c : _t_last;
            float calc_press = std::isnan(_p_last) ? getSeaLevelPressure() : _p_last;

            _rho_last = calculateAirDensity(calc_temp, calc_press);
            raw_as = (raw_diff_press_pa > 0) ? sqrtf(2.0f * raw_diff_press_pa / _rho_last) : 0.0f;
            success = true;
        } else {
            success = false;
        }
    } else {
        success = true; 
    }

    getAirspeedData(raw_as, _as_last);
    return success;
}

void SensorManager::recordSample(const QueueData &data) {
    if (!std::isnan(data.pressure))    { _p_acc   += data.pressure; _p_n++; }
    if (!std::isnan(data.temp))        { _t_acc   += data.temp; _t_n++; }
    if (!std::isnan(data.altitude))    { 
        if (!std::isnan(_last_recorded_alt)) {
            _alt_diff_acc += (data.altitude - _last_recorded_alt);
        }
        _last_recorded_alt = data.altitude;
        _alt_acc += data.altitude; 
        _alt_n++; 
    }
    if (!std::isnan(data.airspeed))    { _as_acc += data.airspeed; _as_n++; }
    if (!std::isnan(data.air_rho))     { _rho_acc += data.air_rho; _rho_n++; }
    if (!std::isnan(data.ground_speed)) { _gs_acc += data.ground_speed; _gs_n++; }
    if (!std::isnan(data.climb_rate)) { _climb_rate_acc += data.climb_rate; _climb_rate_n++; }
    if (!std::isnan(data.forward_accel)) { _forward_accel_acc += data.forward_accel; _forward_accel_n++; }
    if (!std::isnan(data.accel_x) && !std::isnan(data.accel_y) && !std::isnan(data.accel_z)) {
        _imu_x_acc += data.accel_x;
        _imu_y_acc += data.accel_y;
        _imu_z_acc += data.accel_z;
        _imu_n++;
    }
}

void SensorManager::getAveragedTelemetry(QueueData &outData, float &altChange) {
    outData = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
    altChange = 0.0f;

    if (_p_n > 0)   outData.pressure = _p_acc / (float)_p_n;
    if (_t_n > 0)   outData.temp     = _t_acc / (float)_t_n;
    if (_alt_n > 0) outData.altitude = _alt_acc / (float)_alt_n;
    if (_as_n > 0)  outData.airspeed = _as_acc / (float)_as_n;
    if (_rho_n > 0) outData.air_rho  = _rho_acc / (float)_rho_n;
    if (_gs_n > 0)  outData.ground_speed = _gs_acc / (float)_gs_n;
    if (_climb_rate_n > 0) outData.climb_rate = _climb_rate_acc / (float)_climb_rate_n;
    if (_forward_accel_n > 0) outData.forward_accel = _forward_accel_acc / (float)_forward_accel_n;
    if (_imu_n > 0) {
        outData.accel_x = _imu_x_acc / (float)_imu_n;
        outData.accel_y = _imu_y_acc / (float)_imu_n;
        outData.accel_z = _imu_z_acc / (float)_imu_n;
    }

    if ((bmpAvailable || bmp280Available)) {
        altChange = _alt_diff_acc;
        if (std::abs(altChange) < ALT_CHANGE_DEADZONE) {
            altChange = 0.0f;
        }
    }

    if (_alt_n == 0) {
        _last_recorded_alt = NAN;
    }

    _p_acc = _t_acc = _alt_acc = _as_acc = _rho_acc = _gs_acc = _climb_rate_acc = _forward_accel_acc = _alt_diff_acc = 0.0f;
    _imu_x_acc = _imu_y_acc = _imu_z_acc = 0.0f;
    _p_n = _t_n = _alt_n = _as_n = _rho_n = _gs_n = 0;
    _climb_rate_n = _forward_accel_n = _imu_n = 0;
}

void SensorManager::getAirspeedData(float measurement, float &smoothed_as) {
    float val = as_filter.update(measurement);
    if (val < 0.03f) {
        val = 0.0f;
        as_filter.setState(0.0f);
    }
    smoothed_as = val;
}

bool SensorManager::updateSingleCore(QueueData &outData) {
    static unsigned long lastBmpRead = 0;
    static unsigned long lastAirspeedRead = 0;
    static unsigned long lastAhtRead = 0;
    static unsigned long lastImuRead = 0;
    static int bmpErrors = 0, ahtErrors = 0, airspeedErrors = 0;
    static unsigned long lastRecovery = 0;

    unsigned long now = millis();
    bool updated = false;
    bool imuUpdated = false;

    if (now - lastImuRead >= IMU_READ_INTERVAL || lastImuRead == 0) {
        lastImuRead = now;
        imuUpdated = _readImuData();
    }

    // --- BMP Reading ---
    if (now - lastBmpRead >= BMP_READ_INTERVAL || lastBmpRead == 0) {
        lastBmpRead = now;
        const bool ok = _readBmpData();
        if (bmpAvailable || bmp280Available) bmpErrors = ok ? 0 : bmpErrors + 1;
    }

    // --- AHT Reading ---
    if (now - lastAhtRead >= AHT_READ_INTERVAL || lastAhtRead == 0) {
        lastAhtRead = now;
        const bool ok = _readAhtData();
        if (ahtAvailable) ahtErrors = ok ? 0 : ahtErrors + 1;
    }

    // --- Airspeed Reading ---
    if (now - lastAirspeedRead >= AIRSPEED_READ_INTERVAL || lastAirspeedRead == 0) {
        lastAirspeedRead = now;
        const bool ok = _readAirspeedData();
        if (asAvailable) airspeedErrors = ok ? 0 : airspeedErrors + 1;
        updated = true; 
    }

    if (bmpErrors >= I2C_MAX_ERRORS || ahtErrors >= I2C_MAX_ERRORS ||
        airspeedErrors >= I2C_MAX_ERRORS) {
        if (now - lastRecovery >= 5000 || lastRecovery == 0) {
            ESP_LOGW(TAG, "I2C recovery trigger: BMP=%d AHT=%d Airspeed=%d",
                     bmpErrors, ahtErrors, airspeedErrors);
            recoverI2CBus();
            lastRecovery = millis();
            bmpErrors = ahtErrors = airspeedErrors = 0;
        }
    }

    _gs_last = (now - g_lastWheelUpdateMillis.load() > 3000) ? NAN : g_ble_ground_speed.load();
    outData = {_t_last, _p_last, _as_last, _alt_last, _rho_last, _gs_last,
               imuUpdated ? _climb_rate_mps : NAN,
               imuUpdated ? _forward_accel_mps2 : NAN,
               imuUpdated ? _imu_x_last : NAN,
               imuUpdated ? _imu_y_last : NAN,
               imuUpdated ? _imu_z_last : NAN};
    return updated;
}

// FreeRTOS task function implementation
void sensorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor task started on Core: %d", xPortGetCoreID());

    SensorManager* sensorManager = static_cast<SensorManager*>(pvParameters);
    if (sensorManager == nullptr) {
        ESP_LOGE(TAG, "Error: SensorManager instance not passed to sensorTask");
        vTaskDelete(NULL);
        return;
    }

    if (sensorQueue == NULL) {
        ESP_LOGE(TAG, "Error: sensorQueue is NULL");
        vTaskDelete(NULL);
        return;
    }

    unsigned long lastBmpRead = 0;
    unsigned long lastAirspeedRead = 0;
    unsigned long lastAhtRead = 0;
    unsigned long lastImuRead = 0;
    int bmpErrors = 0, ahtErrors = 0, airspeedErrors = 0, imuErrors = 0;
    unsigned long lastRecovery = 0;

    for (;;) {
        unsigned long now = millis();
        bool bmpUpdated = false;
        bool asUpdated = false;
        bool ahtUpdated = false;
        bool imuUpdated = false;

        if (now - lastImuRead >= IMU_READ_INTERVAL || lastImuRead == 0) {
            lastImuRead = now;
            imuUpdated = sensorManager->_readImuData();
            if (sensorManager->imuAvailable) imuErrors = imuUpdated ? 0 : imuErrors + 1;
#ifdef M5STACK_CAPSULE
            if (imuErrors >= I2C_MAX_ERRORS) {
                sensorManager->imuAvailable =
                    M5.Imu.begin(&M5.In_I2C, m5::board_t::board_M5Capsule);
                sensorManager->_imuFiltersInitialized = false;
                sensorManager->_gravityInitialized = false;
                sensorManager->resetClimbRateEstimator();
                g_imuRecoveries.fetch_add(1);
                imuErrors = 0;
            }
#endif
        }

        // --- AHT Reading ---
        if (now - lastAhtRead >= AHT_READ_INTERVAL || lastAhtRead == 0) {
            lastAhtRead = now;
            ahtUpdated = sensorManager->_readAhtData();
            if (sensorManager->ahtAvailable) ahtErrors = ahtUpdated ? 0 : ahtErrors + 1;
        }

        // --- BMP Reading ---
        if (now - lastBmpRead >= BMP_READ_INTERVAL || lastBmpRead == 0) {
            lastBmpRead = now;
            bmpUpdated = sensorManager->_readBmpData();
            if (sensorManager->bmpAvailable || sensorManager->bmp280Available)
                bmpErrors = bmpUpdated ? 0 : bmpErrors + 1;
        }

        // --- Airspeed Reading & Smoothing ---
        if (now - lastAirspeedRead >= AIRSPEED_READ_INTERVAL || lastAirspeedRead == 0) {
            lastAirspeedRead = now;
            asUpdated = sensorManager->_readAirspeedData();
            if (sensorManager->asAvailable)
                airspeedErrors = asUpdated ? 0 : airspeedErrors + 1;
        }

        if (bmpErrors >= I2C_MAX_ERRORS || ahtErrors >= I2C_MAX_ERRORS ||
            airspeedErrors >= I2C_MAX_ERRORS) {
            if (now - lastRecovery >= 5000 || lastRecovery == 0) {
                ESP_LOGW(TAG, "I2C recovery trigger: BMP=%d AHT=%d Airspeed=%d",
                         bmpErrors, ahtErrors, airspeedErrors);
                sensorManager->recoverI2CBus();
                lastRecovery = millis();
                bmpErrors = ahtErrors = airspeedErrors = 0;
            }
        }

        // Ground speed timeout check
        if (now - g_lastWheelUpdateMillis.load() > 3000) {
            g_ble_ground_speed.store(NAN);
            sensorManager->_gs_last = NAN;
        } else {
            sensorManager->_gs_last = g_ble_ground_speed.load();
        }

        QueueData data;
        data.temp         = (ahtUpdated || bmpUpdated) ? sensorManager->_t_last : NAN;
        data.pressure     = bmpUpdated ? sensorManager->_p_last    : NAN;
        data.altitude     = bmpUpdated ? sensorManager->_alt_last  : NAN;
        data.airspeed     = asUpdated  ? sensorManager->_as_last   : NAN;
        // Air density comes from the barometer and remains available even when
        // the separate MS4525DO differential-pressure sensor is disconnected.
        data.air_rho      = bmpUpdated ? sensorManager->_rho_last  : NAN;
        data.ground_speed = sensorManager->_gs_last; 
        data.climb_rate   = imuUpdated ? sensorManager->_climb_rate_mps : NAN;
        data.forward_accel = imuUpdated ? sensorManager->_forward_accel_mps2 : NAN;
        data.accel_x      = imuUpdated ? sensorManager->_imu_x_last : NAN;
        data.accel_y      = imuUpdated ? sensorManager->_imu_y_last : NAN;
        data.accel_z      = imuUpdated ? sensorManager->_imu_z_last : NAN;

        if (imuUpdated || bmpUpdated || ahtUpdated || asUpdated) {
            if (xQueueSend(sensorQueue, &data, 0) != pdTRUE) {
                g_sensorQueueDrops.fetch_add(1);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
