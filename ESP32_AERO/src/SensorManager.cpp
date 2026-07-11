/**
 * @file SensorManager.cpp
 * @brief Manages connected sensors with optimized I2C bus sequencing
 */

#include "SensorManager.h"
#include "esp_log.h" // For ESP_LOGx macros
#include <cmath> // For std::isnan
#include <cstring> // For memset

static const char* TAG = "SensorManager";

static float decodeMS4525PressurePa(int16_t p_counts) {
    const float spanCounts = MS4525DO_COUNTS_MAX - MS4525DO_COUNTS_MIN;
    const float spanPsi    = MS4525DO_PRESSURE_MAX_PSI - MS4525DO_PRESSURE_MIN_PSI;
    float diff_press_psi   = (((float)p_counts - MS4525DO_COUNTS_MIN) * spanPsi / spanCounts) + MS4525DO_PRESSURE_MIN_PSI;
    return diff_press_psi * PSI_TO_PA;
}

// Global atomic for ground speed (declared extern in BLEManager.h)
extern std::atomic<float> g_ble_ground_speed;
// Global timestamp for last wheel update (declared extern in BLEManager.h)
extern unsigned long g_lastWheelUpdateMillis;
 
SensorManager::SensorManager() 
    : as_filter(0.02f, 2.5f, 0.0f),
      _t_last(NAN), _p_last(NAN), _alt_last(NAN), _as_last(NAN), _rho_last(NAN), _gs_last(NAN),
      _ema_alt(NAN), _alt_alpha(0.05f),
      _alt_diff_acc(0.0f), _last_recorded_alt(NAN), _last_avg_alt(NAN),
      _p_acc(0.0f), _t_acc(0.0f), _alt_acc(0.0f), _as_acc(0.0f), _rho_acc(0.0f), _gs_acc(0.0f),
      _p_n(0), _t_n(0), _alt_n(0), _as_n(0), _rho_n(0), _gs_n(0),
      _i2cInitialized(false), seaLevelPressure(101325.0f), as_offset_pa(0.0f), _i2cMutex(nullptr) {}

void SensorManager::begin(SystemConfig* config) {
    if (config != nullptr) {
        sysConfig = config;
    }
    
    if (_i2cMutex == nullptr) {
        _i2cMutex = xSemaphoreCreateRecursiveMutex();
    }

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
        return;
    }
    
    float sum = 0;
    int samples = 20;
    int validSamples = 0;

    // Initial dummy readings to wake up the sensor IC
    for(int i=0; i<3; i++) { 
        if(bmpAvailable) bmp.performReading();
        else if(bmp280Available) { bmp280.readPressure(); }
        delay(20); 
    }

    for (int i = 0; i < samples; i++) {
        if (bmpAvailable) {
            if (bmp.performReading()) {
                sum += bmp.pressure;
                validSamples++;
            }
        } else if (bmp280Available) {
            sum += bmp280.readPressure();
            validSamples++;
        }
        delay(50);
    }
    if (validSamples > 0) {
        seaLevelPressure = sum / (float)validSamples;
        ESP_LOGI(TAG, "BMP calibrated. Sea level pressure: %.2f Pa", seaLevelPressure);
    } else {
        ESP_LOGW(TAG, "BMP calibration failed: No valid samples obtained.");
    }

    // Calibrate Airspeed
    if (!asAvailable) {
        ESP_LOGW(TAG, "Calibration: Airspeed sensor not found, skipping.");
        return;
    }
    
    sum = 0;
    const int as_samples = 40;
    validSamples = 0;

    int staleOrFaultSamples = 0;

    for (int i = 0; i < as_samples; i++) {
        if (Wire.requestFrom(MS4525DO_ADDR, 2) == 2) {
            uint8_t b0 = Wire.read();
            uint8_t b1 = Wire.read();
            uint8_t status = (b0 >> 6) & 0x03;
            if (status != 0) {
                staleOrFaultSamples++;
                delay(20);
                continue;
            }
            int16_t p_counts = ((b0 & 0x3F) << 8) | b1;
            sum += decodeMS4525PressurePa(p_counts);
            validSamples++;
        }
        delay(20);
    }
    if (validSamples > 0) {
        as_offset_pa = sum / (float)validSamples;
        ESP_LOGI(TAG, "Airspeed sensor calibrated. Offset: %.2f Pa (%d valid, %d stale/fault)", as_offset_pa, validSamples, staleOrFaultSamples);
    } else {
        ESP_LOGW(TAG, "Airspeed sensor calibration failed: No valid samples obtained (%d stale/fault).", staleOrFaultSamples);
    }
}

void SensorManager::recoverI2CBus() {
    if (_i2cMutex != nullptr) xSemaphoreTakeRecursive(_i2cMutex, portMAX_DELAY);

    ESP_LOGW(TAG, "Attempting I2C bus recovery...");
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
    float p = std::isnan(pressurePa) ? SEA_LEVEL_PRESSURE_PA : pressurePa;
    float rho = p / (GAS_CONSTANT * (tempC + 273.15f));

    if (fabs(rho - RHO_AIR_DENSITY) > (0.10f * RHO_AIR_DENSITY)) {
        return RHO_AIR_DENSITY; 
    }
    return rho;
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
    if (status != 0) {
        xSemaphoreGiveRecursive(_i2cMutex);
        return false; 
    }

    int16_t p_counts = ((buffer[0] & 0x3F) << 8) | buffer[1];
    float raw_pressure_pa = decodeMS4525PressurePa(p_counts);
    raw_airspeed_diff_press_pa = raw_pressure_pa - as_offset_pa; 

    #ifdef DEBUG_AIRSPEED_RAW
    static unsigned long lastRawLog = 0;
    unsigned long now = millis();
    if (now - lastRawLog >= 1000 || lastRawLog == 0) {
        lastRawLog = now;
        Serial.printf("\n[AS] c=%d raw=%.1f off=%.1f corr=%.1f\n", p_counts, raw_pressure_pa, as_offset_pa, raw_airspeed_diff_press_pa);
    }
    #endif

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
            _t_last = bmp.temperature;
            _p_last = bmp.pressure;
            float raw_alt = calculateAltitude(_p_last, getSeaLevelPressure());

            if (std::isnan(_ema_alt)) _ema_alt = raw_alt;
            else _ema_alt = (_alt_alpha * raw_alt) + ((1.0f - _alt_alpha) * _ema_alt);
            _alt_last = _ema_alt;
            
            success = true;
        } 
        xSemaphoreGiveRecursive(_i2cMutex);
    } else if (bmp280Available) {
        if (_i2cMutex == nullptr || xSemaphoreTakeRecursive(_i2cMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
        _t_last = bmp280.readTemperature();
        _p_last = bmp280.readPressure();
        float raw_alt = calculateAltitude(_p_last, getSeaLevelPressure());

        if (std::isnan(_ema_alt)) _ema_alt = raw_alt;
        else _ema_alt = (_alt_alpha * raw_alt) + ((1.0f - _alt_alpha) * _ema_alt);
        _alt_last = _ema_alt;
        
        success = true;
        xSemaphoreGiveRecursive(_i2cMutex);
    } else {
        _p_last = _alt_last = NAN;
        if (!ahtAvailable) _t_last = NAN;
        success = true; 
    }
    return success;
}

bool SensorManager::_readAirspeedData() {
    float raw_diff_press_pa = 0;
    float internal_sensor_temp_c = NAN;
    bool success = false; 

    if (asAvailable) {
        if (_readRawAirspeedSensor(raw_diff_press_pa, internal_sensor_temp_c, _p_last)) {
            float calc_temp = std::isnan(_t_last) ? internal_sensor_temp_c : _t_last;
            float calc_press = std::isnan(_p_last) ? getSeaLevelPressure() : _p_last;

            _rho_last = calculateAirDensity(calc_temp, calc_press);
            float corrected_press_pa = 0.0f;
            if (raw_diff_press_pa > AIRSPEED_PRESSURE_DEADBAND_PA) {
                corrected_press_pa = raw_diff_press_pa;
                _as_zero_candidate_count = 0;
            } else if (!std::isnan(_as_last) && _as_last > 0.1f &&
                       _as_zero_candidate_count < AIRSPEED_ZERO_CONFIRM_SAMPLES) {
                _as_zero_candidate_count++;
                return true;
            } else {
                _as_zero_candidate_count = AIRSPEED_ZERO_CONFIRM_SAMPLES;
            }

            float raw_as = (corrected_press_pa > 0.0f) ? sqrtf(2.0f * corrected_press_pa / _rho_last) : 0.0f;
            getAirspeedData(raw_as, _as_last);
            success = true;
        } else {
            success = false;
        }
    } else {
        _as_last = NAN;
        _rho_last = NAN;
        success = true; 
    }

    return success;
}

void SensorManager::recordSample(const QueueData &data) {
    if (!std::isnan(data.pressure))    { _p_acc   += data.pressure; _p_n++; }
    if (!std::isnan(data.temp))        { _t_acc   += data.temp; _t_n++; }
    if (!std::isnan(data.altitude))    { 
        _last_recorded_alt = data.altitude;
        _alt_acc += data.altitude; 
        _alt_n++; 
    }
    if (!std::isnan(data.airspeed))    { _as_acc += data.airspeed; _as_n++; }
    if (!std::isnan(data.air_rho))     { _rho_acc += data.air_rho; _rho_n++; }
    if (!std::isnan(data.ground_speed)) { _gs_acc += data.ground_speed; _gs_n++; }
}

void SensorManager::getAveragedTelemetry(QueueData &outData, float &altChange) {
    outData = {NAN, NAN, NAN, NAN, NAN, NAN};
    altChange = 0.0f;

    if (_p_n > 0)   outData.pressure = _p_acc / (float)_p_n;
    if (_t_n > 0)   outData.temp     = _t_acc / (float)_t_n;
    if (_alt_n > 0) outData.altitude = _alt_acc / (float)_alt_n;
    if (_as_n > 0)  outData.airspeed = _as_acc / (float)_as_n;
    if (_rho_n > 0) outData.air_rho  = _rho_acc / (float)_rho_n;
    if (_gs_n > 0)  outData.ground_speed = _gs_acc / (float)_gs_n;

    if ((bmpAvailable || bmp280Available) && _alt_n > 0 && !std::isnan(outData.altitude)) {
        if (!std::isnan(_last_avg_alt)) {
            altChange = outData.altitude - _last_avg_alt;
        }
        _last_avg_alt = outData.altitude;

        if (std::abs(altChange) < ALT_CHANGE_DEADZONE) {
            altChange = 0.0f;
        }
    }

    if (_alt_n == 0) {
        _last_recorded_alt = NAN;
        _last_avg_alt = NAN;
    }

    _p_acc = _t_acc = _alt_acc = _as_acc = _rho_acc = _gs_acc = _alt_diff_acc = 0.0f;
    _p_n = _t_n = _alt_n = _as_n = _rho_n = _gs_n = 0;
}

void SensorManager::getAirspeedData(float measurement, float &smoothed_as) {
    float val = as_filter.update(measurement);
    if (val < 0.1f) { 
        val = 0.0f;
        as_filter.setState(0.0f);
    }
    smoothed_as = val;
}

bool SensorManager::updateSingleCore(QueueData &outData) {
    static unsigned long lastBmpRead = 0;
    static unsigned long lastAirspeedRead = 0;
    static unsigned long lastAhtRead = 0;
    static int local_i2c_errors = 0;

    unsigned long now = millis();
    bool updated = false;

    // --- BMP Reading ---
    if (now - lastBmpRead >= BMP_READ_INTERVAL || lastBmpRead == 0) {
        lastBmpRead = now;
        if (_readBmpData()) {
            local_i2c_errors = 0;
        } else {
            local_i2c_errors++;
        }
    }

    // --- AHT Reading ---
    if (now - lastAhtRead >= AHT_READ_INTERVAL || lastAhtRead == 0) {
        lastAhtRead = now;
        if (_readAhtData()) {
            local_i2c_errors = 0;
        }
    }

    // --- Airspeed Reading ---
    if (now - lastAirspeedRead >= AIRSPEED_READ_INTERVAL || lastAirspeedRead == 0) {
        lastAirspeedRead = now;
        if (!_readAirspeedData()) {
            local_i2c_errors++;
        }
        updated = true; 
    }

    if (local_i2c_errors >= I2C_MAX_ERRORS) {
        recoverI2CBus();
        local_i2c_errors = 0;
    }

    _gs_last = (now - g_lastWheelUpdateMillis > 3000) ? NAN : g_ble_ground_speed.load();
    outData = {_t_last, _p_last, _as_last, _alt_last, _rho_last, _gs_last};
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
    int local_i2c_errors = 0;

    for (;;) {
        unsigned long now = millis();
        bool bmpUpdated = false;
        bool asUpdated = false;
        bool ahtUpdated = false;

        // --- AHT Reading ---
        if (now - lastAhtRead >= AHT_READ_INTERVAL || lastAhtRead == 0) {
            lastAhtRead = now;
            ahtUpdated = sensorManager->_readAhtData();
            if (!ahtUpdated && sensorManager->ahtAvailable) local_i2c_errors++;
        }

        // --- BMP Reading ---
        if (now - lastBmpRead >= BMP_READ_INTERVAL || lastBmpRead == 0) {
            lastBmpRead = now;
            bmpUpdated = sensorManager->_readBmpData();
            if (!bmpUpdated && (sensorManager->bmpAvailable || sensorManager->bmp280Available)) {
                local_i2c_errors++;
            } else {
                local_i2c_errors = 0;
            }
        }

        // --- Airspeed Reading & Smoothing ---
        if (now - lastAirspeedRead >= AIRSPEED_READ_INTERVAL || lastAirspeedRead == 0) {
            lastAirspeedRead = now;
            asUpdated = sensorManager->_readAirspeedData();
            if (!asUpdated && sensorManager->asAvailable) local_i2c_errors++;
        }

        if (local_i2c_errors >= I2C_MAX_ERRORS) {
            sensorManager->recoverI2CBus();
            local_i2c_errors = 0;
        }

        // Ground speed timeout check
        if (now - g_lastWheelUpdateMillis > 3000) {
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
        data.air_rho      = asUpdated  ? sensorManager->_rho_last  : NAN;
        data.ground_speed = sensorManager->_gs_last; 

        xQueueSend(sensorQueue, &data, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
