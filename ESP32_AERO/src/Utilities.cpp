#include "Utilities.h"
#include <SD.h>
#include <LittleFS.h>
#include <cmath>
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SensorManager.h"
#include <atomic>

// Local state for logging

static char          logBuffer[6144];
static size_t        currentBufferIdx     = 0;
static unsigned long lastLogWriteTime     = 0;
static unsigned long lastDashboardLogTime = 0;
const size_t         BUFFER_THRESHOLD = 4096;
const unsigned long  FLUSH_INTERVAL   = 30000; // 30 seconds
static SystemConfig* pConfig          = nullptr;
static constexpr const char* LOG_FILE_PATH = "/aero_log_v2.txt";
static constexpr const char* OLD_LOG_FILE_PATH = "/aero_log_v2.old.txt";
static constexpr const char* LOG_HEADER = "Time|Airspeed|AirDensity|Temperature|Pressure|Altitude|AltitudeChange|GroundSpeed|ClimbRate_mps|ForwardAccel_mps2|AccelX_g|AccelY_g|AccelZ_g|\n";
static std::atomic<uint32_t> storageWriteFailures{0};
static std::atomic<uint32_t> storageFailovers{0};
static std::atomic<uint32_t> serializationTruncations{0};
static std::atomic<uint32_t> logRecordDrops{0};
static File dumpFile;
static bool dumpInProgress = false;

const char* DASHBOARD_HEADER = "Time                   |AirSpd |Dens   |Temp   |Press  |Alt    |AltChg |GndSpd |Climb  |FwdAcc |AccelX |AccelY |AccelZ |";

extern TaskHandle_t sensorTaskHandle;
static SemaphoreHandle_t _storageMutex = nullptr;

bool deleteLogFile(); // Forward declaration

static void formatTimestamp(unsigned long now, char* out, size_t outSize) {
#ifdef M5STACK_CAPSULE
    if (pConfig != nullptr && pConfig->rtcAvailable.load()) {
        m5::rtc_datetime_t rtc;
        if (M5.Rtc.getDateTime(&rtc) && rtc.date.year >= 2024) {
            snprintf(out, outSize, "%04d-%02d-%02d %02d:%02d:%02d.00",
                     rtc.date.year, rtc.date.month, rtc.date.date,
                     rtc.time.hours, rtc.time.minutes, rtc.time.seconds);
            return;
        }
        pConfig->rtcAvailable.store(false);
    }
#endif
    const unsigned long totalSeconds = now / 1000;
    const int hundredths = (now % 1000) / 10;
    snprintf(out, outSize, "%02d:%02d:%02d:%02d",
             static_cast<int>((totalSeconds / 3600) % 24),
             static_cast<int>((totalSeconds / 60) % 60),
             static_cast<int>(totalSeconds % 60), hundredths);
}

void initUtilities(SystemConfig* config) {
    pConfig = config;
    if (_storageMutex == nullptr) _storageMutex = xSemaphoreCreateMutex();
    if (_storageMutex == nullptr && pConfig != nullptr) {
        pConfig->sdAvailable.store(false);
        pConfig->littleFsAvailable.store(false);
        Serial.println("[ERROR] Storage mutex allocation failed; logging disabled.");
    }
}


void writeLogHeader() {
    if (!pConfig || _storageMutex == nullptr) return;
    // Guard against uninitialized storage
    if (!pConfig->sdAvailable && !pConfig->littleFsAvailable) return;

    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    File f = pConfig->sdAvailable.load() ? SD.open(LOG_FILE_PATH, FILE_APPEND)
                                         : LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
    if (!f && pConfig->sdAvailable.load() && pConfig->littleFsAvailable.load()) {
        pConfig->sdAvailable.store(false);
        storageFailovers.fetch_add(1);
        f = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
    }
    if (f) {
        if (f.size() == 0) {
            f.print(LOG_HEADER);
        }
        f.close();
    } else {
        storageWriteFailures.fetch_add(1);
    }
    xSemaphoreGive(_storageMutex);
}

void printDashboardHeader() {
    Serial.println(DASHBOARD_HEADER);
}

void fmtNA(char* buf, size_t sz, float val, const char* fmt, bool available) {
    if (available && !std::isnan(val)) {
        snprintf(buf, sz, fmt, val);
    } else {
        snprintf(buf, sz, "NA");
    }
}

void serializeTelemetry(char* outBuf, size_t sz, float as, float rho, float t,
                        float p, float alt, float alt_ch, float gs,
                        float climb_rate, float forward_accel,
                        float accel_x, float accel_y, float accel_z,
                        bool as_ok, bool temp_ok, bool bmp_ok,
                        bool csc_ok, bool imu_ok) {
    char s_as[16], s_rho[16], s_t[16], s_p[16], s_alt[16], s_alt_ch[16], s_gs[16];
    char s_climb[16], s_forward_accel[16], s_accel_x[16], s_accel_y[16], s_accel_z[16];
    fmtNA(s_as,  sizeof(s_as), as, "%.2f", as_ok);
    fmtNA(s_rho, sizeof(s_rho), rho, "%.4f", bmp_ok);
    fmtNA(s_t,   sizeof(s_t), t, "%.2f", temp_ok);
    fmtNA(s_p,   sizeof(s_p), p, "%.1f", bmp_ok);
    fmtNA(s_alt, sizeof(s_alt), alt, "%.2f", bmp_ok);
    fmtNA(s_alt_ch, sizeof(s_alt_ch), alt_ch, "%.2f", bmp_ok);
    fmtNA(s_gs,     sizeof(s_gs), gs, "%.2f", csc_ok);
    fmtNA(s_climb,  sizeof(s_climb), climb_rate, "%.2f", imu_ok);
    fmtNA(s_forward_accel, sizeof(s_forward_accel), forward_accel, "%.3f", imu_ok);
    fmtNA(s_accel_x, sizeof(s_accel_x), accel_x, "%.4f", imu_ok);
    fmtNA(s_accel_y, sizeof(s_accel_y), accel_y, "%.4f", imu_ok);
    fmtNA(s_accel_z, sizeof(s_accel_z), accel_z, "%.4f", imu_ok);

    const int written = snprintf(
        outBuf, sz, "%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s|%s*",
        s_as, s_rho, s_t, s_p, s_alt, s_alt_ch, s_gs,
        s_climb, s_forward_accel, s_accel_x, s_accel_y, s_accel_z);
    if (written < 0 || static_cast<size_t>(written) >= sz) {
        serializationTruncations.fetch_add(1);
        if (sz > 0) outBuf[0] = '\0';
    }
}

void serializeBLETelemetry(char* outBuf, size_t sz, float as, float rho, float alt, float alt_ch, bool as_ok, bool bmp_ok) {
    char s_as[8], s_rho[8], s_alt[8], s_alt_ch[8];
    // Optimized precision for Garmin 20-byte limit
    // Example: "12.3|1.18|1234|0.1*" = 19 bytes
    fmtNA(s_as,     sizeof(s_as),     as,     "%.1f", as_ok);  // 1 decimal
    fmtNA(s_rho,    sizeof(s_rho),    rho,    "%.2f", bmp_ok); // 2 decimals
    fmtNA(s_alt,    sizeof(s_alt),    alt,    "%.0f", bmp_ok); // Integer
    fmtNA(s_alt_ch, sizeof(s_alt_ch), alt_ch, "%.1f", bmp_ok); // 1 decimal

    snprintf(outBuf, sz, "%s|%s|%s|%s*", s_as, s_rho, s_alt, s_alt_ch);
}

void flushLogBuffer() {
    if (!pConfig || _storageMutex == nullptr) return;
    if (currentBufferIdx == 0) return;
    if (dumpInProgress) return;

    // Guard against uninitialized storage fallback to avoid VFS/LFS crashes
    if (!pConfig->sdAvailable.load() && !pConfig->littleFsAvailable.load()) return;

    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return;
    }

    // Crash Guard: LittleFS can panic with IntegerDivideByZero in lfs_alloc if it runs out of 
    // blocks during file extension. We check for a 4KB safety margin (1 block).
    if (!pConfig->sdAvailable.load()) {
        const size_t total = LittleFS.totalBytes();
        const size_t used  = LittleFS.usedBytes();
        if (total == 0) {
            storageWriteFailures.fetch_add(1);
            xSemaphoreGive(_storageMutex);
            return;
        }
        // Rotate at roughly half capacity so both the retained previous log and
        // the new active log can coexist without exhausting the filesystem.
        if ((used + currentBufferIdx) >= (total * 55U / 100U)) {
            Serial.printf("\n[WARN] LittleFS rotation threshold reached (%u/%u).\n",
                          (unsigned int)used, (unsigned int)total);
            LittleFS.remove(OLD_LOG_FILE_PATH);
            bool rotated = true;
            if (LittleFS.exists(LOG_FILE_PATH)) {
                rotated = LittleFS.rename(LOG_FILE_PATH, OLD_LOG_FILE_PATH);
            }
            if (!rotated) {
                storageWriteFailures.fetch_add(1);
                xSemaphoreGive(_storageMutex);
                return;
            }
            File fresh = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
            if (fresh) {
                fresh.print(LOG_HEADER);
                fresh.close();
            } else {
                storageWriteFailures.fetch_add(1);
                xSemaphoreGive(_storageMutex);
                return;
            }
        }
    }

    bool writingToSd = pConfig->sdAvailable.load();
    File logFile = writingToSd ? SD.open(LOG_FILE_PATH, FILE_APPEND)
                               : LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
    if (!logFile && writingToSd && pConfig->littleFsAvailable.load()) {
        pConfig->sdAvailable.store(false);
        storageFailovers.fetch_add(1);
        writingToSd = false;
        logFile = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
        if (logFile && logFile.size() == 0) logFile.print(LOG_HEADER);
    }

    if (logFile) {
        const size_t bytesWritten = logFile.write((uint8_t*)logBuffer, currentBufferIdx);
        logFile.close();
        if (bytesWritten == currentBufferIdx) {
            Serial.printf("\n[LOG] Flush: %u bytes saved to %s\n",
                          (unsigned int)currentBufferIdx, writingToSd ? "SD" : "LittleFS");
            currentBufferIdx = 0;
            lastLogWriteTime = millis();
        } else {
            storageWriteFailures.fetch_add(1);
            if (writingToSd && pConfig->littleFsAvailable.load()) {
                pConfig->sdAvailable.store(false);
                storageFailovers.fetch_add(1);
            }
            Serial.printf("\n[ERROR] Short log write: %u/%u bytes; buffer retained.\n",
                          (unsigned int)bytesWritten, (unsigned int)currentBufferIdx);
        }
    } else {
        storageWriteFailures.fetch_add(1);
        Serial.println("\n[ERROR] Failed to open log file; buffer retained for retry.");
    }
    xSemaphoreGive(_storageMutex);
}

void displayDashboard(unsigned long now, const char* bleStr) {
    if (!pConfig || !pConfig->serialOutputEnabled) return;
    if (dumpInProgress) return;

    char timestamp[32];
    formatTimestamp(now, timestamp, sizeof(timestamp));

    // Print to console with a carriage return and padding to clear trailing characters
    // We exclude the trailing newline from bleStr to maintain the single-line dashboard overwrite
    int displayLen = strlen(bleStr);
    if (displayLen > 0 && bleStr[displayLen - 1] == '\n') displayLen--;

    Serial.printf("\r%s|%.*s   ", timestamp, displayLen, bleStr);
}

void processLoggingAndDashboard(unsigned long now, const char* bleStr) {
    if (now - lastDashboardLogTime >= ESP32_LOGGING_INTERVAL) {
        lastDashboardLogTime = now;
        displayDashboard(now, bleStr);
        logTelemetry(now, bleStr);
    }
}

bool deleteLogFile() {
    if (!pConfig || _storageMutex == nullptr) return false;
    if (dumpInProgress) return false;
    if (!pConfig->sdAvailable && !pConfig->littleFsAvailable) return false;
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    // A clear command also discards telemetry that has not yet been persisted;
    // otherwise pre-clear records would reappear on the next flush.
    currentBufferIdx = 0;
    
    bool success = pConfig->sdAvailable.load() ? SD.remove(LOG_FILE_PATH)
                                               : LittleFS.remove(LOG_FILE_PATH);
    if (!success && pConfig->sdAvailable.load() && pConfig->littleFsAvailable.load()) {
        pConfig->sdAvailable.store(false);
        storageFailovers.fetch_add(1);
        success = LittleFS.remove(LOG_FILE_PATH);
    }
    xSemaphoreGive(_storageMutex);
    
    if (success) {
        Serial.println("\n>>> LOG FILE DELETED <<<");
        writeLogHeader();
    }
    return success;
}

void logTelemetry(unsigned long now, const char* bleStr) {
    if (!pConfig) return;

    if (pConfig->loggingEnabled) {
        // Buffer the data
        char entry[256];
        char timestamp[32];
        formatTimestamp(now, timestamp, sizeof(timestamp));
        // Append a newline for storage since it is no longer in bleStr
        int len = snprintf(entry, sizeof(entry), "%s|%s\n", timestamp, bleStr);
        
        if (len > 0 && (size_t)len < sizeof(entry)) {
            // If buffer is too full for this entry, force a flush first
            if (currentBufferIdx + len >= sizeof(logBuffer)) {
                flushLogBuffer();
            }
            
            if (currentBufferIdx + len < sizeof(logBuffer)) {
                memcpy(&logBuffer[currentBufferIdx], entry, len);
                currentBufferIdx += len;
            } else {
                logRecordDrops.fetch_add(1);
            }
        } else if (len >= static_cast<int>(sizeof(entry))) {
            serializationTruncations.fetch_add(1);
        }
    }

    /**
     * Flush Logic:
     * 1. 30 seconds have elapsed since last write.
     * 2. Buffer has exceeded the size threshold.
     * 3. Logging was just disabled and there is pending data.
     */
    bool timeToFlush = (now - lastLogWriteTime >= FLUSH_INTERVAL && currentBufferIdx > 0);
    bool bufferFull  = (currentBufferIdx > BUFFER_THRESHOLD);
    bool flushOnStop = (!pConfig->loggingEnabled && currentBufferIdx > 0);

    if ((pConfig->loggingEnabled && (timeToFlush || bufferFull)) || flushOnStop) {
        flushLogBuffer();
    }
}

size_t getLogBufferLength() {
    return currentBufferIdx;
}

void listPartitions() {
    Serial.println("\n--- Flash Partition Table ---");
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it != NULL) {
        const esp_partition_t *p = esp_partition_get(it);
        Serial.printf("  Label: %-12s | Type: 0x%02x | Sub: 0x%02x | Addr: 0x%08x | Size: %7d bytes (%d KB)\n",
                      p->label, p->type, p->subtype, p->address, p->size, p->size / 1024);
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);
    Serial.println("-----------------------------\n");
}

void printMenu() {
    Serial.println("\n--- Operational Commands ---");
    Serial.println("  [S] - Toggle Logging (Start/Stop)");
    Serial.println("  [D] - Flush Buffer & Dump Log File");
    Serial.println("  [I] - Show Storage & File Status");
    Serial.println("  [C] - Clear/Delete Log File");
    Serial.println("  [H] - Halt System (Stop all tasks)");
    Serial.println("  [V] - Toggle Dashboard View");
    Serial.println("  [M] - Show this Menu again");
    Serial.println("---------------------------\n");
}

void handleSerialCommand(char c) {
    if (!pConfig) return;

    if (c == 's' || c == 'S') {
        pConfig->loggingEnabled = !pConfig->loggingEnabled;
        Serial.printf("\n>>> LOGGING %s <<<\n", pConfig->loggingEnabled ? "ENABLED" : "DISABLED");
        if (pConfig->serialOutputEnabled) printDashboardHeader();
    } else if (c == 'd' || c == 'D') {
        if (dumpInProgress) {
            Serial.println("\n>>> Log dump already in progress. <<<");
            return;
        }
        if (!pConfig->sdAvailable && !pConfig->littleFsAvailable) return;
        flushLogBuffer();
        bool exists = pConfig->sdAvailable ? SD.exists(LOG_FILE_PATH) : LittleFS.exists(LOG_FILE_PATH);
        if (!exists) {
            Serial.println("\n>>> No log file found on storage yet. <<<");
            return;
        }
        Serial.println("\n--- START OF LOG FILE DUMP ---");
        dumpFile = pConfig->sdAvailable.load() ? SD.open(LOG_FILE_PATH, FILE_READ)
                                               : LittleFS.open(LOG_FILE_PATH, FILE_READ);
        if (!dumpFile && pConfig->sdAvailable.load() && pConfig->littleFsAvailable.load()) {
            pConfig->sdAvailable.store(false);
            storageFailovers.fetch_add(1);
            dumpFile = LittleFS.open(LOG_FILE_PATH, FILE_READ);
        }
        dumpInProgress = static_cast<bool>(dumpFile);
        if (!dumpInProgress) Serial.println("[ERROR] Unable to open log for dump.");
    } else if (c == 'c' || c == 'C') {
        deleteLogFile();
    } else if (c == 'i' || c == 'I') {
        Serial.println("\n--- Storage Status ---");
        Serial.printf("  Active Storage: %s\n", pConfig->sdAvailable ? "SD Card (TF)" : "Internal Flash (LittleFS)");

        size_t total = pConfig->sdAvailable ? SD.totalBytes() : LittleFS.totalBytes();
        size_t used  = pConfig->sdAvailable ? SD.usedBytes() : LittleFS.usedBytes();
        Serial.printf("  Capacity: %u KB / %u KB used (%.1f%% full)\n", 
                      used/1024, total/1024, (total > 0) ? ((float)used*100.0/total) : 0);
        
        Serial.println("  Directory Listing:");
        File root = pConfig->sdAvailable ? SD.open("/") : LittleFS.open("/");
        File file = root.openNextFile();
        if (!file) Serial.println("    (No files found)");
        while(file){
            Serial.printf("    %-16s | %d bytes\n", file.name(), file.size());
            file = root.openNextFile();
        }
        root.close();

        bool exists = pConfig->sdAvailable ? SD.exists(LOG_FILE_PATH) : LittleFS.exists(LOG_FILE_PATH);
        if (exists) {
            File f = pConfig->sdAvailable ? SD.open(LOG_FILE_PATH, FILE_READ) : LittleFS.open(LOG_FILE_PATH, FILE_READ);
            Serial.printf("\n  Target File: %s | Current Size: %d bytes\n", LOG_FILE_PATH, f.size());
            f.close();
        } else {
            Serial.printf("\n  Target File: %s NOT FOUND (Wait for 30s buffer flush)\n", LOG_FILE_PATH);
        }
        Serial.printf("  Logging: %s | RAM Buffer: %u bytes\n", pConfig->loggingEnabled ? "ENABLED" : "DISABLED", getLogBufferLength());
        Serial.printf("  Clock: %s\n", pConfig->rtcAvailable.load() ? "Capsule RTC" : "uptime");
        Serial.printf("  Health: writeFailures=%u failovers=%u logDrops=%u queueDrops=%u i2cRecoveries=%u imuRecoveries=%u truncations=%u\n",
                      storageWriteFailures.load(), storageFailovers.load(),
                      logRecordDrops.load(), g_sensorQueueDrops.load(), g_i2cRecoveries.load(),
                      g_imuRecoveries.load(), serializationTruncations.load());
        Serial.println("----------------------");
    } else if (c == 'h' || c == 'H') {
        Serial.println("\n>>> SYSTEM HALTING... <<<");
        flushLogBuffer();
        if (sensorTaskHandle != NULL) {
            vTaskDelete(sensorTaskHandle);
            sensorTaskHandle = NULL;
        }
        Serial.println("Tasks terminated. System Halted. Reset to restart.");
        while (true) delay(1000);
    } else if (c == 'v' || c == 'V') {
        pConfig->serialOutputEnabled = !pConfig->serialOutputEnabled;
        Serial.printf("\n>>> DASHBOARD %s <<<\n", pConfig->serialOutputEnabled ? "ON" : "OFF");
        if (pConfig->serialOutputEnabled) printDashboardHeader();
    } else if (c == 'm' || c == 'M') {
        printMenu();
    }
}

void serviceUtilities() {
    if (!dumpInProgress) return;
    uint8_t buffer[512];
    const size_t bytesRead = dumpFile.read(buffer, sizeof(buffer));
    if (bytesRead > 0) Serial.write(buffer, bytesRead);
    if (bytesRead == 0 || !dumpFile.available()) {
        dumpFile.close();
        dumpInProgress = false;
        Serial.println("--- END OF LOG FILE DUMP ---");
    }
}
