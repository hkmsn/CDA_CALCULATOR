#include "Utilities.h"
#include <SD.h>
#include <LittleFS.h>
#include <cmath>
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Local state for logging

static char          logBuffer[6144];
static size_t        currentBufferIdx     = 0;
static unsigned long lastLogWriteTime     = 0;
static unsigned long lastDashboardLogTime = 0;
const size_t         BUFFER_THRESHOLD = 4096;
const unsigned long  FLUSH_INTERVAL   = 30000; // 30 seconds
static SystemConfig* pConfig          = nullptr;

const char* DASHBOARD_HEADER = "Time       |AirSpd |Dens   |Temp   |Press  |Alt    |AltChg |GndSpd |";

extern TaskHandle_t sensorTaskHandle;
static SemaphoreHandle_t _storageMutex = nullptr;

bool deleteLogFile(); // Forward declaration

void initUtilities(SystemConfig* config) {
    pConfig = config;
    if (_storageMutex == nullptr) _storageMutex = xSemaphoreCreateMutex();
}


void writeLogHeader() {
    if (!pConfig) return;
    // Guard against uninitialized storage
    if (!pConfig->sdAvailable && !pConfig->littleFsAvailable) return;

    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    const char* header = "Time|Airspeed|AirDensity|Temperature|Pressure|Altitude|AltitudeChange|GroundSpeed|\n";
    File f = pConfig->sdAvailable ? SD.open("/aero_log.txt", FILE_APPEND) 
                                  : LittleFS.open("/aero_log.txt", FILE_APPEND);
    if (f) {
        if (f.size() == 0) {
            f.print(header);
        }
        f.close();
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

void serializeTelemetry(char* outBuf, size_t sz, float as, float rho, float t, float p, float alt, float alt_ch, float gs, bool as_ok, bool bmp_ok, bool csc_ok) {
    char s_as[16], s_rho[16], s_t[16], s_p[16], s_alt[16], s_alt_ch[16], s_gs[16];
    fmtNA(s_as,  sizeof(s_as), as, "%.2f", as_ok);
    fmtNA(s_rho, sizeof(s_rho), rho, "%.4f", bmp_ok);
    fmtNA(s_t,   sizeof(s_t), t, "%.2f", bmp_ok);
    fmtNA(s_p,   sizeof(s_p), p, "%.1f", bmp_ok);
    fmtNA(s_alt, sizeof(s_alt), alt, "%.2f", bmp_ok);
    fmtNA(s_alt_ch, sizeof(s_alt_ch), alt_ch, "%.2f", bmp_ok);
    fmtNA(s_gs,     sizeof(s_gs), gs, "%.2f", csc_ok);

    snprintf(outBuf, sz, "%s|%s|%s|%s|%s|%s|%s*", s_as, s_rho, s_t, s_p, s_alt, s_alt_ch, s_gs);
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
    if (!pConfig) return;
    if (currentBufferIdx == 0) return;

    // Guard against uninitialized storage fallback to avoid VFS/LFS crashes
    if (!pConfig->sdAvailable && !pConfig->littleFsAvailable) {
        currentBufferIdx = 0; // Discard pending data
        return;
    }

    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return;
    }

    // Crash Guard: LittleFS can panic with IntegerDivideByZero in lfs_alloc if it runs out of 
    // blocks during file extension. We check for a 4KB safety margin (1 block).
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    if (!pConfig->sdAvailable && (total == 0 || (used + currentBufferIdx + 4096) >= total)) {
        Serial.printf("\n[WARN] LittleFS Full (%u/%u). Auto-clearing log to recover space.\n", 
                      (unsigned int)total, (unsigned int)used, (unsigned int)currentBufferIdx);
        xSemaphoreGive(_storageMutex);
        deleteLogFile(); // Automatically clear log to allow new data
        xSemaphoreGive(_storageMutex);
        return;
    }

    File logFile = pConfig->sdAvailable ? SD.open("/aero_log.txt", FILE_APPEND) 
                                        : LittleFS.open("/aero_log.txt", FILE_APPEND);

    if (logFile) {
        logFile.write((uint8_t*)logBuffer, currentBufferIdx);
        logFile.close();
        Serial.printf("\n[LOG] Flush: %u bytes saved to %s\n", 
                      (unsigned int)currentBufferIdx, pConfig->sdAvailable ? "SD" : "LittleFS");
        lastLogWriteTime = millis();
    } else {
        Serial.println("\n[ERROR] Failed to open log file for block write!");
    }
    currentBufferIdx = 0; // Always clear to prevent retry-loops on permanent storage failure
    xSemaphoreGive(_storageMutex);
}

void displayDashboard(unsigned long now, const char* bleStr) {
    if (!pConfig || !pConfig->serialOutputEnabled) return;

    // Standardize time to HH:MM:SS:CC (uptime based)
    unsigned long totalSeconds = now / 1000;
    int hundredths = (now % 1000) / 10;
    int hh = (totalSeconds / 3600) % 24;
    int mm = (totalSeconds / 60) % 60;
    int ss = totalSeconds % 60;

    // Print to console with a carriage return and padding to clear trailing characters
    // We exclude the trailing newline from bleStr to maintain the single-line dashboard overwrite
    int displayLen = strlen(bleStr);
    if (displayLen > 0 && bleStr[displayLen - 1] == '\n') displayLen--;

    Serial.printf("\r%02d:%02d:%02d:%02d|%.*s   ", hh, mm, ss, hundredths, displayLen, bleStr);
}

void processLoggingAndDashboard(unsigned long now, const char* bleStr) {
    if (now - lastDashboardLogTime >= ESP32_LOGGING_INTERVAL) {
        lastDashboardLogTime = now;
        displayDashboard(now, bleStr);
        logTelemetry(now, bleStr);
    }
}

bool deleteLogFile() {
    if (!pConfig) return false;
    if (!pConfig->sdAvailable && !pConfig->littleFsAvailable) return false;
    if (xSemaphoreTake(_storageMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    
    bool success = pConfig->sdAvailable ? SD.remove("/aero_log.txt") : LittleFS.remove("/aero_log.txt");
    xSemaphoreGive(_storageMutex);
    
    if (success) {
        Serial.println("\n>>> LOG FILE DELETED <<<");
        writeLogHeader();
    }
    return success;
}

void logTelemetry(unsigned long now, const char* bleStr) {
    // Standardize time to HH:MM:SS (uptime based)
    unsigned long totalSeconds = now / 1000;
    int hundredths = (now % 1000) / 10;
    int hh = (totalSeconds / 3600) % 24;
    int mm = (totalSeconds / 60) % 60;
    int ss = totalSeconds % 60;

    if (!pConfig) return;

    if (pConfig->loggingEnabled) {
        // Buffer the data
        char entry[256];
        // Append a newline for storage since it is no longer in bleStr
        int len = snprintf(entry, sizeof(entry), "%02d:%02d:%02d:%02d|%s\n", hh, mm, ss, hundredths, bleStr);
        
        if (len > 0 && (size_t)len < sizeof(entry)) {
            // If buffer is too full for this entry, force a flush first
            if (currentBufferIdx + len >= sizeof(logBuffer)) {
                flushLogBuffer();
            }
            
            if (currentBufferIdx + len < sizeof(logBuffer)) {
                memcpy(&logBuffer[currentBufferIdx], entry, len);
                currentBufferIdx += len;
            }
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
        if (!pConfig->sdAvailable && !pConfig->littleFsAvailable) return;
        flushLogBuffer();
        bool exists = pConfig->sdAvailable ? SD.exists("/aero_log.txt") : LittleFS.exists("/aero_log.txt");
        if (!exists) {
            Serial.println("\n>>> No log file found on storage yet. <<<");
            return;
        }
        Serial.println("\n--- START OF LOG FILE DUMP ---");
        Serial.println("Time|Airspeed|AirDensity|Temperature|Pressure|Altitude|AltitudeChange|GroundSpeed|");
        File logFile = pConfig->sdAvailable ? SD.open("/aero_log.txt", FILE_READ) : LittleFS.open("/aero_log.txt", FILE_READ);
        if (logFile) {
            uint8_t buffer[512];
            while (logFile.available()) {
                size_t bytesRead = logFile.read(buffer, sizeof(buffer));
                Serial.write(buffer, bytesRead);
            }
            logFile.close();
            Serial.println("--- END OF LOG FILE DUMP ---");
        }
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

        bool exists = pConfig->sdAvailable ? SD.exists("/aero_log.txt") : LittleFS.exists("/aero_log.txt");
        if (exists) {
            File f = pConfig->sdAvailable ? SD.open("/aero_log.txt", FILE_READ) : LittleFS.open("/aero_log.txt", FILE_READ);
            Serial.printf("\n  Target File: /aero_log.txt | Current Size: %d bytes\n", f.size());
            f.close();
        } else {
            Serial.println("\n  Target File: /aero_log.txt NOT FOUND (Wait for 30s buffer flush)");
        }
        Serial.printf("  Logging: %s | RAM Buffer: %u bytes\n", pConfig->loggingEnabled ? "ENABLED" : "DISABLED", getLogBufferLength());
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