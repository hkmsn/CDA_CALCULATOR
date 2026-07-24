#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>
#include "SystemConfig.h"

/**
 * @brief Initializes the utilities module with system configuration.
 */
void initUtilities(SystemConfig* config);
void displayDashboard(unsigned long now, const char* bleStr);

/**
 * @brief Periodically updates the dashboard and logs telemetry based on ESP32_LOGGING_INTERVAL.
 * @param now Current uptime in milliseconds.
 * @param bleStr The formatted telemetry string.
 */
void processLoggingAndDashboard(unsigned long now, const char* bleStr);

/**
 * @brief Writes the CSV header to the log file if the file is new or empty.
 */
void writeLogHeader();

/**
 * @brief Buffers telemetry data and periodically flushes to storage.
 * @param now Current uptime in milliseconds.
 * @param bleStr The formatted telemetry string to log.
 */
void logTelemetry(unsigned long now, const char* bleStr);

/**
 * @brief Helper to format a float to a string or return "NA" if invalid/unavailable.
 */
void fmtNA(char* buf, size_t sz, float val, const char* fmt, bool available);

/**
 * @brief Serializes sensor data into a pipe-delimited string.
 */
void serializeTelemetry(char* outBuf, size_t sz, float as, float rho, float t,
                        float p, float alt, float alt_ch, float gs,
                        float climb_rate, float forward_accel,
                        float accel_x, float accel_y, float accel_z,
                        bool as_ok, bool temp_ok, bool bmp_ok,
                        bool csc_ok, bool imu_ok);

/**
 * @brief Manually flushes the current RAM buffer to the active storage.
 */
void flushLogBuffer();

/**
 * @brief Returns the current size of the RAM log buffer in bytes.
 */
size_t getLogBufferLength();

/**
 * @brief Lists all flash partitions to Serial for diagnostics.
 */
void listPartitions();

/**
 * @brief Prints the operational command menu to the Serial console.
 */
void printMenu();

/**
 * @brief Processes incoming serial characters and executes corresponding commands.
 */
void handleSerialCommand(char c);
void serviceUtilities();

#endif // UTILITIES_H
