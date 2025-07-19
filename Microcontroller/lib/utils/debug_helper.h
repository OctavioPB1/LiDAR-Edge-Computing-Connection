/**
 * @file debug_helper.h
 * @author  Ossola Florencia (flor.ossola13@gmail.com)
 * @brief Debugging and logging helper for ESP32.
 * 
 * This header file defines logging macros and functions for managing debug 
 * messages in an ESP32 project. It provides support for conditional debugging 
 * and different log levels.
 * 
 * @date 2025-02-09
 * @version 1.0
 */

#ifndef DEBUG_HELPER_H_
#define DEBUG_HELPER_H_

// If DEBUG is not defined elsewhere, define it here with a default value
#ifndef DEBUG
    #define DEBUG 0  /**< Set to 0 to disable debugging logs */
#endif

/**
 * @brief Macro for conditional debugging.
 * 
 * If debugging is enabled (DEBUG != 0), this macro executes the provided 
 * logging arguments. Otherwise, it does nothing.
 */

#if DEBUG
    #define DEBUGING_ESP_LOG(...) do { __VA_ARGS__; } while (0)
#else
    #define DEBUGING_ESP_LOG(...) do {} while (0)
#endif


/**
 * @brief Logs a message with a specified log level.
 * 
 * This function sends a formatted log message to the appropriate log handler 
 * based on the level of severity.
 * 
 * @param[in] level Log level (e.g., LOG_ERROR, LOG_WARNING, LOG_INFO).
 * @param[in] TAG   Tag to identify the source of the log message.
 * @param[in] fmt   Log message format string.
 */
void LOG_MESSAGE(int level, const char *TAG, char *fmt);

// Log level definitions
#define LOG_ERROR   1
#define LOG_WARNING 2
#define LOG_INFO    3

/**
 * @brief Logs an error message.
 */
#define LOG_MESSAGE_E(TAG, fmt) LOG_MESSAGE(LOG_ERROR, TAG, fmt)

/**
 * @brief Logs a warning message.
 */
#define LOG_MESSAGE_W(TAG, fmt) LOG_MESSAGE(LOG_WARNING, TAG, fmt)

/**
 * @brief Logs an informational message.
 */
#define LOG_MESSAGE_I(TAG, fmt) LOG_MESSAGE(LOG_INFO, TAG, fmt)

#endif // DEBUG_HELPER_H_
