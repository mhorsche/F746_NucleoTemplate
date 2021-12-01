/**
 * @file core_modbus_config_defaults.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/**
 * @file core_modbus_config_defaults.h
 * @brief This represents the default values for the configuration macros
 * for the MODBUS library.
 *
 * @note This file SHOULD NOT be modified. If custom values are needed for
 * any configuration macro, a core_modbus_config.h file should be provided to
 * the MODBUS library to override the default values defined in this file.
 * To use the custom config file, the MODBUS_DO_NOT_USE_CUSTOM_CONFIG preprocessor
 * macro SHOULD NOT be set.
 */

#ifndef CORE_MODBUS_CONFIG_DEFAULTS_H_
#define CORE_MODBUS_CONFIG_DEFAULTS_H_

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C"
{
#endif
/* *INDENT-ON* */

/* The macro definition for MODBUS_DO_NOT_USE_CUSTOM_CONFIG is for Doxygen
 * documentation only. */

/**
 * @brief Define this macro to build the MODBUS library without the custom config
 * file core_modbus_config.h.
 *
 * Without the custom config, the MODBUS library builds with
 * default values of config macros defined in core_modbus_config_defaults.h file.
 *
 * If a custom config is provided, then MODBUS_DO_NOT_USE_CUSTOM_CONFIG should not
 * be defined.
 */
#ifdef DOXYGEN
#define MODBUS_DO_NOT_USE_CUSTOM_CONFIG
#endif

/**
 * @brief The size of the MODBUS PDU is limited by the size constraint inherited from
 * the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
 * bytes).
 * Therefore: MODBUS PDU for serial line communication = 256 - Server address
 * (1 byte) - CRC (2 bytes) = 253 bytes.
 */
#ifndef MODBUS_MAX_PDU_BUFFER_LEN
#define MODBUS_MAX_PDU_BUFFER_LEN (253)
#endif
#ifndef MODBUS_MAX_ADU_BUFFER_LEN
/* TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes. */
#define MODBUS_MAX_ADU_BUFFER_LEN (MODBUS_MAX_PDU_BUFFER_LEN + 7)
#endif

/**
 * @brief Quantity of coils 1 to 2000 (0x7D0)
 */
#ifndef MODBUS_MAX_COIL_LENGTH
#define MODBUS_MAX_COIL_LENGTH (2000)
#endif

/**
 * @brief Quantity of Registers 1 to 125 (0x7D)
 */
#ifndef MODBUS_MAX_REGISTER_LENGTH
#define MODBUS_MAX_REGISTER_LENGTH (125)
#endif

/**
 * @brief Macro that is called in the MODBUS library for logging "Error" level
 * messages.
 *
 * To enable error level logging in the MODBUS library, this macro should be mapped to the
 * application-specific logging implementation that supports error logging.
 *
 * @note This logging macro is called in the MODBUS library with parameters wrapped in
 * double parentheses to be ISO C89/C90 standard compliant. For a reference
 * POSIX implementation of the logging macros, refer to core_modbus_config.h files, and the
 * logging-stack in demos folder of the
 * [AWS IoT Embedded C SDK repository](https://github.com/aws/aws-iot-device-sdk-embedded-C).
 *
 * <b>Default value</b>: Error logging is turned off, and no code is generated for calls
 * to the macro in the MODBUS library on compilation.
 */
#ifndef LogError
#define LogError(message)
#endif

/**
 * @brief Macro that is called in the MODBUS library for logging "Warning" level
 * messages.
 *
 * To enable warning level logging in the MODBUS library, this macro should be mapped to the
 * application-specific logging implementation that supports warning logging.
 *
 * @note This logging macro is called in the MODBUS library with parameters wrapped in
 * double parentheses to be ISO C89/C90 standard compliant. For a reference
 * POSIX implementation of the logging macros, refer to core_modbus_config.h files, and the
 * logging-stack in demos folder of the
 * [AWS IoT Embedded C SDK repository](https://github.com/aws/aws-iot-device-sdk-embedded-C/).
 *
 * <b>Default value</b>: Warning logs are turned off, and no code is generated for calls
 * to the macro in the MODBUS library on compilation.
 */
#ifndef LogWarn
#define LogWarn(message)
#endif

/**
 * @brief Macro that is called in the MODBUS library for logging "Info" level
 * messages.
 *
 * To enable info level logging in the MODBUS library, this macro should be mapped to the
 * application-specific logging implementation that supports info logging.
 *
 * @note This logging macro is called in the MODBUS library with parameters wrapped in
 * double parentheses to be ISO C89/C90 standard compliant. For a reference
 * POSIX implementation of the logging macros, refer to core_modbus_config.h files, and the
 * logging-stack in demos folder of the
 * [AWS IoT Embedded C SDK repository](https://github.com/aws/aws-iot-device-sdk-embedded-C/).
 *
 * <b>Default value</b>: Info logging is turned off, and no code is generated for calls
 * to the macro in the MODBUS library on compilation.
 */
#ifndef LogInfo
#define LogInfo(message)
#endif

/**
 * @brief Macro that is called in the MODBUS library for logging "Debug" level
 * messages.
 *
 * To enable debug level logging from MODBUS library, this macro should be mapped to the
 * application-specific logging implementation that supports debug logging.
 *
 * @note This logging macro is called in the MODBUS library with parameters wrapped in
 * double parentheses to be ISO C89/C90 standard compliant. For a reference
 * POSIX implementation of the logging macros, refer to core_modbus_config.h files, and the
 * logging-stack in demos folder of the
 * [AWS IoT Embedded C SDK repository](https://github.com/aws/aws-iot-device-sdk-embedded-C/).
 *
 * <b>Default value</b>: Debug logging is turned off, and no code is generated for calls
 * to the macro in the MODBUS library on compilation.
 */
#ifndef LogDebug
#define LogDebug(message)
#endif

/* *INDENT-OFF* */
#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */

#endif /* ifndef CORE_MODBUS_CONFIG_DEFAULTS_H_ */
