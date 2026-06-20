/**
 * @file    acrosched_config.h
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    2026-05-14
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef ACROSCHED_CONFIG_H_
#define ACROSCHED_CONFIG_H_

/******************************** Included files ******************************/
#include "acrosched_port.h"

/********************************* Definitions ********************************/

/**
 * @def ACROSCHED_VERSION_MAJOR
 * @brief Major version number of AcroSched (breaking API changes).
 */
#define ACROSCHED_VERSION_MAJOR     1

/**
 * @def ACROSCHED_VERSION_MINOR
 * @brief Minor version number of AcroSched (backwards-compatible additions).
 */
#define ACROSCHED_VERSION_MINOR     0

/**
 * @def ACROSCHED_VERSION_PATCH
 * @brief Patch version number of AcroSched (backwards-compatible bug fixes).
 */
#define ACROSCHED_VERSION_PATCH     0

/*----------------------------------------------------------------------------*/

/**
 * @def ACROSCHED_MAX_TASKS
 * @brief Maximum number of tasks in the static task pool.
 * @details Defines the capacity of the statically allocated task array.
 * Increase to support more concurrent tasks at the cost of RAM; decrease on
 * severely RAM-constrained targets (for example MSP430 with 256 bytes of RAM).
 * The pool dominates the static RAM footprint, so each unit costs one
 * SAcroProcess_t (28 bytes on 32-bit / 16 bytes on 16-bit targets).
 * Override in the port header (acrosched_port.h) or with a compiler -D flag.
 * Must be greater than 0.
 */
#ifndef ACROSCHED_MAX_TASKS
#define ACROSCHED_MAX_TASKS         8U
#endif /* ACROSCHED_MAX_TASKS */

/*----------------------------------------------------------------------------*/

/**
 * @def ACROSCHED_USE_WATCHDOG
 * @brief Enable (1) or disable (0) the watchdog refresh hook.
 * @details When enabled, acroWatchdogRefresh() is called from inside the
 * main dispatcher loop. The application provides a strong definition of
 * acroWatchdogRefresh() to kick the hardware watchdog.
 * When disabled, the module contributes zero code size.
 */
#ifndef ACROSCHED_USE_WATCHDOG
#define ACROSCHED_USE_WATCHDOG      0
#endif /* ACROSCHED_USE_WATCHDOG */

/**
 * @def ACROSCHED_USE_IPC
 * @brief Enable (1) or disable (0) the IPC module.
 * @details When disabled, the IPC module is excluded from the build entirely.
 */
#ifndef ACROSCHED_USE_IPC
#define ACROSCHED_USE_IPC           0
#endif /* ACROSCHED_USE_IPC */

/*----------------------------------------------------------------------------*/

/**
 * @def ACROSCHED_TICK_TYPE
 * @brief Underlying integer type for the system tick counter.
 * @details Default is uint16_t (suitable for 8/16-bit platforms such as
 * ATmega and MSP430). Override to uint32_t in the port header for
 * 32-bit platforms (Cortex-M, MSPM0).
 */
#ifndef ACROSCHED_TICK_TYPE
#define ACROSCHED_TICK_TYPE         uint16_t
#endif /* ACROSCHED_TICK_TYPE */

/*----------------------------------------------------------------------------*/

/**
 * @def ACROSCHED_ENTER_CRITICAL
 * @brief Enter a critical section (disable interrupts).
 * @details No-op default. Override this macro in the port header
 * (acrosched_port.h) to provide a platform-specific implementation.
 * Example (Cortex-M):
 *   @code #define ACROSCHED_ENTER_CRITICAL()  __disable_irq() @endcode
 * Example (ATmega):
 *   @code #define ACROSCHED_ENTER_CRITICAL()  cli() @endcode
 */
#ifndef ACROSCHED_ENTER_CRITICAL
#define ACROSCHED_ENTER_CRITICAL()  /* no-op */
#endif /* ACROSCHED_ENTER_CRITICAL */

/**
 * @def ACROSCHED_EXIT_CRITICAL
 * @brief Exit a critical section (re-enable interrupts).
 * @details No-op default. Override this macro in the port header
 * (acrosched_port.h) to provide a platform-specific implementation.
 * Example (Cortex-M):
 *   @code #define ACROSCHED_EXIT_CRITICAL()  __enable_irq() @endcode
 * Example (ATmega):
 *   @code #define ACROSCHED_EXIT_CRITICAL()  sei() @endcode
 */
#ifndef ACROSCHED_EXIT_CRITICAL
#define ACROSCHED_EXIT_CRITICAL()   /* no-op */
#endif /* ACROSCHED_EXIT_CRITICAL */

/******************************************************************************/
#endif //! ACROSCHED_CONFIG_H_
