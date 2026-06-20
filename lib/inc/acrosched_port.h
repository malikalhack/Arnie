/**
 * @file    acrosched_port.h
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    2026-05-14
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef ACROSCHED_PORT_H_
#define ACROSCHED_PORT_H_

/******************************** Included files ******************************/
#include "cmsis_compiler.h"

/********************************* Definitions ********************************/

/**
 * @def ACROSCHED_TICK_TYPE
 * @brief Override tick type to 32-bit for Cortex-M targets.
 */
#define ACROSCHED_TICK_TYPE         uint32_t

/**
 * @def ACROSCHED_ENTER_CRITICAL
 * @brief Disable all interrupts (Cortex-M CMSIS intrinsic).
 */
#define ACROSCHED_ENTER_CRITICAL()  __disable_irq()

/**
 * @def ACROSCHED_EXIT_CRITICAL
 * @brief Re-enable interrupts (Cortex-M CMSIS intrinsic).
 */
#define ACROSCHED_EXIT_CRITICAL()   __enable_irq()

/******************************************************************************/
#endif //! ACROSCHED_PORT_H_
