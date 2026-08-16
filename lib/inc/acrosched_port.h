/**
 * @file    acrosched_port.h
 * @version 2.2.0
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
 * @def ACROSCHED_USE_TICK_DOUBLE_READ
 * @brief Disabled: a 32-bit tick read is atomic on Cortex-M, so the
 *        double-read consistency guard is unnecessary.
 */
#define ACROSCHED_USE_TICK_DOUBLE_READ  0

/**
 * @def ACROSCHED_DECLARE_CRITICAL
 * @brief Local storage for the saved interrupt-enable state (PRIMASK).
 */
#define ACROSCHED_DECLARE_CRITICAL()  uint32_t _acroPriMask

/**
 * @def ACROSCHED_ENTER_CRITICAL
 * @brief Save the current PRIMASK and disable all interrupts (CMSIS intrinsic).
 * @details Nesting-safe: ACROSCHED_EXIT_CRITICAL() restores the saved PRIMASK,
 * so a nested entry made while interrupts are already disabled does not
 * re-enable them on exit.
 */
#define ACROSCHED_ENTER_CRITICAL()      \
    do {                                \
        _acroPriMask = __get_PRIMASK(); \
        __disable_irq();                \
    } while (0)

/**
 * @def ACROSCHED_EXIT_CRITICAL
 * @brief Restore the PRIMASK saved by ACROSCHED_ENTER_CRITICAL().
 */
#define ACROSCHED_EXIT_CRITICAL()   __set_PRIMASK(_acroPriMask)

/**
 * @def ACROSCHED_WAIT_FOR_EVENT
 * @brief Enter sleep until the next interrupt (Cortex-M Wait-For-Interrupt).
 * @details Used by the optional idle-hook module (ACROSCHED_USE_IDLE_HOOK) to
 * stop the core when no task ran during a dispatcher iteration. The system
 * tick interrupt (or any other enabled interrupt) wakes the core. For deeper
 * tickless sleep, replace this with a sequence that reprograms a low-power
 * wake timer using acroTimeToNextEvent() before entering a deep-sleep mode.
 */
#define ACROSCHED_WAIT_FOR_EVENT()  __WFI()

/******************************************************************************/
#endif //! ACROSCHED_PORT_H_
