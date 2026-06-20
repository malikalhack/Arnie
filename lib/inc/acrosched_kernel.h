/**
 * @file    acrosched_kernel.h
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    2026-05-14
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef ACROSCHED_KERNEL_H_
#define ACROSCHED_KERNEL_H_

/*
 * INTERNAL INTERFACE - NOT PART OF THE PUBLIC API.
 * Include this header only in acrosched.c and acrosched_kernel_*.c.
 * Application code must include acrosched.h instead.
 *
 * Extension point (REQ-004):
 *   To swap the scheduling algorithm, replace the kernel source file:
 *     - src/acrosched_kernel_coop.c    cooperative dispatcher (current)
 *     - src/acrosched_kernel_preempt.c preemptive dispatcher  (future)
 *   No changes to acrosched.c or the public acrosched.h are required.
 */

/******************************** Included files ******************************/
#include "acrosched.h"

/********************************* Definitions ********************************/

/**
 * @brief Internal process control block.
 * @details Represents a single slot in the static task pool. Owned and
 *          managed by the pool layer (acrosched.c). The kernel reads entries
 *          to dispatch tasks; it must never write to this structure directly.
 */
typedef struct SAcroProcess {
    AcroTask_t  *pFunc;       /* Task function pointer                      */
    AcroParam_t  pParams;     /* Task function argument                     */
    AcroTick_t   curTime;     /* Tick count at last dispatch / mode change  */
    AcroTick_t   nextRunTime; /* Period, timeout, or standby duration       */
    AcroTick_t   prevRunTime; /* Saved run time while in standby            */
    uint8_t      id;          /* Assigned task ID (1-based)                 */
    uint8_t      dscr;        /* User-defined descriptor                    */
    uint8_t      mode;        /* Current execution mode                     */
    uint8_t      prevMode;    /* Saved mode while in standby                */
    uint8_t      priority;    /* Dispatch priority (higher value = earlier) */
} SAcroProcess_t;

/*----------------------------------------------------------------------------*/

/** @brief Mode handler function pointer type. */
typedef void (*AcroModeHandler_t)(uint8_t);

/********************* Application Programming Interface *********************/

/* Pool layer exports - defined in acrosched.c, consumed by the kernel.      */

/**
 * @brief Pointer to the application-provided system tick counter.
 * @details Set by acroInit(). NULL until the scheduler is initialised.
 *          The kernel checks this value before entering the dispatcher loop.
 */
extern volatile AcroTick_t const *p_systime;

/**
 * @brief Number of tasks currently present in the pool.
 * @details Written exclusively by the pool layer; read by the kernel on
 *          every dispatcher iteration.
 */
extern volatile uint8_t process_count;

/**
 * @brief Static task pool array.
 * @details Allocated in acrosched.c. The kernel reads pool entries to
 *          dispatch tasks; it must never modify this array directly.
 */
extern SAcroProcess_t process_pool[];

/*----------------------------------------------------------------------------*/

/* Mode handlers - implemented in acrosched.c, called by the kernel.         */
/* Each handler receives the pool index of the task to dispatch and applies   */
/* the mode-specific scheduling logic.                                        */

/** @brief Registered but never dispatched; keeps the slot occupied (eIdle). */
void idle(uint8_t index);

/** @brief Dispatched on every iteration of the dispatcher loop (eRealtime). */
void realtime(uint8_t index);

/** @brief Dispatched once after the delay expires; task is then removed (eOnetime). */
void onetime(uint8_t index);

/** @brief Dispatched at a fixed period; rearms itself after each call (ePeriodic). */
void periodic(uint8_t index);

/** @brief Suspends the task for the configured duration, then restores its previous mode (eStandby). */
void standby(uint8_t index);

/** @brief Dispatched until the timeout expires, then the task is removed (eLimitedLifetime). */
void limitedLifetime(uint8_t index);

/******************************************************************************/

#endif //! ACROSCHED_KERNEL_H_
