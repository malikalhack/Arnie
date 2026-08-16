/**
 * @file    acrosched.h
 * @version 2.2.0
 * @authors Anton Chernov
 * @date    2026-05-14
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef ACROSCHED_H_
#define ACROSCHED_H_

/******************************** Included files ******************************/
#include <stddef.h>
#include <stdint.h>
#include "acrosched_config.h"

/********************************* Definitions ********************************/

/**
 * @def ACROSCHED_INVALID_ID
 * @brief Sentinel value returned when a task ID cannot be resolved.
 */
#define ACROSCHED_INVALID_ID        (0U)

/*----------------------------------------------------------------------------*/

/**
 * @enum EAcroStatus
 * @brief Return status codes for the AcroSched public API.
 */
typedef enum EAcroStatus {
    eAcroOk           = 0, /**< Operation completed successfully.           */
    eAcroError        = 1, /**< Unspecified internal error.                 */
    eAcroInvalidParam = 2, /**< One or more parameters are invalid (NULL,   */
                           /**< out-of-range, etc.).                        */
    eAcroFull         = 3, /**< A resource pool is full; no free slot is    */
                           /**< available (task pool, timer pool, mailbox). */
    eAcroTimeout      = 4  /**< A blocking wait expired before the resource */
                           /**< became available (preemptive kernel only).  */
} EAcroStatus_t;

/*----------------------------------------------------------------------------*/

/**
 * @enum EAcroMode
 * @brief Task execution modes.
 */
typedef enum EAcroMode {
    eRealtime        = 0, /**< Called on every dispatcher iteration.        */
    eOnetime         = 1, /**< Called once after a delay, then removed.     */
    ePeriodic        = 2, /**< Called repeatedly at a fixed period.         */
    eLimitedLifetime = 3, /**< Called until a timeout expires, then removed.*/
    eStandby         = 4, /**< Suspended for a given time, then resumed.    */
    eIdle            = 5, /**< Registered but never called.                 */
    eMaxMode         = 6  /**< Sentinel - do not use as a mode value.       */
} EAcroMode_t;

/*----------------------------------------------------------------------------*/

/** @brief Generic parameter pointer passed to task functions. */
typedef void * AcroParam_t;

/** @brief Task function signature. */
typedef void AcroTask_t(AcroParam_t);

/**
 * @brief System tick counter type.
 * @details Width is controlled by ACROSCHED_TICK_TYPE in acrosched_config.h.
 * Default: uint16_t (ATmega, MSP430). Override to uint32_t in the port
 * header for 32-bit platforms (Cortex-M, MSPM0).
 */
typedef ACROSCHED_TICK_TYPE AcroTick_t;

#if (ACROSCHED_USE_STATISTICS == 1)
/**
 * @brief Dispatch counter type for the optional statistics module.
 * @details Counts per-task function invocations and total dispatcher
 * iterations. Defined only when ACROSCHED_USE_STATISTICS = 1.
 */
typedef uint32_t AcroDispatchCount_t;
#endif /* ACROSCHED_USE_STATISTICS */

/********************* Application Programming Interface *********************/

/**
 * @brief Initializes the scheduler with a system tick source.
 * @param[in] pTimeSource - pointer to the volatile system tick counter.
 * @returns Operation status.
 * @retval eAcroOk           Initialisation successful.
 * @retval eAcroInvalidParam pTimeSource is NULL.
 */
EAcroStatus_t acroInit(volatile AcroTick_t const *pTimeSource);

/**
 * @brief Resets the scheduler state.
 * @details Clears the task pool and re-initialises the ID generator.
 */
void acroReset(void);

/**
 * @brief Runs the cooperative dispatcher loop.
 * @details Dispatches tasks by mode in an infinite loop. Returns only
 * if the scheduler was not initialised prior to this call.
 * @returns Operation status.
 * @retval eAcroError  Scheduler was not initialised (acroInit() not called).
 */
EAcroStatus_t acroRun(void);

/**
 * @brief Adds a task to the scheduler pool.
 * @param[in] dscr     - user-defined task descriptor.
 * @param[in] pFunc    - pointer to the task function (must not be NULL).
 * @param[in] pParams  - argument passed to the task function (may be NULL).
 * @param[in] mode     - initial execution mode (@ref EAcroMode_t).
 * @param[in] runTime  - period or timeout in system ticks.
 * @param[in] priority - dispatch priority; higher value means earlier
 *                       dispatch. Equal-priority tasks are dispatched in
 *                       registration order. Use 0 for round-robin behaviour.
 * @returns Operation status.
 * @acrostdreturns
 */
EAcroStatus_t acroAddTask(
    uint8_t      dscr,
    AcroTask_t  *pFunc,
    AcroParam_t  pParams,
    uint8_t      mode,
    AcroTick_t   runTime,
    uint8_t      priority
);

/**
 * @brief Adds a task to the pool with an initial start delay.
 * @param[in] dscr     - user-defined task descriptor.
 * @param[in] pFunc    - pointer to the task function (must not be NULL).
 * @param[in] pParams  - argument passed to the task function (may be NULL).
 * @param[in] mode     - execution mode after the delay expires.
 * @param[in] runTime  - period or timeout in system ticks.
 * @param[in] delay    - initial standby duration in system ticks.
 * @param[in] priority - dispatch priority; higher value means earlier
 *                       dispatch. Use 0 for round-robin behaviour.
 * @returns Operation status.
 * @acrostdreturns
 */
EAcroStatus_t acroAddTaskWithDelay(
    uint8_t      dscr,
    AcroTask_t  *pFunc,
    AcroParam_t  pParams,
    uint8_t      mode,
    AcroTick_t   runTime,
    AcroTick_t   delay,
    uint8_t      priority
);

/**
 * @brief Removes a task from the pool by its descriptor.
 * @param[in] dscr - user-defined task descriptor.
 * @returns Operation status.
 * @retval eAcroOk           Task removed successfully.
 * @retval eAcroInvalidParam No task with the given descriptor was found.
 */
EAcroStatus_t acroKillTask(uint8_t dscr);

/**
 * @brief Removes a task from the pool by its ID.
 * @param[in] id - task ID obtained from acroGetTaskId().
 * @returns Operation status.
 * @retval eAcroOk           Task removed successfully.
 * @retval eAcroInvalidParam No task with the given id was found.
 */
EAcroStatus_t acroKillTaskById(uint8_t id);

/**
 * @brief Suspends a task for a given time, then resumes its previous mode.
 * @param[in] id          - task ID.
 * @param[in] standbyTime - suspension duration in system ticks.
 * @returns Operation status.
 * @retval eAcroOk           Task suspended successfully.
 * @retval eAcroInvalidParam No task with the given id was found.
 */
EAcroStatus_t acroPutTaskOnStandby(uint8_t id, AcroTick_t standbyTime);

/**
 * @brief Changes the execution mode and timing of a task.
 * @param[in] id      - task ID.
 * @param[in] mode    - new execution mode (@ref EAcroMode_t).
 * @param[in] runTime - new period or timeout in system ticks.
 * @returns Operation status.
 * @retval eAcroOk           Mode changed successfully.
 * @retval eAcroInvalidParam id not found or mode is out of range.
 */
EAcroStatus_t acroSetTaskMode(
    uint8_t    id,
    uint8_t    mode,
    AcroTick_t runTime
);

/**
 * @brief Replaces the function of an existing task.
 * @param[in] id      - task ID.
 * @param[in] pFunc   - pointer to the new task function (must not be NULL).
 * @param[in] mode    - execution mode for the new function.
 * @param[in] runTime - period or timeout in system ticks.
 * @returns Operation status.
 * @retval eAcroOk           Task function replaced successfully.
 * @retval eAcroInvalidParam id not found, pFunc is NULL, or mode is out of range.
 */
EAcroStatus_t acroReplaceTask(
    uint8_t     id,
    AcroTask_t *pFunc,
    uint8_t     mode,
    AcroTick_t  runTime
);

/**
 * @brief Returns the ID of the task with the given descriptor.
 * @param[in] dscr - user-defined task descriptor.
 * @returns Task ID, or ACROSCHED_INVALID_ID (0) if no task with that descriptor
 * exists.
 */
uint8_t acroGetTaskId(uint8_t dscr);

/*----------------------------------------------------------------------------*/

#if (ACROSCHED_USE_WATCHDOG == 1)

/**
 * @brief Watchdog refresh hook called from the dispatcher loop.
 * @details Weak no-op default. Override this function in the port or
 * application layer to kick the hardware watchdog timer.
 * Enable via ACROSCHED_USE_WATCHDOG = 1 in acrosched_config.h.
 */
void acroWatchdogRefresh(void);

#endif /* ACROSCHED_USE_WATCHDOG */

/*----------------------------------------------------------------------------*/

#if (ACROSCHED_USE_IDLE_HOOK == 1)

/**
 * @brief Idle hook called from the dispatcher loop when no task ran.
 * @details Weak no-op default. The cooperative dispatcher calls this hook once
 * per iteration whenever no task function was invoked during that iteration,
 * immediately before ACROSCHED_WAIT_FOR_EVENT(). Override it in the port or
 * application layer to prepare for or enter a low-power state.
 * Enable via ACROSCHED_USE_IDLE_HOOK = 1 in acrosched_config.h.
 */
void acroIdleHook(void);

/**
 * @brief Returns the number of ticks until the soonest scheduled task wake-up.
 * @details Read-only helper for tickless / deep-sleep ports: it scans the task
 * pool and reports how many ticks remain until the earliest timed task is due
 * (onetime, periodic or standby). Tasks dispatched on every iteration
 * (realtime, limited-lifetime) make the result 0 (an event is due now); idle
 * tasks contribute no event. When no task schedules a wake-up the function
 * returns the maximum AcroTick_t value, meaning the caller may sleep until an
 * external interrupt. The scan runs inside a critical section so the snapshot
 * is consistent. Combine with acroTimersTimeToNext() (software-timer module)
 * to obtain an overall sleep budget. Enable via ACROSCHED_USE_IDLE_HOOK = 1.
 * @returns Ticks until the next scheduled task event; 0 if one is already due,
 * or the maximum AcroTick_t value if no task schedules a wake-up.
 */
AcroTick_t acroTimeToNextEvent(void);

#endif /* ACROSCHED_USE_IDLE_HOOK */

/*----------------------------------------------------------------------------*/

#if (ACROSCHED_USE_HOOKS == 1)

/**
 * @brief Diagnostic hook called when a pool operation is rejected with an error.
 * @details Weak no-op default. The pool layer calls this hook whenever a public
 * API operation fails for a reason other than pool overflow (for example an
 * invalid parameter). The hook runs after the critical section has been left,
 * so interrupts are in their normal state when it executes. Override this
 * function in the application layer to log or react to the failure. Pool
 * overflow is reported separately through acroPoolOverflowHook().
 * Enable via ACROSCHED_USE_HOOKS = 1 in acrosched_config.h.
 * @param[in] status - the non-success status returned by the operation.
 */
void acroErrorHook(EAcroStatus_t status);

/**
 * @brief Diagnostic hook called when a task cannot be added because the pool
 *        is full.
 * @details Weak no-op default. The pool layer calls this hook whenever
 * acroAddTask() or acroAddTaskWithDelay() is rejected with eAcroFull. The hook
 * runs after the critical section has been left. Override this function in the
 * application layer to log the overflow or take corrective action.
 * Enable via ACROSCHED_USE_HOOKS = 1 in acrosched_config.h.
 */
void acroPoolOverflowHook(void);

#endif /* ACROSCHED_USE_HOOKS */

/*----------------------------------------------------------------------------*/

#if (ACROSCHED_USE_INTROSPECTION == 1)

/**
 * @struct SAcroTaskInfo
 * @brief Read-only snapshot of a task's state returned by acroGetTaskInfo().
 */
typedef struct SAcroTaskInfo {
    uint8_t    dscr;        /**< User-defined task descriptor.              */
    uint8_t    mode;        /**< Current execution mode (@ref EAcroMode_t). */
    uint8_t    priority;    /**< Dispatch priority (higher = earlier).      */
    AcroTick_t ticksToNext; /**< Ticks until the next run (0 if due now).   */
#if (ACROSCHED_USE_STATISTICS == 1)
    AcroDispatchCount_t dispatchCount; /**< Times the task function ran.    */
#endif /* ACROSCHED_USE_STATISTICS */
} SAcroTaskInfo_t;

/**
 * @brief Returns the number of tasks currently present in the pool.
 * @details Read-only introspection helper. Enable via
 * ACROSCHED_USE_INTROSPECTION = 1 in acrosched_config.h.
 * @returns Current task count (0 to ACROSCHED_MAX_TASKS).
 */
uint8_t acroGetTaskCount(void);

/**
 * @brief Fills a caller-provided structure with a read-only snapshot of a task.
 * @details Strictly read-only: pool state is never modified. The snapshot is
 * taken inside a critical section so the four fields are mutually consistent.
 * Enable via ACROSCHED_USE_INTROSPECTION = 1 in acrosched_config.h.
 * @param[in]  id    - task ID obtained from acroGetTaskId().
 * @param[out] pInfo - pointer to the structure to fill (must not be NULL).
 * @returns Operation status.
 * @retval eAcroOk           Snapshot written successfully.
 * @retval eAcroInvalidParam pInfo is NULL, or no task with the given id exists.
 */
EAcroStatus_t acroGetTaskInfo(uint8_t id, SAcroTaskInfo_t *pInfo);

#endif /* ACROSCHED_USE_INTROSPECTION */

/*----------------------------------------------------------------------------*/

#if (ACROSCHED_USE_STATISTICS == 1)

/**
 * @brief Returns the per-task dispatch counter (function invocation count).
 * @details Reports how many times the given task's function has actually been
 * invoked by the dispatcher since the task was added. The counter is read
 * inside a critical section so it is consistent with concurrent dispatching.
 * Enable via ACROSCHED_USE_STATISTICS = 1 in acrosched_config.h.
 * @param[in]  id     - task ID obtained from acroGetTaskId().
 * @param[out] pCount - pointer to the variable to fill (must not be NULL).
 * @returns Operation status.
 * @retval eAcroOk           Counter written successfully.
 * @retval eAcroInvalidParam pCount is NULL, or no task with the given id exists.
 */
EAcroStatus_t acroGetTaskDispatchCount(uint8_t id, AcroDispatchCount_t *pCount);

/**
 * @brief Returns the total number of dispatcher iterations executed so far.
 * @details Counts how many times the cooperative dispatcher has traversed the
 * task pool since acroReset(). Useful as a profiling denominator together with
 * the per-task counters. Enable via ACROSCHED_USE_STATISTICS = 1 in
 * acrosched_config.h.
 * @returns The dispatcher iteration count.
 */
AcroDispatchCount_t acroGetIterationCount(void);

#endif /* ACROSCHED_USE_STATISTICS */

/******************************************************************************/
#endif //! ACROSCHED_H_
