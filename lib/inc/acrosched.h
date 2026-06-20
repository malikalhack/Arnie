/**
 * @file    acrosched.h
 * @version 1.0.0
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
    eAcroFull         = 3  /**< Task pool is full; no free slot available.  */
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

/******************************************************************************/
#endif //! ACROSCHED_H_
