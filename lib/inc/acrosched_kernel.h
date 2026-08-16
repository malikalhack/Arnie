/**
 * @file    acrosched_kernel.h
 * @version 2.2.0
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
 * @def ACROSCHED_LOCAL
 * @brief Linkage qualifier for core helpers shared with the preemptive kernel.
 * @details Internal detail - NOT a user configuration knob; kept out of
 *          acrosched_config.h so an application never overrides it. In a
 *          cooperative build (ACROSCHED_USE_PREEMPT == 0) it expands to static,
 *          so the core helpers in acrosched.c keep the internal linkage they
 *          had in the 1.x releases and the cooperative translation unit stays
 *          preprocessor-identical. In a preemptive build it expands to nothing,
 *          giving getCurTime() external linkage so acrosched_kernel_preempt.c
 *          can reuse the single, port-tunable tick reader (DRY) instead of
 *          duplicating the double-read guard.
 */
#if (ACROSCHED_USE_PREEMPT == 1)
#define ACROSCHED_LOCAL
#else
#define ACROSCHED_LOCAL             static
#endif /* ACROSCHED_USE_PREEMPT */

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
#if (ACROSCHED_USE_STATISTICS == 1)
    AcroDispatchCount_t dispatchCount; /* Times the task function ran        */
#endif /* ACROSCHED_USE_STATISTICS */
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

#if (ACROSCHED_USE_STATISTICS == 1)
/**
 * @brief Total number of dispatcher iterations executed.
 * @details Defined in acrosched.c, reset by acroReset(). The kernel increments
 *          it once per traversal of the task pool; read back through
 *          acroGetIterationCount(). Present only when the statistics module is
 *          enabled (ACROSCHED_USE_STATISTICS = 1).
 */
extern volatile AcroDispatchCount_t acro_iteration_count;
#endif /* ACROSCHED_USE_STATISTICS */

#if (ACROSCHED_USE_IDLE_HOOK == 1)
/**
 * @brief Flag indicating that at least one task ran in the current iteration.
 * @details Defined in acrosched.c. The kernel clears it at the start of every
 *          dispatcher iteration; each mode handler sets it when it invokes a
 *          task function. After traversing the pool the kernel runs the idle
 *          hook if the flag is still clear. Present only when the idle-hook
 *          module is enabled (ACROSCHED_USE_IDLE_HOOK = 1).
 */
extern volatile uint8_t acro_task_ran;
#endif /* ACROSCHED_USE_IDLE_HOOK */

/*----------------------------------------------------------------------------*/

#if (ACROSCHED_USE_PREEMPT == 0)

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

#endif /* ACROSCHED_USE_PREEMPT */

/******************************************************************************/

#if (ACROSCHED_USE_PREEMPT == 1)

/*
 * PREEMPTIVE KERNEL - INTERNAL INTERFACE (REQ-021).
 *
 * These declarations are shared between the preemptive scheduler
 * (acrosched_kernel_preempt.c), the blocking IPC primitives
 * (acrosched_preempt.c) and the per-architecture context-switch port
 * (port/<arch>/acrosched_port_preempt.*). They are compiled only when the
 * preemptive kernel is selected (ACROSCHED_USE_PREEMPT == 1) and carry no
 * meaning for the cooperative kernel.
 *
 * Unlike the cooperative kernel, the preemptive kernel legitimately re-arms a
 * task's timing fields (curTime) on an activation boundary: the six acrosched.c
 * mode handlers are excluded from this build (they are compiled only when
 * ACROSCHED_USE_PREEMPT == 0), so their run-to-completion bookkeeping is
 * reproduced here instead, reading the same mode field.
 */

/**
 * @enum EAcroPState
 * @brief Preemptive task lifecycle states stored in SAcroTcb_t::pstate.
 */
typedef enum EAcroPState {
    eAcroPsNew     = 0, /**< Registered; private stack not yet initialised.  */
    eAcroPsActive  = 1, /**< Runnable or currently running on its stack.     */
    eAcroPsBlocked = 2  /**< Suspended on a sync object or a timed deadline. */
} EAcroPState_t;

/**
 * @def ACRO_TCB_TIMED
 * @brief Flag bit: the wakeTime deadline is armed for a timed wait.
 */
#define ACRO_TCB_TIMED              0x01U

/**
 * @brief Preemptive task control block (one per task, keyed by task ID).
 * @details Holds the volatile scheduling state that the preemptive kernel
 *          keeps outside the compacting task pool. Indexing by (id - 1) keeps
 *          the block stable across pool compaction (acroAddTask /
 *          acroKillTaskById shift pool entries but never touch this array).
 * @note stackPtr MUST remain the first member: the context-switch port loads
 *       it with a plain word read at offset 0 from the current TCB pointer.
 */
typedef struct SAcroTcb {
    void       *stackPtr;   /* Saved stack pointer (first member - see note) */
    void       *blockedOn;  /* Sync object waited on; NULL = timed / none    */
    AcroTick_t  wakeTime;   /* Absolute deadline when ACRO_TCB_TIMED is set  */
    uint8_t     pstate;     /* Task state (EAcroPState_t)                    */
    uint8_t     flags;      /* Bit flags (ACRO_TCB_*)                        */
    uint8_t     waitResult; /* EAcroStatus_t outcome of the last wait        */
    uint8_t     lastRunSeq; /* Dispatch stamp for fair equal-priority RR     */
#if (ACROSCHED_USE_PRIO_INHERIT == 1)
    uint8_t     inheritedPrio; /* Donated priority (0 = none); mutex inherit  */
#endif /* ACROSCHED_USE_PRIO_INHERIT */
} SAcroTcb_t;

/*----------------------------------------------------------------------------*/

/* Scheduler state - defined in acrosched_kernel_preempt.c.                   */

/** @brief Per-task control blocks, indexed by (task id - 1). */
extern SAcroTcb_t acro_tcb[ACROSCHED_MAX_TASKS];

/** @brief Control block of the built-in idle task (runs when nothing else). */
extern SAcroTcb_t acro_idle_tcb;

/** @brief Pointer to the control block of the currently running task. */
extern SAcroTcb_t * volatile acro_current;

/** @brief Non-zero once acroRun() has started the first task. */
extern volatile uint8_t acro_sched_started;

#if (ACROSCHED_USE_PRIO_INHERIT == 1)
/** @brief Count of tasks currently holding a donated (inherited) priority. */
extern uint8_t acro_inherit_active;
#endif /* ACROSCHED_USE_PRIO_INHERIT */

/*----------------------------------------------------------------------------*/

/* Kernel helpers - defined in acrosched_kernel_preempt.c, used by the IPC   */
/* module (acrosched_preempt.c) and the context-switch port.                 */

/**
 * @brief Core system-tick reader shared from acrosched.c.
 * @details Exported with external linkage only in a preemptive build (via the
 *          ACROSCHED_LOCAL macro), so the preemptive kernel and IPC primitives
 *          reuse the single tick double-read consistency guard instead of
 *          duplicating it (DRY).
 */
AcroTick_t getCurTime(void);

/** @brief Resolve a task ID to its current pool index, or ACROSCHED_MAX_TASKS. */
uint8_t acroFindIndexById(uint8_t id);

/**
 * @brief Block the calling task on a sync object (or a timed deadline) and
 *        yield the CPU; returns the wait outcome once the task resumes.
 * @param obj     - sync object to wait on, or NULL for a pure timed wait.
 * @param timeout - deadline in ticks, or ACROSCHED_WAIT_FOREVER to wait
 *                  indefinitely.
 * @returns The EAcroStatus_t stored in the task's TCB when it was woken
 *          (eAcroOk on a successful hand-off, eAcroTimeout on expiry).
 */
uint8_t acroBlockCurrent(void *obj, AcroTick_t timeout);

/**
 * @brief Wake the highest-priority task blocked on a sync object.
 * @param obj - sync object whose waiters should be searched.
 * @returns 1 if a waiter was woken (its wait completes with eAcroOk), else 0.
 */
uint8_t acroWakeOne(void *obj);

/**
 * @brief Reset a task's control block to the pristine "new" state.
 * @details Called by acroAddTask() (pool layer) for the id it assigns, so a
 *          freshly registered task - including one that reuses an id freed by
 *          a self-removing eOnetime / eLimitedLifetime task - starts with an
 *          uninitialised private stack (eAcroPsNew) instead of inheriting the
 *          parked state of the previous owner.
 * @param id - task id (1-based) whose control block to reset.
 */
void acroTcbReset(uint8_t id);

/**
 * @brief Task id of the currently running task.
 * @details Returns the id (1-based) of the task the scheduler last dispatched,
 *          or ACRO_IDLE_ID while the built-in idle task runs. Used by the mutex
 *          primitive to record and verify ownership; must be called from task
 *          context, never from an ISR.
 * @returns The current task's id, or ACRO_IDLE_ID for the idle task.
 */
uint8_t acroCurrentTaskId(void);

#if (ACROSCHED_USE_PRIO_INHERIT == 1)
/**
 * @brief Effective scheduling priority of a task: max(base, inherited).
 * @details Returns the greater of the task's static pool priority and any
 *          priority donated to it through mutex priority inheritance. Used by
 *          the scheduler to pick the next task and by the mutex primitive to
 *          size the donation. Returns 0 for an unknown id.
 * @param id - task id (1-based).
 * @returns The task's effective scheduling priority.
 */
uint8_t acroTaskEffPrio(uint8_t id);
#endif /* ACROSCHED_USE_PRIO_INHERIT */

/*----------------------------------------------------------------------------*/

/* Context-switch port - defined in port/<arch>/acrosched_port_preempt.*.    */

/** @brief One-time port setup: exception priorities for the switch. */
void acroPortInitPreempt(void);

/**
 * @brief Build an initial context frame on a fresh task stack.
 * @param stackTop - highest address of the task's stack region (exclusive).
 * @param entry    - task trampoline entry point.
 * @param arg      - value delivered as the trampoline's first argument.
 * @returns The initial saved stack pointer to store in the task's TCB.
 */
void *acroPortInitStack(void *stackTop, void (*entry)(void *), void *arg);

/** @brief Start the first task selected in acro_current; does not return. */
void acroPortStartFirstTask(void);

/** @brief Request a context switch (pend the switch exception). */
void acroPortYield(void);

/**
 * @brief Save the outgoing stack pointer, select the next task and return its
 *        stack pointer. Called from the context-switch exception.
 * @param sp - stack pointer of the outgoing task.
 * @returns Stack pointer of the task selected to run next.
 */
void *acroSwitchContext(void *sp);

#endif /* ACROSCHED_USE_PREEMPT */

/******************************************************************************/

#endif //! ACROSCHED_KERNEL_H_
