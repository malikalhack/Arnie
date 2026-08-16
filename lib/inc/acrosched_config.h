/**
 * @file    acrosched_config.h
 * @version 2.2.0
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
#define ACROSCHED_VERSION_MAJOR     2

/**
 * @def ACROSCHED_VERSION_MINOR
 * @brief Minor version number of AcroSched (backwards-compatible additions).
 */
#define ACROSCHED_VERSION_MINOR     2

/**
 * @def ACROSCHED_VERSION_PATCH
 * @brief Patch version number of AcroSched (backwards-compatible bug fixes).
 */
#define ACROSCHED_VERSION_PATCH     0

/**
 * @def ACROSCHED_VERSION_STRING
 * @brief AcroSched version as a printable "MAJOR.MINOR.PATCH" string literal.
 * @details Assembled at compile time from the numeric version macros, so it
 * costs no RAM - suitable even for the most memory-constrained targets.
 */
#define ACROSCHED_VERSION_STR_(x)   #x
#define ACROSCHED_VERSION_XSTR_(x)  ACROSCHED_VERSION_STR_(x)
#define ACROSCHED_VERSION_STRING \
    ACROSCHED_VERSION_XSTR_(ACROSCHED_VERSION_MAJOR) "." \
    ACROSCHED_VERSION_XSTR_(ACROSCHED_VERSION_MINOR) "." \
    ACROSCHED_VERSION_XSTR_(ACROSCHED_VERSION_PATCH)

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
 * @def ACROSCHED_MAX_TIMERS
 * @brief Maximum number of software timers in the static timer pool.
 * @details Defines the capacity of the statically allocated software-timer
 * array used by the optional software-timer module (ACROSCHED_USE_TIMERS).
 * Each unit costs one SAcroTimer_t. Only relevant when the module is enabled.
 * Override in the port header (acrosched_port.h) or with a compiler -D flag.
 * Must be greater than 0.
 */
#ifndef ACROSCHED_MAX_TIMERS
#define ACROSCHED_MAX_TIMERS        4U
#endif /* ACROSCHED_MAX_TIMERS */

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
#define ACROSCHED_USE_WATCHDOG      1
#endif /* ACROSCHED_USE_WATCHDOG */

/**
 * @def ACROSCHED_USE_IPC
 * @brief Enable (1) or disable (0) the IPC module.
 * @details When disabled, the IPC module is excluded from the build entirely.
 */
#ifndef ACROSCHED_USE_IPC
#define ACROSCHED_USE_IPC           0
#endif /* ACROSCHED_USE_IPC */

/**
 * @def ACROSCHED_USE_HOOKS
 * @brief Enable (1) or disable (0) the optional diagnostic observer hooks.
 * @details When enabled, the pool layer calls weak observer hooks at
 * well-defined failure points: acroPoolOverflowHook() when a task cannot be
 * added because the pool is full, and acroErrorHook() when an operation is
 * rejected for any other reason (for example an invalid parameter). Both have
 * weak no-op defaults; the application provides strong definitions to log or
 * react to the events. The hooks are independent of every other optional
 * module (watchdog, IPC). When disabled, they contribute zero code size.
 */
#ifndef ACROSCHED_USE_HOOKS
#define ACROSCHED_USE_HOOKS         0
#endif /* ACROSCHED_USE_HOOKS */

/**
 * @def ACROSCHED_USE_INTROSPECTION
 * @brief Enable (1) or disable (0) the optional read-only introspection API.
 * @details When enabled, acroGetTaskCount() and acroGetTaskInfo() let the
 * application query scheduler state at runtime for debugging and diagnostics.
 * The API is strictly read-only and never modifies pool state; pool accesses
 * are guarded by ACROSCHED_ENTER_CRITICAL() / ACROSCHED_EXIT_CRITICAL(). When
 * disabled, the module contributes zero code size. It has no compile-time
 * dependency on any other optional module.
 * @note Defining the umbrella ACROSCHED_DEBUG flag enables this module
 * automatically (unless ACROSCHED_USE_INTROSPECTION is set explicitly).
 */
#ifndef ACROSCHED_USE_INTROSPECTION
#ifdef ACROSCHED_DEBUG
#define ACROSCHED_USE_INTROSPECTION 1
#else
#define ACROSCHED_USE_INTROSPECTION 0
#endif /* ACROSCHED_DEBUG */
#endif /* ACROSCHED_USE_INTROSPECTION */

/**
 * @def ACROSCHED_USE_STATISTICS
 * @brief Enable (1) or disable (0) the optional per-task dispatch statistics.
 * @details When enabled, the scheduler counts how many times each task
 * function is actually invoked and how many dispatcher iterations have run.
 * The counters are read back through acroGetTaskDispatchCount() and
 * acroGetIterationCount() (and through acroGetTaskInfo() when the
 * introspection module is also enabled). When disabled, the module contributes
 * zero code size and zero RAM - no counter field is added to the task pool. It
 * has no compile-time dependency on any other optional module.
 * @note Defining the umbrella ACROSCHED_DEBUG flag enables this module
 * automatically (unless ACROSCHED_USE_STATISTICS is set explicitly).
 */
#ifndef ACROSCHED_USE_STATISTICS
#ifdef ACROSCHED_DEBUG
#define ACROSCHED_USE_STATISTICS    1
#else
#define ACROSCHED_USE_STATISTICS    0
#endif /* ACROSCHED_DEBUG */
#endif /* ACROSCHED_USE_STATISTICS */

/**
 * @def ACROSCHED_USE_TIMERS
 * @brief Enable (1) or disable (0) the optional software-timer module.
 * @details When enabled, the module derives up to ACROSCHED_MAX_TIMERS virtual
 * timers from the single hardware tick source. Each timer invokes a user
 * callback on expiry in one-shot or periodic mode. Callbacks are dispatched
 * from the cooperative dispatcher loop (via acroTimersService()), never
 * directly from an ISR. The timers are statically allocated and the module has
 * no compile-time dependency on any other optional module. When disabled, it
 * contributes zero code size.
 */
#ifndef ACROSCHED_USE_TIMERS
#define ACROSCHED_USE_TIMERS        0
#endif /* ACROSCHED_USE_TIMERS */

/**
 * @def ACROSCHED_USE_IDLE_HOOK
 * @brief Enable (1) or disable (0) the optional idle / low-power sleep hook.
 * @details When enabled, the cooperative dispatcher tracks whether any task
 * function ran during an iteration. If none did, it invokes the weak
 * acroIdleHook() and then the port-provided ACROSCHED_WAIT_FOR_EVENT() macro
 * so the core can enter a low-power state until the next interrupt. The module
 * also exposes the read-only helper acroTimeToNextEvent(), which reports the
 * number of ticks until the soonest scheduled task wake-up; a tickless port or
 * application can combine it with acroTimersTimeToNext() (when the software
 * timer module is enabled) to compute a deep-sleep budget. The hook has no
 * compile-time dependency on any other optional module. When disabled, it
 * contributes zero code size and zero RAM - no run-flag is added to the loop.
 */
#ifndef ACROSCHED_USE_IDLE_HOOK
#define ACROSCHED_USE_IDLE_HOOK     0
#endif /* ACROSCHED_USE_IDLE_HOOK */

/*----------------------------------------------------------------------------*/

/**
 * @def ACROSCHED_USE_PREEMPT
 * @brief Select the preemptive kernel (1) instead of the cooperative one (0).
 * @details Opt-in switch for the fixed-priority preemptive kernel introduced in
 * AcroSched 2.0.0. When 0 (the default), the classic cooperative,
 * run-to-completion dispatcher (acrosched_kernel_coop.c) is used and the
 * library behaves exactly as in the 1.x releases. When 1, the application must
 * instead build and link acrosched_kernel_preempt.c together with the
 * per-architecture context-switch port and give every task its own private
 * stack. The public acrosched.h API (acroInit / acroAddTask / acroRun / ...)
 * is unchanged; the execution MODE of a task now selects its readiness policy
 * while a higher-priority ready task may preempt a lower-priority one on its
 * own stack. Selecting the preemptive kernel unlocks the blocking IPC
 * primitives declared in acrosched_preempt.h (ACROSCHED_USE_SEM).
 * @note This is a compile-time selection: exactly one kernel translation unit
 * (coop or preempt) must be linked. Contributes zero code size when 0.
 */
#ifndef ACROSCHED_USE_PREEMPT
#define ACROSCHED_USE_PREEMPT       0
#endif /* ACROSCHED_USE_PREEMPT */

/**
 * @def ACROSCHED_STACK_SIZE
 * @brief Size, in machine words, of each private task stack (preemptive only).
 * @details A single value applies to every task and to the built-in idle task.
 * One word equals sizeof(void *) - 4 bytes on a 32-bit core such as Cortex-M.
 * The default of 64 words (256 bytes on a 32-bit target) suits shallow task
 * bodies; raise it for tasks that use deep call chains, large local buffers or
 * floating-point context. Used only when ACROSCHED_USE_PREEMPT is 1.
 */
#ifndef ACROSCHED_STACK_SIZE
#define ACROSCHED_STACK_SIZE        64U
#endif /* ACROSCHED_STACK_SIZE */

/**
 * @def ACROSCHED_USE_SEM
 * @brief Enable (1) or disable (0) the counting-semaphore IPC primitive.
 * @details When enabled (only meaningful with the preemptive kernel), the
 * blocking semaphore API in acrosched_preempt.h - acroSemWait(), acroSemPost()
 * and acroSemTake() - is compiled in. A task that waits on an empty semaphore
 * is suspended and yields the CPU to the next ready task; acroSemPost() (which
 * is ISR-safe) wakes the highest-priority waiter. When disabled, the primitive
 * contributes zero code size. Requires ACROSCHED_USE_PREEMPT == 1.
 */
#ifndef ACROSCHED_USE_SEM
#define ACROSCHED_USE_SEM           0
#endif /* ACROSCHED_USE_SEM */

/**
 * @def ACROSCHED_USE_MUTEX
 * @brief Enable (1) or disable (0) the mutual-exclusion (mutex) primitive.
 * @details When enabled (only meaningful with the preemptive kernel), the
 * ownership-aware mutex API in acrosched_preempt.h - acroMutexInit(),
 * acroMutexLock(), acroMutexTake() and acroMutexUnlock() - is compiled in. A
 * mutex is a binary semaphore that additionally records its owning task: only
 * the task that locked it may unlock it, and it must never be used from an ISR.
 * When disabled, the primitive contributes zero code size. Built on the
 * counting semaphore, so it requires ACROSCHED_USE_SEM == 1 (which in turn
 * requires ACROSCHED_USE_PREEMPT == 1).
 */
#ifndef ACROSCHED_USE_MUTEX
#define ACROSCHED_USE_MUTEX         0
#endif /* ACROSCHED_USE_MUTEX */

/**
 * @def ACROSCHED_USE_RECURSIVE_MUTEX
 * @brief Enable (1) or disable (0) recursive locking for the mutex.
 * @details When enabled, the task that owns a mutex may lock it again without
 * deadlocking: each re-lock by the owner increments a recursion counter and
 * each unlock decrements it, so the mutex is released to other waiters only
 * when the counter returns to zero. Only the owner may lock recursively or
 * unlock, and the mutex must never be used from an ISR. When disabled, the
 * mutex is strictly non-recursive and this feature contributes zero code size.
 * Built on the mutex, so it requires ACROSCHED_USE_MUTEX == 1.
 */
#ifndef ACROSCHED_USE_RECURSIVE_MUTEX
#define ACROSCHED_USE_RECURSIVE_MUTEX  0
#endif /* ACROSCHED_USE_RECURSIVE_MUTEX */

/**
 * @def ACROSCHED_USE_PRIO_INHERIT
 * @brief Enable (1) or disable (0) priority inheritance for the mutex.
 * @details When enabled, while a higher-priority task waits on a mutex the
 * current owner's scheduling priority is temporarily raised to the waiter's,
 * bounding priority inversion; the owner's base priority is restored when it
 * releases the mutex. Single-level only: transitive (nested) inheritance is
 * out of scope, and holding more than one inherited mutex at a time is not
 * supported. When disabled, the feature contributes zero code size and the
 * scheduler keeps its sorted-pool fast path. Built on the mutex, so it
 * requires ACROSCHED_USE_MUTEX == 1.
 */
#ifndef ACROSCHED_USE_PRIO_INHERIT
#define ACROSCHED_USE_PRIO_INHERIT  0
#endif /* ACROSCHED_USE_PRIO_INHERIT */

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
 * @def ACROSCHED_USE_TICK_DOUBLE_READ
 * @brief Enable (1) or disable (0) the double-read consistency guard for the
 *        system tick.
 * @details getCurTime() reads the system tick twice and retries until two
 * consecutive reads agree, guaranteeing a torn-free snapshot on platforms
 * where reading AcroTick_t is not atomic - that is, where the type is wider
 * than the native machine word (for example uint16_t on 8-bit AVR, or
 * uint32_t on an 8/16-bit target).
 * When the tick read is naturally atomic (sizeof(AcroTick_t) does not exceed
 * the native word, for example uint32_t on Cortex-M or uint16_t on the 16-bit
 * MSP430), set this to 0 in the port header to perform a single read and save
 * a few cycles per call.
 * Default 1 (safe on every platform).
 */
#ifndef ACROSCHED_USE_TICK_DOUBLE_READ
#define ACROSCHED_USE_TICK_DOUBLE_READ  1
#endif /* ACROSCHED_USE_TICK_DOUBLE_READ */

/*----------------------------------------------------------------------------*/

/**
 * @def ACROSCHED_DECLARE_CRITICAL
 * @brief Declare storage for the saved interrupt-enable state.
 * @details Place this among the local declarations at the top of any function
 * that opens a critical section with ACROSCHED_ENTER_CRITICAL() /
 * ACROSCHED_EXIT_CRITICAL(). The no-op default expands to nothing; the port
 * header (acrosched_port.h) overrides it with a platform-specific local
 * variable that holds the saved interrupt state (for example the prior
 * PRIMASK on Cortex-M, SREG on ATmega, or SR on MSP430).
 *
 * @note Usage rule (the saved-state variable is per-function, on the stack):
 *       declare it exactly once per function and pair it with exactly one
 *       ACROSCHED_ENTER_CRITICAL() / ACROSCHED_EXIT_CRITICAL() region.
 *       - Sequential (non-overlapping) regions in one function are fine: the
 *         variable is simply reused.
 *       - Do NOT nest two ENTER/EXIT pairs that share the same declared
 *         variable in a single scope: the inner ENTER overwrites the saved
 *         state and the outer EXIT then restores the wrong value (interrupts
 *         may stay disabled). The compiler already rejects a second
 *         ACROSCHED_DECLARE_CRITICAL() in the same scope (redefinition).
 *       - If a region must genuinely be nested, give the inner region its own
 *         scope (a nested { } block with its own ACROSCHED_DECLARE_CRITICAL())
 *         or, preferably, use the ACROSCHED_CRITICAL() wrapper below, which
 *         creates that scope for you. Nesting across function calls is always
 *         safe because each function owns its variable.
 */
#ifndef ACROSCHED_DECLARE_CRITICAL
#define ACROSCHED_DECLARE_CRITICAL()  /* no-op */
#endif /* ACROSCHED_DECLARE_CRITICAL */

/**
 * @def ACROSCHED_ENTER_CRITICAL
 * @brief Enter a critical section: save the current interrupt-enable state,
 *        then disable interrupts.
 * @details No-op default. Override this macro in the port header
 * (acrosched_port.h) to provide a platform-specific, nesting-safe
 * implementation that saves the prior state into the variable declared by
 * ACROSCHED_DECLARE_CRITICAL(). The matching ACROSCHED_EXIT_CRITICAL()
 * restores that state, so nested entry/exit pairs leave interrupts disabled
 * until the outermost exit and never re-enable interrupts that were already
 * disabled on entry.
 * Example (Cortex-M):
 *   @code _acroPriMask = __get_PRIMASK(); __disable_irq(); @endcode
 * Example (ATmega):
 *   @code _acroSreg = SREG; cli(); @endcode
 */
#ifndef ACROSCHED_ENTER_CRITICAL
#define ACROSCHED_ENTER_CRITICAL()  /* no-op */
#endif /* ACROSCHED_ENTER_CRITICAL */

/**
 * @def ACROSCHED_EXIT_CRITICAL
 * @brief Exit a critical section: restore the interrupt-enable state saved by
 *        ACROSCHED_ENTER_CRITICAL().
 * @details No-op default. Override this macro in the port header
 * (acrosched_port.h) to provide a platform-specific implementation that
 * restores the state held by ACROSCHED_DECLARE_CRITICAL() rather than
 * unconditionally re-enabling interrupts.
 * Example (Cortex-M):
 *   @code __set_PRIMASK(_acroPriMask) @endcode
 * Example (ATmega):
 *   @code SREG = _acroSreg @endcode
 */
#ifndef ACROSCHED_EXIT_CRITICAL
#define ACROSCHED_EXIT_CRITICAL()   /* no-op */
#endif /* ACROSCHED_EXIT_CRITICAL */

/**
 * @def ACROSCHED_CRITICAL
 * @brief Execute a block of statements inside a nesting-safe critical section.
 * @details Optional convenience wrapper that bundles ACROSCHED_DECLARE_CRITICAL(),
 * ACROSCHED_ENTER_CRITICAL() and ACROSCHED_EXIT_CRITICAL() into one
 * self-contained scope. Because the saved-state variable is declared inside
 * the wrapper's own block, every invocation gets its own storage; this makes
 * the wrapper a footgun-safe alternative to the three separate macros:
 *   - the matching EXIT can never be forgotten;
 *   - nesting (directly or through nested blocks) is always correct, since
 *     each level owns a distinct saved-state variable rather than clobbering
 *     a shared one.
 * Prefer this form wherever the protected region is a simple statement block.
 * The three lower-level macros remain available for the rare cases that need
 * the enter and exit points in different scopes (for example a region whose
 * end is decided by control flow).
 *
 * @param statements - one or more C statements to run with interrupts disabled.
 *
 * @warning Do not transfer control out of the block (return, break, continue,
 *          goto): that bypasses ACROSCHED_EXIT_CRITICAL() and leaves the saved
 *          interrupt state unrestored.
 *
 * Example:
 *   @code
 *   ACROSCHED_CRITICAL(
 *       shared_flags |= bit;
 *   );
 *   @endcode
 */
#ifndef ACROSCHED_CRITICAL
#define ACROSCHED_CRITICAL(statements)  \
    do {                                \
        ACROSCHED_DECLARE_CRITICAL();   \
        ACROSCHED_ENTER_CRITICAL();     \
        { statements }                  \
        ACROSCHED_EXIT_CRITICAL();      \
    } while (0)
#endif /* ACROSCHED_CRITICAL */

/*----------------------------------------------------------------------------*/

/**
 * @def ACROSCHED_WAIT_FOR_EVENT
 * @brief Enter a low-power state until the next interrupt (idle-hook module).
 * @details No-op default. When the optional idle-hook module is enabled
 * (ACROSCHED_USE_IDLE_HOOK = 1), the cooperative dispatcher calls this macro
 * after acroIdleHook() on any iteration in which no task function ran, so the
 * core can sleep until woken by an interrupt (for example the system tick).
 * Override this macro in the port header (acrosched_port.h) with the
 * platform's wait-for-event primitive:
 *   @code __WFI();      // Cortex-M  @endcode
 *   @code sleep_cpu();  // ATmega    @endcode
 *   @code LPM0;         // MSP430    @endcode
 * A tickless port may instead read acroTimeToNextEvent() here, reprogram a
 * low-power wake timer and enter a deeper sleep mode, advancing the tick
 * counter by the elapsed time on wake-up.
 * @note Leaving this as a no-op turns the idle branch into a busy-wait; it is
 * the port's responsibility to provide the actual sleep instruction.
 */
#ifndef ACROSCHED_WAIT_FOR_EVENT
#define ACROSCHED_WAIT_FOR_EVENT()  /* no-op */
#endif /* ACROSCHED_WAIT_FOR_EVENT */

/*************************** Configuration checks ****************************/

#if (ACROSCHED_MAX_TASKS < 1)
#error "ACROSCHED_MAX_TASKS must be greater than 0."
#endif /* ACROSCHED_MAX_TASKS < 1 */

#if (ACROSCHED_USE_TIMERS == 1)
#if (ACROSCHED_MAX_TIMERS < 1)
#error "ACROSCHED_MAX_TIMERS must be greater than 0 when ACROSCHED_USE_TIMERS == 1."
#endif /* ACROSCHED_MAX_TIMERS < 1 */
#endif /* ACROSCHED_USE_TIMERS == 1 */

#if (ACROSCHED_USE_PREEMPT == 1)
#if (ACROSCHED_STACK_SIZE < 32U)
#error "ACROSCHED_STACK_SIZE must be at least 32 words for the preemptive kernel."
#endif /* ACROSCHED_STACK_SIZE < 32U */
#endif /* ACROSCHED_USE_PREEMPT == 1 */

#if (ACROSCHED_USE_SEM == 1)
#if (ACROSCHED_USE_PREEMPT != 1)
#error "ACROSCHED_USE_SEM requires the preemptive kernel (ACROSCHED_USE_PREEMPT == 1)."
#endif /* ACROSCHED_USE_PREEMPT != 1 */
#endif /* ACROSCHED_USE_SEM == 1 */

#if (ACROSCHED_USE_MUTEX == 1)
#if (ACROSCHED_USE_SEM != 1)
#error "ACROSCHED_USE_MUTEX requires the counting semaphore (ACROSCHED_USE_SEM == 1)."
#endif /* ACROSCHED_USE_SEM != 1 */
#endif /* ACROSCHED_USE_MUTEX == 1 */

#if (ACROSCHED_USE_RECURSIVE_MUTEX == 1)
#if (ACROSCHED_USE_MUTEX != 1)
#error "ACROSCHED_USE_RECURSIVE_MUTEX requires the mutex (ACROSCHED_USE_MUTEX == 1)."
#endif /* ACROSCHED_USE_MUTEX != 1 */
#endif /* ACROSCHED_USE_RECURSIVE_MUTEX == 1 */

#if (ACROSCHED_USE_PRIO_INHERIT == 1)
#if (ACROSCHED_USE_MUTEX != 1)
#error "ACROSCHED_USE_PRIO_INHERIT requires the mutex (ACROSCHED_USE_MUTEX == 1)."
#endif /* ACROSCHED_USE_MUTEX != 1 */
#endif /* ACROSCHED_USE_PRIO_INHERIT == 1 */

/******************************************************************************/
#endif //! ACROSCHED_CONFIG_H_
