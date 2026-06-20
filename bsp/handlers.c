/**
 * @file    handlers.c
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 *
 * @brief   Cortex-M3 fault handlers for STM32F103.
 *
 * @details Strong definitions of the four ARMv7-M fault handlers
 *          (HardFault, MemManage, BusFault, UsageFault), overriding the weak
 *          aliases in the startup file. Each is a naked trampoline that
 *          selects the active stack (MSP or PSP) from EXC_RETURN and branches
 *          to a common C handler. The C handler captures the stacked
 *          exception frame and all fault-status registers (CFSR / HFSR / DFSR
 *          / AFSR / BFAR / MMFAR) into volatiles for inspection, then parks in
 *          an infinite loop.
 *
 *          The Blue Pill (STM32F103C8T6) board's on-board LED is lit
 *          as the visible fault indicator.
 *
 *          The configurable faults (MemManage / BusFault / UsageFault) must be
 *          enabled in SCB->SHCSR before they can fire; until then they escalate
 *          to HardFault. bspInit() enables them (see bsp.c).
 *
 *          Usage in the debugger:
 *            1. Set a breakpoint on the `while (1)` line inside
 *               Common_Fault_Handler_C.
 *            2. Read the volatile vars (cfsr, hfsr, bfar, mmar, stacked_pc, ...)
 *               and paste the hex values into the ARM Fault Analyzer.
 */

/******************************** Included files ******************************/
#include <stdint.h>
#include "RTE_Components.h"
#include CMSIS_device_header
#include "bsp.h"

/********************************* Definitions ********************************/

/**
 * @def FAULT_TRAMPOLINE
 * @brief Naked trampoline: selects MSP or PSP from EXC_RETURN, then branches
 *        to the common C fault handler with the active SP in r0.
 */
#define FAULT_TRAMPOLINE()                                          \
    __asm volatile(                                                 \
        "tst   lr, #4               \n" /* EXC_RETURN bit 2       */\
        "ite   eq                   \n"                             \
        "mrseq r0, msp              \n" /* bit2==0: MSP was active */\
        "mrsne r0, psp              \n" /* bit2==1: PSP was active */\
        "b     Common_Fault_Handler_C \n"                           \
    )

/***************************** Private prototypes *****************************/

void Common_Fault_Handler_C(uint32_t *fault_args);

/****************************** Private functions *****************************/

/** @fn HardFault_Handler */
__attribute__((naked)) void HardFault_Handler(void)  { FAULT_TRAMPOLINE(); }
/*----------------------------------------------------------------------------*/

/** @fn MemManage_Handler */
__attribute__((naked)) void MemManage_Handler(void)  { FAULT_TRAMPOLINE(); }
/*----------------------------------------------------------------------------*/

/** @fn BusFault_Handler */
__attribute__((naked)) void BusFault_Handler(void)   { FAULT_TRAMPOLINE(); }
/*----------------------------------------------------------------------------*/

/** @fn UsageFault_Handler */
__attribute__((naked)) void UsageFault_Handler(void) { FAULT_TRAMPOLINE(); }

/********************* Application Programming Interface *********************/

/** @fn Common_Fault_Handler_C */
void Common_Fault_Handler_C(uint32_t *fault_args) {
    /* Registers stacked automatically by the CPU: r0-r3, r12, lr, pc, xpsr */
    volatile uint32_t stacked_r0  = fault_args[0];
    volatile uint32_t stacked_r1  = fault_args[1];
    volatile uint32_t stacked_r2  = fault_args[2];
    volatile uint32_t stacked_r3  = fault_args[3];
    volatile uint32_t stacked_r12 = fault_args[4];
    volatile uint32_t stacked_lr  = fault_args[5];
    volatile uint32_t stacked_pc  = fault_args[6]; /* faulting instruction */
    volatile uint32_t stacked_psr = fault_args[7];

    /* Fault status registers (read before they are cleared) */
    volatile uint32_t cfsr = SCB->CFSR;  /* MMFSR/BFSR/UFSR combined           */
    volatile uint32_t hfsr = SCB->HFSR;  /* bit30 FORCED = escalated to HardF. */
    volatile uint32_t dfsr = SCB->DFSR;  /* debug fault status                 */
    volatile uint32_t afsr = SCB->AFSR;  /* implementation defined             */
    volatile uint32_t bfar = SCB->BFAR;  /* valid when CFSR.BFARVALID == 1     */
    volatile uint32_t mmar = SCB->MMFAR; /* valid when CFSR.MMARVALID == 1     */

    __disable_irq();

    /* Only one on-board LED: light it as the fault indicator. */
    turn_on_led_green();

    (void)stacked_r0;  (void)stacked_r1;  (void)stacked_r2;  (void)stacked_r3;
    (void)stacked_r12; (void)stacked_lr;  (void)stacked_pc;  (void)stacked_psr;
    (void)cfsr; (void)hfsr; (void)dfsr; (void)afsr; (void)bfar; (void)mmar;

    /* Set a breakpoint here, read the volatile vars in the debugger and paste
       the hex values into the ARM Fault Analyzer. */
    while (1) { }
}

/******************************************************************************/
