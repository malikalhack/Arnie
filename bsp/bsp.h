/**
 * @file    bsp.h
 * @version 0.3.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef BSP_H_
#define BSP_H_
/******************************** Included files ******************************/
#include <stdint.h>
#include "acrosched.h"
/********************************* Definitions *******************************/

/**
 * @def BSP_TICKS_PER_SEC
 * @brief SysTick timer frequency in Hz.
 */
#define BSP_TICKS_PER_SEC   1000U

/********************* Application Programming Interface *********************/

/**
 * @brief System tick counter, incremented every 1 ms by the SysTick ISR.
 * @details Pass &sys_tick to acroInit() as the scheduler time source.
 *          Defined in bsp.c.
 */
extern volatile AcroTick_t sys_tick;

/**
 * @brief Starts the BSP runtime services: 1 ms SysTick, fault exceptions and
 *        global interrupts.
 * @details Call once from main() after reset. SystemInit() has already
 *          configured clocks, GPIO and USART1 before main runs; this routine
 *          brings up the timebase used by the scheduler. Kept as a named,
 *          platform-portable entry point (each port provides its own).
 */
void bspStart(void);

/**
 * @brief Returns a coherent snapshot of the system tick counter.
 * @returns Current value of the system tick counter.
 */
AcroTick_t bspGetTick(void);

/**
 * @brief Get the system core clock frequency.
 * @returns System core clock frequency in Hz.
 */
unsigned int get_system_core_clock(void);

/** @brief Reset the MCU. */
void reset_mcu(void);

/**
 * @brief Transmits a single character over UART (blocking).
 * @param[in] c - character to transmit.
 */
void uartSendChar(char c);

/**
 * @brief Transmits a null-terminated string over UART (blocking).
 * @param[in] str - pointer to the string to transmit (must not be NULL).
 */
void uartSendStr(const char *str);

/**
 * @brief Transmits an unsigned 8-bit integer as decimal digits over UART.
 * @param[in] n - value to transmit (0..255).
 */
void uartSendUint8(uint8_t n);

/**
 * @brief Transmits an unsigned 16-bit integer as decimal digits over UART.
 * @details Leading zeros are suppressed; the value 0 is printed as "0".
 * @param[in] n - value to transmit (0..65535).
 */
void uartSendUint16(uint16_t n);

/** @brief Turn on the on-board LED (PC13, active-low). */
void turn_on_led_green(void);

/** @brief Turn off the on-board LED. */
void turn_off_led_green(void);

/*****************************************************************************/
#endif //! BSP_H_
