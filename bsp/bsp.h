/**
 * @file    bsp.h
 * @version 0.6.0
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

/**
 * @brief Transmits a signed 16-bit integer as decimal digits over UART.
 * @details Negative values are prefixed with '-'; leading zeros are suppressed.
 * @param[in] n - value to transmit (-32768..32767).
 */
void uartSendInt16(int16_t n);

/**
 * @brief Transmits an unsigned 8-bit integer as two hexadecimal digits.
 * @param[in] n - value to transmit (0..255).
 */
void uartSendHex8(uint8_t n);

/** @brief Turn on the on-board LED (PC13, active-low). */
void turn_on_led_green(void);

/** @brief Turn off the on-board LED. */
void turn_off_led_green(void);

/*------------------------------ Lidar RX DMA -------------------------------*/

/**
 * @brief Configures DMA1 Channel 5 for USART1 RX in circular mode.
 * @details Routes the USART1 receiver into the caller-owned circular buffer.
 *          The link is simplex (radar → MCU); no transmit path is set up.
 * @param[in] pBuf - destination circular buffer (must not be NULL).
 * @param[in] len  - buffer length in bytes.
 */
void bspLidarRxDmaInit(uint8_t *pBuf, uint16_t len);

/**
 * @brief Returns the DMA write index within the RX buffer.
 * @details The number of bytes the controller has written, modulo @p len,
 *          i.e. (len - DMA remaining count). Used to drain the circular
 *          buffer without interrupts.
 * @param[in] len - buffer length passed to bspLidarRxDmaInit().
 * @returns Current write index in the range [0 .. len-1].
 */
uint16_t bspLidarRxDmaIndex(uint16_t len);

/*------------------------------ I2C1 (polled) ------------------------------*/

/**
 * @brief Brings up I2C1 (PB6 SCL, PB7 SDA) in standard mode 100 kHz.
 * @details Uses the bus device's own pull-ups. All transfers are polled with
 *          a bounded timeout, so a stuck bus can never hang the scheduler.
 */
void bspI2c1Init(void);

/**
 * @brief Probes a 7-bit I2C address for an acknowledge (bus scan helper).
 * @details Issues START, the address byte with the write bit, then evaluates
 *          ACK vs NACK and releases the bus with STOP. Polled and bounded.
 * @param[in] addr7 - 7-bit slave address to probe.
 * @returns Nonzero if the device acknowledged; 0 on NACK or bus timeout.
 */
uint8_t bspI2c1Ping(uint8_t addr7);

/**
 * @brief Samples the idle logic level of the SCL and SDA lines.
 * @details Temporarily switches PB6/PB7 to floating input, reads the pins,
 *          then restores the I2C alternate function. On a healthy powered bus
 *          both lines read high (pulled up). A low line indicates missing
 *          power, a missing pull-up, or a short to ground.
 * @returns Bit0 = SCL (PB6) level, bit1 = SDA (PB7) level.
 */
uint8_t bspI2c1LineLevels(void);

/**
 * @brief Writes a single register on an I2C device (polled, bounded).
 * @param[in] addr7 - 7-bit slave address.
 * @param[in] reg   - register address.
 * @param[in] val   - value to store.
 * @returns Nonzero on success; 0 on bus timeout.
 */
uint8_t bspI2c1WriteReg(uint8_t addr7, uint8_t reg, uint8_t val);

/**
 * @brief Reads a block of registers from an I2C device (polled, bounded).
 * @details Implements the RM0008 N > 2 closing sequence; @p len must be >= 3.
 * @param[in]  addr7 - 7-bit slave address.
 * @param[in]  reg   - starting register address.
 * @param[out] pBuf  - destination buffer.
 * @param[in]  len   - number of bytes to read (>= 3).
 * @returns Nonzero on success; 0 on bus timeout.
 */
uint8_t bspI2c1ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *pBuf, uint8_t len);

/*------------------------------- Motor drive -------------------------------*/

/**
 * @def BSP_MOTOR_DUTY_FULL
 * @brief PWM compare value corresponding to 100 % duty (timer auto-reload).
 * @details A driver maps its normalised command onto [0 .. BSP_MOTOR_DUTY_FULL]
 *          before calling bspMotorSetDutyA()/bspMotorSetDutyB().
 */
#define BSP_MOTOR_DUTY_FULL     3599U

/**
 * @brief Brings up TIM1/TIM4, the motor GPIO and the IR2184 shutdown lines.
 * @details Motor A on TIM1_CH1 (PA8) / TIM1_CH4 (PA11); Motor B on TIM4_CH3
 *          (PB8) / TIM4_CH4 (PB9); 20 kHz PWM. SD/EN lines PB12/PB14 are driven
 *          to the safe OFF (disabled) state and both duties are left at zero.
 */
void bspMotorInit(void);

/**
 * @brief Sets the two PWM leg duties of motor A.
 * @param[in] fwd - forward-leg compare value (0 .. BSP_MOTOR_DUTY_FULL).
 * @param[in] rev - reverse-leg compare value (0 .. BSP_MOTOR_DUTY_FULL).
 */
void bspMotorSetDutyA(uint16_t fwd, uint16_t rev);

/**
 * @brief Sets the two PWM leg duties of motor B.
 * @param[in] fwd - forward-leg compare value (0 .. BSP_MOTOR_DUTY_FULL).
 * @param[in] rev - reverse-leg compare value (0 .. BSP_MOTOR_DUTY_FULL).
 */
void bspMotorSetDutyB(uint16_t fwd, uint16_t rev);

/**
 * @brief Arms or disables both H-bridges via the IR2184 SD lines.
 * @param[in] enable - nonzero to arm (SD high), 0 to disable (SD low).
 */
void bspMotorEnable(uint8_t enable);

/*---------------------------- Wheel encoders -------------------------------*/

/**
 * @brief Brings up TIM2 and TIM3 as ×4 quadrature encoder interfaces.
 * @details Encoder A on TIM2 (PA0 = CH1/A, PA1 = CH2/B); encoder B on TIM3
 *          (PA6 = CH1/A, PA7 = CH2/B). Each timer counts on both edges of both
 *          channels (encoder mode 3), so the hardware 16-bit counter tracks
 *          signed wheel travel and direction without CPU intervention.
 */
void bspEncoderInit(void);

/**
 * @brief Returns the raw 16-bit quadrature count of encoder A (TIM2).
 * @details Free-running, wraps modulo 65536. A driver takes the signed
 *          difference between successive reads to obtain travel and direction.
 * @returns Current counter value.
 */
uint16_t bspEncoderCountA(void);

/**
 * @brief Returns the raw 16-bit quadrature count of encoder B (TIM3).
 * @returns Current counter value.
 */
uint16_t bspEncoderCountB(void);

/*****************************************************************************/
#endif //! BSP_H_
