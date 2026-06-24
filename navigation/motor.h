/**
 * @file    motor.h
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-22
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef MOTOR_H_
#define MOTOR_H_
/******************************** Included files ******************************/
#include <stdint.h>
/********************************* Definitions *******************************/

/**
 * @def MOTOR_SPEED_MAX
 * @brief Maximum magnitude of a signed motor speed command (per-mille).
 * @details motorSetA()/motorSetB() accept the closed range
 *          [-MOTOR_SPEED_MAX .. +MOTOR_SPEED_MAX]; the sign selects the
 *          rotation direction and the magnitude the PWM duty cycle.
 */
#define MOTOR_SPEED_MAX         1000

/********************* Application Programming Interface *********************/

/**
 * @brief Brings up TIM1/TIM4, the motor GPIO and the IR2184 shutdown lines.
 * @details Configures PA8/PA11 (Motor A, TIM1_CH1/CH4) and PB8/PB9
 *          (Motor B, TIM4_CH3/CH4) as 20 kHz sign-magnitude PWM outputs and
 *          PB12/PB14 as the per-motor SD/EN outputs, driven to the safe OFF
 *          (disabled) state. Both motors are left stopped; call motorEnable()
 *          to arm the bridges.
 */
void motorInit(void);

/**
 * @brief Arms or disables both H-bridges via the IR2184 SD lines.
 * @details While disabled the bridges float (coast) regardless of the PWM
 *          duty. On reset the SD lines default to the disabled state through
 *          the board pull resistors.
 * @param[in] enable - nonzero to arm the bridges (SD high), 0 to disable.
 */
void motorEnable(uint8_t enable);

/**
 * @brief Sets the signed speed command for motor A.
 * @details Sign-magnitude drive: the sign selects the direction, the
 *          magnitude the PWM duty. Values outside the range are clamped.
 * @param[in] speed - signed command in
 *                    [-MOTOR_SPEED_MAX .. +MOTOR_SPEED_MAX].
 */
void motorSetA(int16_t speed);

/**
 * @brief Sets the signed speed command for motor B.
 * @details See motorSetA().
 * @param[in] speed - signed command in
 *                    [-MOTOR_SPEED_MAX .. +MOTOR_SPEED_MAX].
 */
void motorSetB(int16_t speed);

/**
 * @brief Stops both motors and disables the bridges.
 * @details Sets both PWM duties to zero and deasserts the SD lines, leaving
 *          the motors coasting in the safe OFF state.
 */
void motorStop(void);

/*****************************************************************************/
#endif //! MOTOR_H_
