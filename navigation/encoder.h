/**
 * @file    encoder.h
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-22
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef ENCODER_H_
#define ENCODER_H_
/******************************** Included files ******************************/
#include <stdint.h>
/********************************* Definitions *******************************/

/********************* Application Programming Interface *********************/

/**
 * @brief Brings up the wheel quadrature encoders via the BSP.
 * @details Encoder A is TIM2 (PA0/PA1), encoder B is TIM3 (PA6/PA7). The
 *          travel accumulators and speeds start at zero. Both direction and
 *          magnitude are tracked.
 */
void encoderInit(void);

/**
 * @brief Updates the cached wheel speeds and travel from the encoder counters.
 * @details Call periodically from a scheduler task. Each call takes the signed
 *          counter delta since the previous call, accumulates it into the
 *          odometry position and converts it to a signed rev/min. Never blocks.
 */
void encoderProcess(void);

/**
 * @brief Returns the accumulated signed travel of wheel A.
 * @returns Travel in encoder counts (positive = forward).
 */
int32_t encoderGetCountA(void);

/**
 * @brief Returns the accumulated signed travel of wheel B.
 * @returns Travel in encoder counts (positive = forward).
 */
int32_t encoderGetCountB(void);

/**
 * @brief Returns the latest measured speed of wheel A.
 * @returns Speed in rev/min (signed; negative = reverse).
 */
int16_t encoderGetRpmA(void);

/**
 * @brief Returns the latest measured speed of wheel B.
 * @returns Speed in rev/min (signed; negative = reverse).
 */
int16_t encoderGetRpmB(void);

/*****************************************************************************/
#endif //! ENCODER_H_
