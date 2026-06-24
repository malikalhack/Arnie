/**
 * @file    encoder.c
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-22
 * @date    @showdate "%Y-%m-%d"
 *
 * @brief   Wheel quadrature encoder driver: signed speed and odometry from the
 *          hardware ×4 encoder counters.
 *
 * @details The BSP runs TIM2/TIM3 as ×4 quadrature interfaces; their 16-bit
 *          counters track signed wheel travel. This driver takes the signed
 *          counter delta between calls, accumulates it into an odometry
 *          position and converts it into a signed rev/min (frequency method).
 *          All work is non-blocking so the cooperative scheduler is never
 *          stalled.
 */

/******************************** Included files ******************************/
#include "encoder.h"
#include "bsp.h"
/********************************* Definitions ********************************/

/**
 * @def ENC_PULSES_PER_REV
 * @brief Encoder pulses per channel per revolution of the motor shaft.
 */
#define ENC_PULSES_PER_REV      100U

/**
 * @def ENC_QUADRATURE
 * @brief Decoding factor of the hardware ×4 quadrature counter.
 */
#define ENC_QUADRATURE          4U

/**
 * @def ENC_GEAR_RATIO
 * @brief Gearbox reduction: motor-shaft revolutions per wheel revolution.
 * @details Estimated (≈ 1:15) from the brushed-motor no-load speed; calibrate
 *          on the bench against a measured distance and correct as needed.
 */
#define ENC_GEAR_RATIO          15U

/**
 * @def ENC_COUNTS_PER_WHEEL_REV
 * @brief Hardware counts accumulated over one full wheel revolution.
 */
#define ENC_COUNTS_PER_WHEEL_REV \
    (ENC_PULSES_PER_REV * ENC_QUADRATURE * ENC_GEAR_RATIO)

/****************************** Module variables ******************************/

static uint16_t   prev_a;
static uint16_t   prev_b;
static int32_t    pos_a;
static int32_t    pos_b;
static int16_t    rpm_a;
static int16_t    rpm_b;
static AcroTick_t stamp;

/***************************** Private prototypes *****************************/

/**
 * @brief Converts a signed count delta over an interval to rev/min.
 * @param[in] delta - signed encoder counts accumulated over @p ms.
 * @param[in] ms    - elapsed time in milliseconds (must be nonzero).
 * @returns Signed speed in rev/min.
 */
static int16_t encoderRpm(int16_t delta, AcroTick_t ms);

/****************************** Private functions *****************************/

/** @fn encoderRpm */
static int16_t encoderRpm(int16_t delta, AcroTick_t ms) {
    int16_t ret_val = 0;

    if (ms != 0U) {
        ret_val = (int16_t)(((int32_t)delta * 60000)
                / ((int32_t)ENC_COUNTS_PER_WHEEL_REV * (int32_t)ms));
    }
    return ret_val;
}

/********************* Application Programming Interface *********************/

/** @fn encoderInit */
void encoderInit(void) {
    bspEncoderInit();
    prev_a = bspEncoderCountA();
    prev_b = bspEncoderCountB();
    pos_a  = 0;
    pos_b  = 0;
    rpm_a  = 0;
    rpm_b  = 0;
    stamp  = bspGetTick();
}
/*----------------------------------------------------------------------------*/

/** @fn encoderProcess */
void encoderProcess(void) {
    uint16_t   usCnt;
    int16_t    delta;
    AcroTick_t now;
    AcroTick_t ms;

    now = bspGetTick();
    ms  = (AcroTick_t)(now - stamp);

    if (ms != 0U) {
        usCnt  = bspEncoderCountA();
        delta  = (int16_t)(usCnt - prev_a);
        prev_a = usCnt;
        pos_a += (int32_t)delta;
        rpm_a  = encoderRpm(delta, ms);

        usCnt  = bspEncoderCountB();
        delta  = (int16_t)(usCnt - prev_b);
        prev_b = usCnt;
        pos_b += (int32_t)delta;
        rpm_b  = encoderRpm(delta, ms);

        stamp = now;
    }
}
/*----------------------------------------------------------------------------*/

/** @fn encoderGetCountA */
int32_t encoderGetCountA(void) {
    return pos_a;
}
/*----------------------------------------------------------------------------*/

/** @fn encoderGetCountB */
int32_t encoderGetCountB(void) {
    return pos_b;
}
/*----------------------------------------------------------------------------*/

/** @fn encoderGetRpmA */
int16_t encoderGetRpmA(void) {
    return rpm_a;
}
/*----------------------------------------------------------------------------*/

/** @fn encoderGetRpmB */
int16_t encoderGetRpmB(void) {
    return rpm_b;
}

/******************************************************************************/
