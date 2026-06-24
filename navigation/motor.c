/**
 * @file    motor.c
 * @version 0.2.0
 * @authors Anton Chernov
 * @date    2026-06-22
 * @date    @showdate "%Y-%m-%d"
 */

/******************************** Included files ******************************/
#include "motor.h"
#include "bsp.h"
/********************************* Definitions ********************************/

/**
 * @def MOTOR_SELECT_A
 * @brief Motor selector for motorApply() — motor A.
 */
#define MOTOR_SELECT_A          0U

/**
 * @def MOTOR_SELECT_B
 * @brief Motor selector for motorApply() — motor B.
 */
#define MOTOR_SELECT_B          1U

/***************************** Private prototypes *****************************/

/**
 * @brief Maps a signed command onto the PWM leg duties of one motor.
 * @details Sign-magnitude: the sign selects the driven leg, the magnitude the
 *          duty. Values outside the range are clamped. The duty is scaled to
 *          the board PWM full-scale (BSP_MOTOR_DUTY_FULL) and forwarded to the
 *          BSP.
 * @param[in] ucMotor - MOTOR_SELECT_A or MOTOR_SELECT_B.
 * @param[in] speed   - signed command, clamped to +/- MOTOR_SPEED_MAX.
 */
static void motorApply(uint8_t ucMotor, int16_t speed);

/****************************** Private functions *****************************/

/** @fn motorApply */
static void motorApply(uint8_t ucMotor, int16_t speed) {
    uint16_t usMag;
    uint16_t usFwd;
    uint16_t usRev;

    if (speed > MOTOR_SPEED_MAX) {
        speed = MOTOR_SPEED_MAX;
    }
    else if (speed < -MOTOR_SPEED_MAX) {
        speed = -MOTOR_SPEED_MAX;
    }
    else {
        /* command already within range */
    }

    if (speed >= 0) {
        usMag = (uint16_t)speed;
    }
    else {
        usMag = (uint16_t)(-speed);
    }

    usFwd = (uint16_t)(((uint32_t)usMag * BSP_MOTOR_DUTY_FULL)
                       / (uint32_t)MOTOR_SPEED_MAX);
    usRev = 0U;

    if (speed < 0) {
        usRev = usFwd;          /* reverse leg PWM'd, forward leg held low */
        usFwd = 0U;
    }

    if (ucMotor == MOTOR_SELECT_A) {
        bspMotorSetDutyA(usFwd, usRev);
    }
    else {
        bspMotorSetDutyB(usFwd, usRev);
    }
}

/********************* Application Programming Interface *********************/

/** @fn motorInit */
void motorInit(void) {
    bspMotorInit();

    /* Start stopped; the BSP leaves the SD lines low (bridges disabled). */
    motorSetA(0);
    motorSetB(0);
}
/*----------------------------------------------------------------------------*/

/** @fn motorEnable */
void motorEnable(uint8_t enable) {
    bspMotorEnable(enable);
}
/*----------------------------------------------------------------------------*/

/** @fn motorSetA */
void motorSetA(int16_t speed) {
    motorApply(MOTOR_SELECT_A, speed);
}
/*----------------------------------------------------------------------------*/

/** @fn motorSetB */
void motorSetB(int16_t speed) {
    motorApply(MOTOR_SELECT_B, speed);
}
/*----------------------------------------------------------------------------*/

/** @fn motorStop */
void motorStop(void) {
    motorSetA(0);
    motorSetB(0);
    motorEnable(0U);
}

/******************************************************************************/
