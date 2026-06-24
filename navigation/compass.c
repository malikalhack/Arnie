/**
 * @file    compass.c
 * @version 0.2.0
 * @authors Anton Chernov
 * @date    2026-06-21
 * @date    @showdate "%Y-%m-%d"
 *
 * @brief   HMC5883L 3-axis magnetometer driver over I2C1 (polled).
 *
 * @details The sensor sits on I2C1 (PB6 SCL, PB7 SDA) with the module's own
 *          pull-ups. It is configured for 8-sample averaging at 15 Hz in
 *          continuous-measurement mode. Bus transfers are polled with bounded
 *          timeouts so a stuck bus can never hang the cooperative scheduler.
 *          The data registers stream X, Z, Y (signed 16-bit, big-endian); the
 *          heading is derived from the horizontal X/Y components.
 */

/******************************** Included files ******************************/
#include "compass.h"
#include "bsp.h"
#include <stddef.h>
/********************************* Definitions ********************************/

/**
 * @def HMC_ADDR
 * @brief 7-bit I2C slave address of the HMC5883L.
 */
#define HMC_ADDR                0x1EU

/**
 * @def HMC_REG_CONFIG_A
 * @brief Configuration register A (averaging, output rate, mode).
 */
#define HMC_REG_CONFIG_A        0x00U

/**
 * @def HMC_REG_CONFIG_B
 * @brief Configuration register B (gain).
 */
#define HMC_REG_CONFIG_B        0x01U

/**
 * @def HMC_REG_MODE
 * @brief Mode register.
 */
#define HMC_REG_MODE            0x02U

/**
 * @def HMC_REG_DATA
 * @brief First data output register (X MSB); order is X, Z, Y.
 */
#define HMC_REG_DATA            0x03U

/**
 * @def HMC_REG_IDENT_A
 * @brief First identification register; reads 'H','4','3'.
 */
#define HMC_REG_IDENT_A         0x0AU

/**
 * @def HMC_ID_A
 * @brief Identification register A expected value ('H').
 */
#define HMC_ID_A                0x48U

/**
 * @def HMC_ID_B
 * @brief Identification register B expected value ('4').
 */
#define HMC_ID_B                0x34U

/**
 * @def HMC_ID_C
 * @brief Identification register C expected value ('3').
 */
#define HMC_ID_C                0x33U

/**
 * @def HMC_CONFIG_A_VAL
 * @brief 8-sample averaging, 15 Hz output, normal measurement.
 */
#define HMC_CONFIG_A_VAL        0x70U

/**
 * @def HMC_CONFIG_B_VAL
 * @brief Gain = 1090 LSB/Gauss (±1.3 Gauss range).
 */
#define HMC_CONFIG_B_VAL        0x20U

/**
 * @def HMC_MODE_CONTINUOUS
 * @brief Continuous-measurement mode.
 */
#define HMC_MODE_CONTINUOUS     0x00U

/**
 * @def COMPASS_DATA_LEN
 * @brief Number of data bytes read per sample (X, Z, Y × 2).
 */
#define COMPASS_DATA_LEN        6U

/**
 * @def COMPASS_TWO_PI
 * @brief 2π constant for heading wrap-around.
 */
#define COMPASS_TWO_PI          6.28318531f

/**
 * @def COMPASS_DECLINATION_RAD
 * @brief Local magnetic declination added to the heading, in radians.
 */
#define COMPASS_DECLINATION_RAD 0.0f

/**
 * @def COMPASS_RAD_TO_DECIDEG
 * @brief Radians → deci-degrees scale (3600 / 2π).
 */
#define COMPASS_RAD_TO_DECIDEG  572.957795f

/**
 * @def COMPASS_PI_4
 * @brief π/4 constant for the atan2 approximation.
 */
#define COMPASS_PI_4            0.785398163f

/**
 * @def COMPASS_3PI_4
 * @brief 3π/4 constant for the atan2 approximation.
 */
#define COMPASS_3PI_4           2.35619449f

/**
 * @def COMPASS_ATAN_C1
 * @brief Linear coefficient of the cubic atan2 approximation.
 */
#define COMPASS_ATAN_C1         0.9817f

/**
 * @def COMPASS_ATAN_C3
 * @brief Cubic coefficient of the cubic atan2 approximation.
 */
#define COMPASS_ATAN_C3         0.1963f

/****************************** Module variables ******************************/

static int16_t  mag_x;
static int16_t  mag_y;
static int16_t  mag_z;
static uint16_t heading_deci;
static uint8_t  compass_fault;

/***************************** Private prototypes *****************************/

/** @brief Recomputes the cached heading from the X/Y components. */
static void compassUpdateHeading(void);

/**
 * @brief Four-quadrant arc-tangent approximation (no libm dependency).
 * @details Cubic approximation accurate to about 0.3°, using only basic
 *          floating-point arithmetic provided by the compiler runtime.
 * @param[in] y - ordinate.
 * @param[in] x - abscissa.
 * @returns Angle in radians in the range [-π, π].
 */
static float compassAtan2(float y, float x);

/****************************** Private functions *****************************/

/** @fn compassAtan2 */
static float compassAtan2(float y, float x) {
    float fResult = 0.0f;
    float fAbsY;
    float fR;

    fAbsY = (y < 0.0f) ? -y : y;

    if ((x != 0.0f) || (y != 0.0f)) {
        if (x >= 0.0f) {
            fR      = (x - fAbsY) / (x + fAbsY);
            fResult = (COMPASS_ATAN_C3 * fR * fR * fR)
                    - (COMPASS_ATAN_C1 * fR) + COMPASS_PI_4;
        }
        else {
            fR      = (x + fAbsY) / (fAbsY - x);
            fResult = (COMPASS_ATAN_C3 * fR * fR * fR)
                    - (COMPASS_ATAN_C1 * fR) + COMPASS_3PI_4;
        }
        if (y < 0.0f) {
            fResult = -fResult;
        }
    }
    return fResult;
}
/*----------------------------------------------------------------------------*/

/** @fn compassUpdateHeading */
static void compassUpdateHeading(void) {
    float fHeading;

    fHeading  = compassAtan2((float)mag_y, (float)mag_x);
    fHeading += COMPASS_DECLINATION_RAD;

    if (fHeading < 0.0f) {
        fHeading += COMPASS_TWO_PI;
    }
    if (fHeading >= COMPASS_TWO_PI) {
        fHeading -= COMPASS_TWO_PI;
    }

    heading_deci = (uint16_t)(fHeading * COMPASS_RAD_TO_DECIDEG);
}

/********************* Application Programming Interface *********************/

/** @fn compassInit */
void compassInit(void) {
    uint8_t id[3];
    uint8_t ucCfgOk;

    mag_x         = 0;
    mag_y         = 0;
    mag_z         = 0;
    heading_deci  = 0U;
    compass_fault = 1U;            /* assume failure until identified */

    bspI2c1Init();

    if (bspI2c1ReadRegs(HMC_ADDR, HMC_REG_IDENT_A, id, 3U) != 0U) {
        if (
            (id[0] == HMC_ID_A) &&
            (id[1] == HMC_ID_B) &&
            (id[2] == HMC_ID_C)
        ) {
            compass_fault = 0U;
        }
    }

    if (compass_fault == 0U) {
        ucCfgOk  = bspI2c1WriteReg(HMC_ADDR, HMC_REG_CONFIG_A, HMC_CONFIG_A_VAL);
        ucCfgOk &= bspI2c1WriteReg(HMC_ADDR, HMC_REG_CONFIG_B, HMC_CONFIG_B_VAL);
        ucCfgOk &= bspI2c1WriteReg(HMC_ADDR, HMC_REG_MODE, HMC_MODE_CONTINUOUS);
        if (ucCfgOk == 0U) {
            compass_fault = 1U;
        }
    }
}
/*----------------------------------------------------------------------------*/

/** @fn compassProcess */
uint8_t compassProcess(void) {
    uint8_t raw[COMPASS_DATA_LEN];
    uint8_t ret_val = 0U;

    if (compass_fault == 0U) {
        if (bspI2c1ReadRegs(HMC_ADDR, HMC_REG_DATA, raw, COMPASS_DATA_LEN) != 0U) {
            mag_x = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
            mag_z = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
            mag_y = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);
            compassUpdateHeading();
            ret_val = 1U;
        }
    }
    return ret_val;
}
/*----------------------------------------------------------------------------*/

/** @fn compassGetRaw */
void compassGetRaw(int16_t *pX, int16_t *pY, int16_t *pZ) {
    if (pX != NULL) {
        *pX = mag_x;
    }
    if (pY != NULL) {
        *pY = mag_y;
    }
    if (pZ != NULL) {
        *pZ = mag_z;
    }
}
/*----------------------------------------------------------------------------*/

/** @fn compassGetHeadingDeci */
uint16_t compassGetHeadingDeci(void) {
    return heading_deci;
}
/*----------------------------------------------------------------------------*/

/** @fn compassGetFault */
uint8_t compassGetFault(void) {
    return compass_fault;
}
/******************************************************************************/
