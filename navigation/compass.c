/**
 * @file    compass.c
 * @version 0.3.0
 * @authors Anton Chernov
 * @date    2026-06-21
 * @date    @showdate "%Y-%m-%d"
 *
 * @brief   HMC5883L / QMC5883L 3-axis magnetometer driver over I2C1 (polled).
 *
 * @details The sensor sits on I2C1 (PB6 SCL, PB7 SDA) with the module's own
 *          pull-ups. At start-up the driver auto-detects the chip: a Honeywell
 *          HMC5883L (address 0x1E, signature 'H','4','3') or the pin-compatible
 *          QST QMC5883L (address 0x0D, chip-ID 0xFF) found on most 4-pin GY-271
 *          modules. Bus transfers are polled with bounded timeouts so a stuck
 *          bus can never hang the cooperative scheduler. The HMC streams
 *          X, Z, Y big-endian; the QMC streams X, Y, Z little-endian. The
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
 * @def QMC_ADDR
 * @brief 7-bit I2C slave address of the QMC5883L.
 */
#define QMC_ADDR                0x0DU

/**
 * @def QMC_REG_DATA
 * @brief First data output register (X LSB); order is X, Y, Z, little-endian.
 */
#define QMC_REG_DATA            0x00U

/**
 * @def QMC_REG_CONTROL1
 * @brief Control register 1 (oversampling, range, output rate, mode).
 */
#define QMC_REG_CONTROL1        0x09U

/**
 * @def QMC_REG_SET_RESET
 * @brief SET/RESET period register (datasheet mandates 0x01).
 */
#define QMC_REG_SET_RESET       0x0BU

/**
 * @def QMC_REG_CHIP_ID
 * @brief Chip identification register; reads 0xFF on the QMC5883L.
 */
#define QMC_REG_CHIP_ID         0x0DU

/**
 * @def QMC_CHIP_ID_VAL
 * @brief Expected chip-ID value of the QMC5883L.
 */
#define QMC_CHIP_ID_VAL         0xFFU

/**
 * @def QMC_CONTROL1_VAL
 * @brief OSR=512, ±2 Gauss range, 100 Hz output, continuous mode.
 */
#define QMC_CONTROL1_VAL        0x09U

/**
 * @def QMC_SET_RESET_VAL
 * @brief Recommended SET/RESET period value.
 */
#define QMC_SET_RESET_VAL       0x01U

/**
 * @def COMPASS_DATA_LEN
 * @brief Number of data bytes read per sample (X, Z, Y × 2).
 */
#define COMPASS_DATA_LEN        6U

/**
 * @def COMPASS_ID_LEN
 * @brief Number of identification bytes read while probing a device.
 */
#define COMPASS_ID_LEN          3U

/**
 * @def COMPASS_CHIP_NONE
 * @brief No magnetometer was detected on the bus.
 */
#define COMPASS_CHIP_NONE       0U

/**
 * @def COMPASS_CHIP_HMC
 * @brief Detected device is a Honeywell HMC5883L.
 */
#define COMPASS_CHIP_HMC        1U

/**
 * @def COMPASS_CHIP_QMC
 * @brief Detected device is a QST QMC5883L.
 */
#define COMPASS_CHIP_QMC        2U

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
static uint8_t  compass_chip;

/***************************** Private prototypes *****************************/

/** @brief Recomputes the cached heading from the X/Y components. */
static void compassUpdateHeading(void);

/**
 * @brief Detects and configures the magnetometer, setting the fault flag.
 * @details Resets the I2C bus, probes for an HMC5883L (0x1E) then a QMC5883L
 *          (0x0D), and on success programs continuous measurement. Safe to
 *          call repeatedly, so a brittle power-on instant cannot latch the
 *          fault state permanently.
 */
static void compassBringup(void);

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
/*----------------------------------------------------------------------------*/

/** @fn compassBringup */
static void compassBringup(void) {
    uint8_t id[COMPASS_ID_LEN];
    uint8_t ucCfgOk;

    compass_fault = 1U;            /* assume failure until identified */
    compass_chip  = COMPASS_CHIP_NONE;

    bspI2c1Init();                 /* includes bus recovery */

    /* Probe 1 - Honeywell HMC5883L at 0x1E (signature 'H','4','3'). */
    if (bspI2c1ReadRegs(HMC_ADDR, HMC_REG_IDENT_A, id, COMPASS_ID_LEN) != 0U) {
        if (
            (id[0] == HMC_ID_A) &&
            (id[1] == HMC_ID_B) &&
            (id[2] == HMC_ID_C)
        ) {
            compass_chip = COMPASS_CHIP_HMC;
        }
    }

    /* Probe 2 - QST QMC5883L at 0x0D (chip-ID register reads 0xFF). */
    if (compass_chip == COMPASS_CHIP_NONE) {
        if (
            bspI2c1ReadRegs(QMC_ADDR, QMC_REG_CHIP_ID, id, COMPASS_ID_LEN) != 0U
        ) {
            if (id[0] == QMC_CHIP_ID_VAL) {
                compass_chip = COMPASS_CHIP_QMC;
            }
        }
    }

    /* Configure whichever device answered. */
    if (compass_chip == COMPASS_CHIP_HMC) {
        ucCfgOk  = bspI2c1WriteReg(HMC_ADDR, HMC_REG_CONFIG_A, HMC_CONFIG_A_VAL);
        ucCfgOk &= bspI2c1WriteReg(HMC_ADDR, HMC_REG_CONFIG_B, HMC_CONFIG_B_VAL);
        ucCfgOk &= bspI2c1WriteReg(HMC_ADDR, HMC_REG_MODE, HMC_MODE_CONTINUOUS);
        if (ucCfgOk != 0U) {
            compass_fault = 0U;
        }
    }
    else if (compass_chip == COMPASS_CHIP_QMC) {
        ucCfgOk  = bspI2c1WriteReg(QMC_ADDR, QMC_REG_SET_RESET, QMC_SET_RESET_VAL);
        ucCfgOk &= bspI2c1WriteReg(QMC_ADDR, QMC_REG_CONTROL1, QMC_CONTROL1_VAL);
        if (ucCfgOk != 0U) {
            compass_fault = 0U;
        }
    }
    else {
        /* No magnetometer detected - leave compass_fault asserted. */
    }
}

/********************* Application Programming Interface *********************/

/** @fn compassInit */
void compassInit(void) {
    mag_x         = 0;
    mag_y         = 0;
    mag_z         = 0;
    heading_deci  = 0U;

    compassBringup();
}
/*----------------------------------------------------------------------------*/

/** @fn compassProcess */
uint8_t compassProcess(void) {
    uint8_t raw[COMPASS_DATA_LEN];
    uint8_t ret_val = 0U;

    if (compass_fault != 0U) {
        compassBringup();          /* retry detection until the bus settles */
    }

    if (compass_fault == 0U) {
        if (compass_chip == COMPASS_CHIP_HMC) {
            if (
                bspI2c1ReadRegs(HMC_ADDR, HMC_REG_DATA, raw, COMPASS_DATA_LEN)
                    != 0U
            ) {
                mag_x = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
                mag_z = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
                mag_y = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);
                compassUpdateHeading();
                ret_val = 1U;
            }
        }
        else {
            if (
                bspI2c1ReadRegs(QMC_ADDR, QMC_REG_DATA, raw, COMPASS_DATA_LEN)
                    != 0U
            ) {
                mag_x = (int16_t)(((uint16_t)raw[1] << 8U) | raw[0]);
                mag_y = (int16_t)(((uint16_t)raw[3] << 8U) | raw[2]);
                mag_z = (int16_t)(((uint16_t)raw[5] << 8U) | raw[4]);
                compassUpdateHeading();
                ret_val = 1U;
            }
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
