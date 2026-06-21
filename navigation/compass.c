/**
 * @file    compass.c
 * @version 0.1.0
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
#include "RTE_Components.h"
#include CMSIS_device_header
#include "compass.h"
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
 * @def COMPASS_FREQ_MHZ
 * @brief I2C input clock (PCLK1) in MHz, loaded into I2C_CR2 FREQ.
 */
#define COMPASS_FREQ_MHZ        36U

/**
 * @def COMPASS_CCR_STD
 * @brief CCR for 100 kHz standard mode: PCLK1 / (2 × Fscl) = 36M / 200k.
 */
#define COMPASS_CCR_STD         180U

/**
 * @def COMPASS_TRISE_STD
 * @brief TRISE for standard mode: FREQ + 1 (1000 ns / 27.8 ns + 1).
 */
#define COMPASS_TRISE_STD       37U

/**
 * @def COMPASS_I2C_TIMEOUT
 * @brief Poll iterations before a bus operation is abandoned.
 */
#define COMPASS_I2C_TIMEOUT     50000U

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

/** @brief Configures GPIOB pins and the I2C1 peripheral. */
static void compassI2CInit(void);

/**
 * @brief Waits for an I2C_SR1 flag with a bounded timeout.
 * @param[in] flag - SR1 bit mask to wait for.
 * @returns Nonzero if the flag was observed; 0 on timeout.
 */
static uint8_t i2cWaitSR1(uint16_t flag);

/**
 * @brief Writes a single register on the magnetometer.
 * @param[in] reg - register address.
 * @param[in] val - value to store.
 * @returns Nonzero on success; 0 on bus timeout.
 */
static uint8_t compassWriteReg(uint8_t reg, uint8_t val);

/**
 * @brief Reads a block of registers starting at @p reg.
 * @param[in]  reg   - starting register address.
 * @param[out] pBuf  - destination buffer.
 * @param[in]  len   - number of bytes to read (must be >= 3).
 * @returns Nonzero on success; 0 on bus timeout.
 */
static uint8_t compassReadRegs(uint8_t reg, uint8_t *pBuf, uint8_t len);

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

/** @fn compassI2CInit */
static void compassI2CInit(void) {
    /* Enable GPIOB and I2C1 clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /*
     * PB6 (I2C1_SCL) and PB7 (I2C1_SDA): alternate-function open-drain,
     * 50 MHz → CNF=11, MODE=11 → 0xF. PB6 = CRL[27:24], PB7 = CRL[31:28].
     */
    GPIOB->CRL = (GPIOB->CRL & ~((0xFUL << 24U) | (0xFUL << 28U)))
               | (0xFUL << 24U)    /* PB6 SCL */
               | (0xFUL << 28U);   /* PB7 SDA */

    /* Software-reset the peripheral, then program standard mode 100 kHz */
    I2C1->CR1   = I2C_CR1_SWRST;
    I2C1->CR1   = 0U;
    I2C1->CR2   = COMPASS_FREQ_MHZ;
    I2C1->CCR   = COMPASS_CCR_STD;
    I2C1->TRISE = COMPASS_TRISE_STD;
    I2C1->CR1   = I2C_CR1_PE;
}
/*----------------------------------------------------------------------------*/

/** @fn i2cWaitSR1 */
static uint8_t i2cWaitSR1(uint16_t flag) {
    uint8_t  ret_val   = 0U;
    uint32_t ulTimeout = COMPASS_I2C_TIMEOUT;

    while (ulTimeout != 0U) {
        if ((I2C1->SR1 & flag) != 0U) {
            ret_val   = 1U;
            ulTimeout = 0U;     /* flag observed → leave the loop */
        }
        else {
            ulTimeout--;
        }
    }
    return ret_val;
}
/*----------------------------------------------------------------------------*/

/** @fn compassWriteReg */
static uint8_t compassWriteReg(uint8_t reg, uint8_t val) {
    uint8_t ucOk = 1U;

    I2C1->CR1 |= I2C_CR1_START;
    ucOk &= i2cWaitSR1(I2C_SR1_SB);

    I2C1->DR = (uint16_t)(HMC_ADDR << 1U);     /* address + write */
    ucOk &= i2cWaitSR1(I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;                           /* clear ADDR */

    ucOk &= i2cWaitSR1(I2C_SR1_TXE);
    I2C1->DR = (uint16_t)reg;

    ucOk &= i2cWaitSR1(I2C_SR1_TXE);
    I2C1->DR = (uint16_t)val;

    ucOk &= i2cWaitSR1(I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;

    return ucOk;
}
/*----------------------------------------------------------------------------*/

/** @fn compassReadRegs */
static uint8_t compassReadRegs(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    uint8_t ucOk = 1U;
    uint8_t i    = 0U;

    /* Phase 1 — point the device at the starting register */
    I2C1->CR1 |= I2C_CR1_START;
    ucOk &= i2cWaitSR1(I2C_SR1_SB);
    I2C1->DR = (uint16_t)(HMC_ADDR << 1U);     /* address + write */
    ucOk &= i2cWaitSR1(I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;                           /* clear ADDR */
    ucOk &= i2cWaitSR1(I2C_SR1_TXE);
    I2C1->DR = (uint16_t)reg;
    ucOk &= i2cWaitSR1(I2C_SR1_BTF);

    /* Phase 2 — repeated start, switch to receiver, read len bytes */
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    ucOk &= i2cWaitSR1(I2C_SR1_SB);
    I2C1->DR = (uint16_t)((HMC_ADDR << 1U) | 1U);   /* address + read */
    ucOk &= i2cWaitSR1(I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;                           /* clear ADDR */

    /* Read all but the last three bytes with ACK enabled */
    while ((uint8_t)(len - i) > 3U) {
        ucOk &= i2cWaitSR1(I2C_SR1_RXNE);
        pBuf[i] = (uint8_t)I2C1->DR;
        i++;
    }

    /* Closing sequence for the final three bytes (RM0008 N > 2 path) */
    ucOk &= i2cWaitSR1(I2C_SR1_BTF);
    I2C1->CR1 &= (uint16_t)~I2C_CR1_ACK;
    pBuf[i] = (uint8_t)I2C1->DR;               /* data N-2 */
    i++;

    ucOk &= i2cWaitSR1(I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;
    pBuf[i] = (uint8_t)I2C1->DR;               /* data N-1 */
    i++;

    ucOk &= i2cWaitSR1(I2C_SR1_RXNE);
    pBuf[i] = (uint8_t)I2C1->DR;               /* data N   */

    return ucOk;
}
/*----------------------------------------------------------------------------*/

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

    compassI2CInit();

    if (compassReadRegs(HMC_REG_IDENT_A, id, 3U) != 0U) {
        if (
            (id[0] == HMC_ID_A) &&
            (id[1] == HMC_ID_B) &&
            (id[2] == HMC_ID_C)
        ) {
            compass_fault = 0U;
        }
    }

    if (compass_fault == 0U) {
        ucCfgOk  = compassWriteReg(HMC_REG_CONFIG_A, HMC_CONFIG_A_VAL);
        ucCfgOk &= compassWriteReg(HMC_REG_CONFIG_B, HMC_CONFIG_B_VAL);
        ucCfgOk &= compassWriteReg(HMC_REG_MODE, HMC_MODE_CONTINUOUS);
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
        if (compassReadRegs(HMC_REG_DATA, raw, COMPASS_DATA_LEN) != 0U) {
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
