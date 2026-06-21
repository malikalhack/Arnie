/**
 * @file    compass.h
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-21
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef COMPASS_H_
#define COMPASS_H_
/******************************** Included files ******************************/
#include <stdint.h>
/********************************* Definitions *******************************/

/********************* Application Programming Interface *********************/

/**
 * @brief Brings up I2C1 (PB6 SCL, PB7 SDA) and configures the HMC5883L
 *        magnetometer for continuous measurement.
 * @details Verifies the device identity ('H','4','3'); on mismatch or bus
 *          error the fault flag is set and subsequent reads are skipped.
 */
void compassInit(void);

/**
 * @brief Reads one magnetometer sample and updates the cached vector and
 *        heading.
 * @details Call periodically. Uses polled I2C with timeouts, so it never
 *          blocks the cooperative scheduler indefinitely.
 * @returns Nonzero if a fresh sample was read; 0 on fault or bus timeout.
 */
uint8_t compassProcess(void);

/**
 * @brief Returns the most recent raw magnetometer vector.
 * @details Values are signed 16-bit counts (gain 1090 LSB/Gauss). NULL
 *          pointers are ignored.
 * @param[out] pX - destination for the X axis (may be NULL).
 * @param[out] pY - destination for the Y axis (may be NULL).
 * @param[out] pZ - destination for the Z axis (may be NULL).
 */
void compassGetRaw(int16_t *pX, int16_t *pY, int16_t *pZ);

/**
 * @brief Returns the latest heading in deci-degrees.
 * @returns Heading in the range 0..3599 (0.1° per LSB), measured from the
 *          +X axis towards +Y.
 */
uint16_t compassGetHeadingDeci(void);

/**
 * @brief Returns the device fault status.
 * @returns Nonzero if the magnetometer is absent or the bus failed.
 */
uint8_t compassGetFault(void);

/*****************************************************************************/
#endif //! COMPASS_H_
