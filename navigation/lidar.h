/**
 * @file    lidar.h
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef LIDAR_H_
#define LIDAR_H_
/******************************** Included files ******************************/
#include <stdint.h>
/********************************* Definitions *******************************/

/**
 * @def LIDAR_SECTOR_COUNT
 * @brief Number of angular sectors in the polar histogram (1° resolution).
 */
#define LIDAR_SECTOR_COUNT      360U

/********************* Application Programming Interface *********************/

/**
 * @brief Initializes the lidar receiver (USART1 RX via DMA) and clears the
 *        internal polar histogram.
 * @details USART1 must already be configured (baud, 8N1, RE/UE enabled) by the
 *          BSP. This routine attaches DMA1 Channel 5 to USART1_RX in circular
 *          mode and resets the frame parser.
 */
void lidarInit(void);

/**
 * @brief Drains newly received bytes from the DMA buffer and runs the frame
 *        parser, accumulating measurements into the polar histogram.
 * @details Call periodically from the main loop. A completed revolution is
 *          published into the readable scan buffer and the scan counter is
 *          incremented.
 */
void lidarProcess(void);

/**
 * @brief Returns a pointer to the most recently completed scan.
 * @details The array holds the minimum measured distance (in millimetres) for
 *          each 1° sector; a value of 0 means "no measurement" in that sector.
 * @returns Pointer to an array of LIDAR_SECTOR_COUNT uint16_t values.
 */
const uint16_t *lidarGetScan(void);

/**
 * @brief Returns the number of completed revolutions (scans) so far.
 * @returns Monotonically increasing scan counter.
 */
uint32_t lidarGetScanCount(void);

/**
 * @brief Returns the latest radar rotation speed.
 * @returns Rotation speed in raw units of 0.05 r/s per LSB.
 */
uint8_t lidarGetSpeedRaw(void);

/**
 * @brief Returns the device health/fault status.
 * @returns Nonzero once a speed-fault frame (0xAE) has been received.
 */
uint8_t lidarGetFault(void);

/*****************************************************************************/
#endif //! LIDAR_H_
