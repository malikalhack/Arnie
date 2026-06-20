/**
 * @file    main.c
 * @version 0.3.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 */

/******************************** Included files ******************************/
#include "bsp.h"
#include "lidar.h"
#include "acrosched.h"
#include "acrosched_defs.h"

/********************************* Definitions ********************************/

/**
 * @def HEARTBEAT_PERIOD_MS
 * @brief Period of the heartbeat / telemetry task in milliseconds.
 */
#define HEARTBEAT_PERIOD_MS     500U

/**
 * @def CHECK_STATUS
 * @brief Invoke Error_Handler() if a scheduler call did not return eAcroOk.
 */
#define CHECK_STATUS(x)         \
    do {                        \
        if ((x) != eAcroOk) {   \
            Error_Handler();    \
        }                       \
    } while (0)

/**
 * @enum ETaskDescriptor
 * @brief User task descriptors for the scheduler pool.
 */
enum ETaskDescriptor {
    eDscrLidar     = 1, /**< Lidar DMA drain + frame parse (realtime). */
    eDscrHeartbeat = 2  /**< LED blink + scan telemetry (periodic).    */
};

/***************************** Private prototypes *****************************/

/** @brief Unrecoverable-error trap: lights the LED and halts. */
static void Error_Handler(void);

/** @brief Realtime task: services the lidar receiver. */
static void taskLidar(AcroParam_t pParam);

/** @brief Periodic task: toggles the LED and reports the front distance. */
static void taskHeartbeat(AcroParam_t pParam);

/****************************** Private functions *****************************/

/** @fn Error_Handler */
static void Error_Handler(void) {
    turn_on_led_green();
    for (;;) {
    }
}
/*----------------------------------------------------------------------------*/

/** @fn taskLidar */
static void taskLidar(AcroParam_t pParam) {
    UNUSED(pParam);
    lidarProcess();
}
/*----------------------------------------------------------------------------*/

/** @fn taskHeartbeat */
static void taskHeartbeat(AcroParam_t pParam) {
    static uint8_t  uc_led_on = 0U;
    const uint16_t *pScan;

    UNUSED(pParam);

    if (uc_led_on != 0U) {
        turn_off_led_green();
        uc_led_on = 0U;
    }
    else {
        turn_on_led_green();
        uc_led_on = 1U;
    }

    pScan = lidarGetScan();

    uartSendStr("scans=");
    uartSendUint16((uint16_t)lidarGetScanCount());
    uartSendStr(" front=");
    uartSendUint16(pScan[0]);
    uartSendStr(" mm  speed=");
    uartSendUint8(lidarGetSpeedRaw());
    uartSendStr("\r\n");
}
/*----------------------------------------------------------------------------*/

/********************* Application Programming Interface *********************/

/** @fn main */
int main(void) {
    bspStart();
    lidarInit();

    CHECK_STATUS(acroInit(&sys_tick));
    CHECK_STATUS(acroAddTask(
        eDscrLidar, taskLidar, NULL, (uint8_t)eRealtime, 0U, 1U
    ));
    CHECK_STATUS(acroAddTask(
        eDscrHeartbeat, taskHeartbeat, NULL,
        (uint8_t)ePeriodic, HEARTBEAT_PERIOD_MS, 0U
    ));
    IGNORE_RETURN(acroRun());   /* dispatcher loop - never returns */

    Error_Handler();            /* reached only if acroRun() returns */
    return 0;
}
/******************************************************************************/
