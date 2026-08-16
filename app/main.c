/**
 * @file    main.c
 * @version 0.5.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 */

/******************************** Included files ******************************/
#include "bsp.h"
#include "lidar.h"
#include "compass.h"
#include "motor.h"
#include "encoder.h"
#include "acrosched.h"
#include "acrosched_defs.h"

/********************************* Definitions ********************************/

/**
 * @def COMPASS_ENABLED
 * @brief Set to 1 once the HMC5883L magnetometer is fitted on the I2C bus.
 * @details While 0 the compass is not initialised or polled and no heading
 *          telemetry is emitted, avoiding a permanent cfault and idle I2C
 *          traffic.
 */
#define COMPASS_ENABLED         1U

/**
 * @def MOTOR_ENABLED
 * @brief Set to 1 once the H-bridge motor driver is wired and validated.
 * @details While 0 the motor peripherals (TIM1/TIM4, SD lines) are not
 *          initialised, so the motor pins stay in their reset state and the
 *          bridges remain disabled.
 */
#define MOTOR_ENABLED           0U

/**
 * @def ENCODER_ENABLED
 * @brief Set to 1 once the wheel quadrature encoders are wired.
 * @details While 0 the encoder timers are not initialised and no speed or
 *          odometry telemetry is emitted.
 */
#define ENCODER_ENABLED         0U

/**
 * @def I2C_SCAN_ENABLED
 * @brief Set to 1 to dump an I2C bus scan over the debug UART at start-up.
 * @details Diagnostic aid: lists every 7-bit address that acknowledges, so a
 *          missing or mis-wired magnetometer can be told apart from a wrong
 *          driver. Set back to 0 for normal operation.
 */
#define I2C_SCAN_ENABLED        0U

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

#if (I2C_SCAN_ENABLED == 1)
/** @brief One-shot diagnostic: lists every I2C address that acknowledges. */
static void i2cBusScan(void);
#endif /* I2C_SCAN_ENABLED */

/****************************** Private functions *****************************/

/** @fn Error_Handler */
static void Error_Handler(void) {
    turn_on_led_green();
    for (;;) {
    }
}
/*----------------------------------------------------------------------------*/

#if (I2C_SCAN_ENABLED == 1)
/** @fn i2cBusScan */
static void i2cBusScan(void) {
    uint8_t ucAddr;
    uint8_t ucLvl;

    bspI2c1Init();
    ucLvl = bspI2c1LineLevels();
    uartSendStr("I2C idle SCL=");
    uartSendUint8((uint8_t)(ucLvl & 1U));
    uartSendStr(" SDA=");
    uartSendUint8((uint8_t)((ucLvl >> 1U) & 1U));
    uartSendStr("\r\n");

    uartSendStr("I2C scan:");
    for (ucAddr = 0x08U; ucAddr <= 0x77U; ucAddr++) {
        if (bspI2c1Ping(ucAddr) != 0U) {
            uartSendStr(" 0x");
            uartSendHex8(ucAddr);
        }
    }
    uartSendStr("\r\n");
}
/*----------------------------------------------------------------------------*/
#endif /* I2C_SCAN_ENABLED */

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
    uint16_t        usIdx;
    uint16_t        usMin;
    uint16_t        usCount;
#if (COMPASS_ENABLED == 1)
    uint16_t        hdg;
#endif /* COMPASS_ENABLED */

    UNUSED(pParam);

    if (uc_led_on != 0U) {
        turn_off_led_green();
        uc_led_on = 0U;
    }
    else {
        turn_on_led_green();
        uc_led_on = 1U;
    }

    pScan   = lidarGetScan();
    usCount = 0U;
    usMin   = 0U;
    for (usIdx = 0U; usIdx < LIDAR_SECTOR_COUNT; usIdx++) {
        if (pScan[usIdx] != 0U) {
            usCount++;
            if ((usMin == 0U) || (pScan[usIdx] < usMin)) {
                usMin = pScan[usIdx];
            }
        }
    }

    uartSendStr("scans=");
    uartSendUint16((uint16_t)lidarGetScanCount());
    uartSendStr(" pts=");
    uartSendUint16(usCount);
    uartSendStr(" min=");
    uartSendUint16(usMin);
    uartSendStr(" mm  speed=");
    uartSendUint8(lidarGetSpeedRaw());

#if (COMPASS_ENABLED == 1)
    compassProcess();
    hdg = compassGetHeadingDeci();
    uartSendStr("  hdg=");
    uartSendUint16((uint16_t)(hdg / 10U));
    uartSendChar('.');
    uartSendUint8((uint8_t)(hdg % 10U));
    uartSendStr(" cfault=");
    uartSendUint8(compassGetFault());
#endif /* COMPASS_ENABLED */
#if (ENCODER_ENABLED == 1)
    encoderProcess();
    uartSendStr("  rpmA=");
    uartSendInt16(encoderGetRpmA());
    uartSendStr(" rpmB=");
    uartSendInt16(encoderGetRpmB());
#endif /* ENCODER_ENABLED */
    uartSendStr("\r\n");
}

/********************* Application Programming Interface *********************/

/** @fn acroWatchdogRefresh */
void acroWatchdogRefresh(void) {
    /* Strong override: kick the hardware watchdog once per dispatcher
     * iteration. The kernel calls this only when the loop completes. */
    bspWatchdogKick();
}
/*----------------------------------------------------------------------------*/

/** @fn main */
int main(void) {
    bspStart();
#if (I2C_SCAN_ENABLED == 1)
    i2cBusScan();
#endif /* I2C_SCAN_ENABLED */
    lidarInit();
#if (COMPASS_ENABLED == 1)
    compassInit();
#endif /* COMPASS_ENABLED */
#if (MOTOR_ENABLED == 1)
    motorInit();
#endif /* MOTOR_ENABLED */
#if (ENCODER_ENABLED == 1)
    encoderInit();
#endif /* ENCODER_ENABLED */

    bspWatchdogStart();
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
