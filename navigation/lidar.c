/**
 * @file    lidar.c
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 *
 * @brief   Delta-2B laser radar receiver: USART1 RX over circular DMA, frame
 *          parser and polar histogram accumulation.
 *
 * @details The radar is simplex (it streams frames, no response needed). Frames
 *          are decoded by a byte-fed state machine and the measured distances
 *          are folded into a 360-sector polar histogram (minimum distance per
 *          1° sector). A completed revolution is detected when the per-frame
 *          start angle wraps 360°→0°, at which point the accumulated histogram
 *          is published to the readable scan buffer.
 *
 *          Frame layout (multi-byte fields are big-endian):
 *            Header(0xAA) Length(2) Version(0x00) Type(0x61) Command(1)
 *            ParamLength(2) Parameter(N) CheckCode(2)
 *          Length counts every byte up to (not including) the check code, i.e.
 *          Length = 8 + ParamLength. The check code is a Modbus CRC16 over the
 *          Length-covered bytes (a cumulative sum when the address code is
 *          nonzero).
 */

/******************************** Included files ******************************/
#include "RTE_Components.h"
#include CMSIS_device_header
#include "lidar.h"
#include <string.h>
/********************************* Definitions ********************************/

/**
 * @def LIDAR_FRAME_HEADER
 * @brief Fixed frame header byte.
 */
#define LIDAR_FRAME_HEADER          0xAAU

/**
 * @def LIDAR_FRAME_TYPE
 * @brief Fixed frame type byte.
 */
#define LIDAR_FRAME_TYPE            0x61U

/**
 * @def LIDAR_CMD_MEASUREMENT
 * @brief Command word for measurement information frames.
 */
#define LIDAR_CMD_MEASUREMENT       0xADU

/**
 * @def LIDAR_CMD_HEALTH
 * @brief Command word for device health (speed-fault) frames.
 */
#define LIDAR_CMD_HEALTH            0xAEU

/**
 * @def LIDAR_FRAME_OVERHEAD
 * @brief Bytes counted by the Length field before the parameter (Length = this
 *        value + ParamLength).
 */
#define LIDAR_FRAME_OVERHEAD        8U

/**
 * @def LIDAR_MEAS_HEADER_LEN
 * @brief Fixed parameter prefix of a measurement frame: speed(1) + offset(2) +
 *        start angle(2), before the repeating distance points.
 */
#define LIDAR_MEAS_HEADER_LEN       5U

/**
 * @def LIDAR_FRAME_SPAN_CDEG
 * @brief Angular span covered by one measurement frame, in centi-degrees (22.5°).
 */
#define LIDAR_FRAME_SPAN_CDEG       2250U

/**
 * @def LIDAR_WRAP_THRESHOLD_CDEG
 * @brief Minimum negative jump of the start angle (centi-degrees) treated as a
 *        full-revolution wrap rather than jitter.
 */
#define LIDAR_WRAP_THRESHOLD_CDEG   18000U

/**
 * @def LIDAR_DMA_BUF_SIZE
 * @brief Size of the circular DMA receive buffer in bytes.
 */
#define LIDAR_DMA_BUF_SIZE          512U

/**
 * @def LIDAR_PARAM_MAX
 * @brief Maximum accepted parameter length in bytes (caps points per frame).
 */
#define LIDAR_PARAM_MAX             256U

/**
 * @enum ELidarState
 * @brief Frame parser state machine states.
 */
typedef enum ELidarState {
    eLidarWaitHeader = 0,
    eLidarLengthHi,
    eLidarLengthLo,
    eLidarVersion,
    eLidarType,
    eLidarCommand,
    eLidarParamLenHi,
    eLidarParamLenLo,
    eLidarParam,
    eLidarCheckHi,
    eLidarCheckLo
} ELidarState_t;

/**
 * @struct SLidarParser
 * @brief Frame parser context.
 */
typedef struct SLidarParser {
    ELidarState_t state;                    /**< current parser state          */
    uint16_t      length;                   /**< Length field value            */
    uint16_t      param_len;                /**< ParamLength field value       */
    uint16_t      param_idx;                /**< parameter bytes collected     */
    uint16_t      crc;                      /**< running Modbus CRC16           */
    uint16_t      sum;                      /**< running cumulative byte sum    */
    uint8_t       proto;                    /**< address / prototype code       */
    uint16_t      rx_check;                 /**< received check code           */
    uint8_t       command;                  /**< command word                  */
    uint8_t       param[LIDAR_PARAM_MAX];   /**< parameter byte buffer         */
} SLidarParser_t;

/****************************** Module variables ******************************/

static uint8_t        dma_buffer[LIDAR_DMA_BUF_SIZE];
static uint16_t       dma_tail;
static SLidarParser_t parser;
static uint16_t       hist_fill[LIDAR_SECTOR_COUNT];
static uint16_t       hist_ready[LIDAR_SECTOR_COUNT];
static uint32_t       scan_count;
static uint16_t       last_start_cdeg;
static uint8_t        scan_initialized;
static uint8_t        radar_speed;
static uint8_t        health_fault;

/***************************** Private prototypes *****************************/

/** @brief Configures DMA1 Channel 5 for USART1 RX in circular mode. */
static void lidarDmaInit(void);

/** @brief Resets the frame parser to wait for a new header. */
static void lidarResetParser(void);

/**
 * @brief Updates the running frame checksums (CRC16 and sum) with one byte.
 * @param[in] b - byte to fold into the checksums.
 */
static void lidarChecksumByte(uint8_t b);

/**
 * @brief Returns the expected frame checksum for the current address code.
 * @returns Modbus CRC16 when the address code is 0, else the cumulative sum.
 */
static uint16_t lidarFrameChecksum(void);

/**
 * @brief Feeds one received byte into the frame parser state machine.
 * @param[in] b - received byte.
 */
static void lidarFeedByte(uint8_t b);

/** @brief Dispatches a fully received and checksum-verified frame. */
static void lidarHandleFrame(void);

/** @brief Parses a measurement frame into the polar histogram. */
static void lidarParseMeasurement(void);

/** @brief Publishes the accumulated histogram as a completed scan. */
static void lidarPublishScan(void);

/****************************** Private functions *****************************/

/** @fn lidarDmaInit */
static void lidarDmaInit(void) {
    /* Enable DMA1 controller clock */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* USART1_RX is mapped to DMA1 Channel 5 on STM32F103 */
    DMA1_Channel5->CCR   = 0U;                  /* disable while configuring */
    DMA1_Channel5->CPAR  = (uint32_t)(&USART1->DR);
    DMA1_Channel5->CMAR  = (uint32_t)dma_buffer;
    DMA1_Channel5->CNDTR = LIDAR_DMA_BUF_SIZE;
    DMA1_Channel5->CCR   = DMA_CCR1_MINC        /* memory increment */
                         | DMA_CCR1_CIRC        /* circular buffer  */
                         | DMA_CCR1_PL_0        /* medium priority  */
                         | DMA_CCR1_EN;         /* enable channel   */

    /* Route the USART1 receiver to DMA */
    USART1->CR3 |= USART_CR3_DMAR;

    dma_tail = 0U;
}
/*----------------------------------------------------------------------------*/

/** @fn lidarResetParser */
static void lidarResetParser(void) {
    parser.state     = eLidarWaitHeader;
    parser.param_idx = 0U;
    parser.crc       = 0xFFFFU;
    parser.sum       = 0U;
}
/*----------------------------------------------------------------------------*/

/** @fn lidarChecksumByte */
static void lidarChecksumByte(uint8_t b) {
    uint8_t i;

    parser.crc ^= (uint16_t)b;
    for (i = 0U; i < 8U; i++) {
        if ((parser.crc & 0x0001U) != 0U) {
            parser.crc = (uint16_t)((parser.crc >> 1U) ^ 0xA001U);
        }
        else {
            parser.crc = (uint16_t)(parser.crc >> 1U);
        }
    }
    parser.sum = (uint16_t)(parser.sum + b);
}
/*----------------------------------------------------------------------------*/

/** @fn lidarFrameChecksum */
static uint16_t lidarFrameChecksum(void) {
    uint16_t ret_val;

    if (parser.proto < 1U) {
        ret_val = parser.crc;
    }
    else {
        ret_val = parser.sum;
    }
    return ret_val;
}
/*----------------------------------------------------------------------------*/

/** @fn lidarFeedByte */
static void lidarFeedByte(uint8_t b) {
    switch (parser.state) {
    case eLidarWaitHeader:
        if (b == LIDAR_FRAME_HEADER) {
            parser.crc       = 0xFFFFU;
            parser.sum       = 0U;
            lidarChecksumByte(b);
            parser.param_idx = 0U;
            parser.state     = eLidarLengthHi;
        }
        break;

    case eLidarLengthHi:
        parser.length = (uint16_t)((uint16_t)b << 8U);
        lidarChecksumByte(b);
        parser.state  = eLidarLengthLo;
        break;

    case eLidarLengthLo:
        parser.length |= (uint16_t)b;
        lidarChecksumByte(b);
        parser.state   = eLidarVersion;
        break;

    case eLidarVersion:
        lidarChecksumByte(b);
        parser.proto = b;            /* address / prototype code */
        parser.state = eLidarType;
        break;

    case eLidarType:
        lidarChecksumByte(b);
        if (b == LIDAR_FRAME_TYPE) {
            parser.state = eLidarCommand;
        }
        else {
            parser.state = eLidarWaitHeader;
        }
        break;

    case eLidarCommand:
        lidarChecksumByte(b);
        parser.command = b;
        parser.state   = eLidarParamLenHi;
        break;

    case eLidarParamLenHi:
        parser.param_len = (uint16_t)((uint16_t)b << 8U);
        lidarChecksumByte(b);
        parser.state     = eLidarParamLenLo;
        break;

    case eLidarParamLenLo:
        parser.param_len |= (uint16_t)b;
        lidarChecksumByte(b);
        if (
            (parser.param_len <= LIDAR_PARAM_MAX)                       &&
            (parser.length    == (LIDAR_FRAME_OVERHEAD + parser.param_len))
        ) {
            if (parser.param_len == 0U) {
                parser.state = eLidarCheckHi;
            }
            else {
                parser.state = eLidarParam;
            }
        }
        else {
            parser.state = eLidarWaitHeader;
        }
        break;

    case eLidarParam:
        lidarChecksumByte(b);
        parser.param[parser.param_idx] = b;
        parser.param_idx++;
        if (parser.param_idx >= parser.param_len) {
            parser.state = eLidarCheckHi;
        }
        break;

    case eLidarCheckHi:
        parser.rx_check = (uint16_t)((uint16_t)b << 8U);
        parser.state    = eLidarCheckLo;
        break;

    case eLidarCheckLo:
        parser.rx_check |= (uint16_t)b;
        if (parser.rx_check == lidarFrameChecksum()) {
            lidarHandleFrame();
        }
        parser.state = eLidarWaitHeader;
        break;

    default:
        parser.state = eLidarWaitHeader;
        break;
    }
}
/*----------------------------------------------------------------------------*/

/** @fn lidarHandleFrame */
static void lidarHandleFrame(void) {
    switch (parser.command) {
    case LIDAR_CMD_MEASUREMENT:
        lidarParseMeasurement();
        break;

    case LIDAR_CMD_HEALTH:
        if (parser.param_len >= 1U) {
            radar_speed = parser.param[0];
        }
        health_fault = 1U;
        break;

    default:
        break;
    }
}
/*----------------------------------------------------------------------------*/

/** @fn lidarParseMeasurement */
static void lidarParseMeasurement(void) {
    uint16_t point_count;
    uint16_t start_cdeg;
    uint16_t i;
    uint16_t base;
    uint16_t dist_raw;
    uint16_t dist_mm;
    uint16_t sector;
    uint32_t angle_cdeg;

    if (parser.param_len >= LIDAR_MEAS_HEADER_LEN) {
        point_count = (uint16_t)((parser.param_len - LIDAR_MEAS_HEADER_LEN) / 3U);

        if (point_count != 0U) {
            radar_speed = parser.param[0];
            start_cdeg  = (uint16_t)(((uint16_t)parser.param[3] << 8U)
                                     | parser.param[4]);

            /* New revolution: start angle wrapped 360°→0°. */
            if (
                (scan_initialized != 0U)       &&
                (start_cdeg < last_start_cdeg) &&
                ((uint16_t)(last_start_cdeg - start_cdeg) >
                 LIDAR_WRAP_THRESHOLD_CDEG)
            ) {
                lidarPublishScan();
            }
            last_start_cdeg  = start_cdeg;
            scan_initialized = 1U;

            for (i = 0U; i < point_count; i++) {
                base     = (uint16_t)(LIDAR_MEAS_HEADER_LEN + (3U * i));
                dist_raw = (uint16_t)(((uint16_t)parser.param[base + 1U] << 8U)
                                      | parser.param[base + 2U]);

                if (dist_raw != 0U) {
                    dist_mm    = (uint16_t)(dist_raw >> 2U); /* 0.25mm/LSB */
                    angle_cdeg = (uint32_t)start_cdeg
                               + (((uint32_t)LIDAR_FRAME_SPAN_CDEG * (uint32_t)i)
                                  / (uint32_t)point_count);
                    sector     = (uint16_t)((angle_cdeg / 100U) % 360U);

                    if ((hist_fill[sector] == 0U) ||
                        (dist_mm < hist_fill[sector])) {
                        hist_fill[sector] = dist_mm;
                    }
                }
            }
        }
    }
}
/*----------------------------------------------------------------------------*/

/** @fn lidarPublishScan */
static void lidarPublishScan(void) {
    memcpy(hist_ready, hist_fill, sizeof(hist_ready));
    memset(hist_fill, 0, sizeof(hist_fill));
    scan_count++;
}

/********************* Application Programming Interface **********************/

/** @fn lidarInit */
void lidarInit(void) {
    lidarResetParser();
    memset(hist_fill, 0, sizeof(hist_fill));
    memset(hist_ready, 0, sizeof(hist_ready));
    scan_count       = 0U;
    last_start_cdeg  = 0U;
    scan_initialized = 0U;
    radar_speed      = 0U;
    health_fault     = 0U;
    lidarDmaInit();
}
/*----------------------------------------------------------------------------*/

/** @fn lidarProcess */
void lidarProcess(void) {
    uint16_t head;

    head = (uint16_t)(LIDAR_DMA_BUF_SIZE - DMA1_Channel5->CNDTR);
    while (dma_tail != head) {
        lidarFeedByte(dma_buffer[dma_tail]);
        dma_tail = (uint16_t)((dma_tail + 1U) % LIDAR_DMA_BUF_SIZE);
    }
}
/*----------------------------------------------------------------------------*/

/** @fn lidarGetScan */
const uint16_t *lidarGetScan(void) {
    return hist_ready;
}
/*----------------------------------------------------------------------------*/

/** @fn lidarGetScanCount */
uint32_t lidarGetScanCount(void) {
    return scan_count;
}
/*----------------------------------------------------------------------------*/

/** @fn lidarGetSpeedRaw */
uint8_t lidarGetSpeedRaw(void) {
    return radar_speed;
}
/*----------------------------------------------------------------------------*/

/** @fn lidarGetFault */
uint8_t lidarGetFault(void) {
    return health_fault;
}
/******************************************************************************/
