/**
 * @file    acrosched_defs.h
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    2026-05-14
 * @date    @showdate "%Y-%m-%d"
 */

#ifndef ACROSCHED_DEFS_H_
#define ACROSCHED_DEFS_H_

/******************************** Included files ******************************/
#include <stdint.h>
#include <stddef.h>

/********************************* Definitions ********************************/

/**
 * @def BIT
 * @brief Produce a bitmask with bit @p offset set (32-bit).
 */
#define BIT(offset)             (1UL << (offset))

/**
 * @def BIT64
 * @brief Produce a bitmask with bit @p offset set (64-bit).
 */
#define BIT64(offset)           (1ULL << (offset))

/*----------------------------------------------------------------------------*/

/**
 * @def UNUSED
 * @brief Suppress unused-parameter / unused-variable compiler warnings.
 */
#define UNUSED(x)               ((void)(x))

/**
 * @def IGNORE_RETURN
 * @brief Explicitly discard a function's return value.
 * @details Use when a return value is intentionally not checked.
 * Makes the intent clear and suppresses compiler warnings.
 */
#define IGNORE_RETURN(x)        ((void)(x))

/*----------------------------------------------------------------------------*/

/**
 * @def GET_LOW_BYTE
 * @brief Extract the low byte (bits 7..0) of an integer value.
 */
#define GET_LOW_BYTE(val)       ((uint8_t)(val))

/**
 * @def GET_HIGH_BYTE
 * @brief Extract the high byte (bits 15..8) of an integer value.
 */
#define GET_HIGH_BYTE(val)      ((uint8_t)((val) >> 8))

/**
 * @def WORD_FROM_BYTES
 * @brief Combine a high byte and a low byte into a 16-bit word.
 * @param hi  High byte.
 * @param lo  Low byte.
 */
#define WORD_FROM_BYTES(hi, lo) \
    ((uint16_t)(((uint16_t)(hi) << 8) | (uint8_t)(lo)))

/*----------------------------------------------------------------------------*/

/**
 * @def SIZEOF_ARRAY
 * @brief Evaluate the number of elements in a statically allocated array.
 * @note  Do not use with pointers - result will be meaningless.
 */
#define SIZEOF_ARRAY(arr)       (sizeof(arr) / sizeof((arr)[0]))

/*----------------------------------------------------------------------------*/

/**
 * @def ACRO_MIN
 * @brief Return the smaller of two values.
 * @warning Arguments are evaluated twice - avoid side effects.
 */
#define ACRO_MIN(a, b)          (((a) < (b)) ? (a) : (b))

/**
 * @def ACRO_MAX
 * @brief Return the larger of two values.
 * @warning Arguments are evaluated twice - avoid side effects.
 */
#define ACRO_MAX(a, b)          (((a) > (b)) ? (a) : (b))

/******************************************************************************/
#endif //! ACROSCHED_DEFS_H_
