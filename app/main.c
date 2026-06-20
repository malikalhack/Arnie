/**
 * @file    main.c
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 */

/******************************** Included files ******************************/
#include "lidar.h"

/********************* Application Programming Interface *********************/

/** @fn main */
int main(void) {
    lidarInit();

    for (;;) {
        lidarProcess();
    }
}
/******************************************************************************/
