/**
 * @file indev.h
 *
 */

#ifndef INDEV_H
#define INDEV_H

/*********************
 *      INCLUDES
 *********************/
#include "mbed.h"
#include <stdbool.h>
#include <stdint.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
//void touchpad_init(void);
#ifdef __cplusplus
extern "C"
{
#endif

void touchpad_init(void);

#ifdef __cplusplus
}
#endif

/**********************
 *      MACROS
 **********************/

#endif
