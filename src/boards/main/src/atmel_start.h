#ifndef ATMEL_START_H_INCLUDED
#define ATMEL_START_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "driver_init.h"
#include "rtos_start.h"
#include "sd_mmc_start.h"
#include "diskio_start.h"
#include <fdcan.h>

/**
 * Initializes MCU, drivers and middleware in the project
 **/
void atmel_start_init(void);

#ifdef __cplusplus
}
#endif
#endif
