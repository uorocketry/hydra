/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_crc_sync.h>

#include <hal_calendar.h>

#include <hal_usart_os.h>

#include <hal_usart_os.h>

#include <hal_mci_sync.h>

#include <hal_delay.h>
#include <hal_can_async.h>
#include <hal_can_async.h>

#define USART_1_BUFFER_SIZE 16

#define USART_0_BUFFER_SIZE 16

extern struct crc_sync_descriptor CRC_0;

extern struct calendar_descriptor CALENDAR_0;

extern struct usart_os_descriptor USART_1;
extern uint8_t                    USART_1_buffer[];

extern struct usart_os_descriptor USART_0;
extern uint8_t                    USART_0_buffer[];
extern struct io_descriptor *ioU0;

extern struct mci_sync_desc IO_BUS;

extern struct can_async_descriptor CAN_0;
extern struct can_async_descriptor CAN_1;

void CALENDAR_0_CLOCK_init(void);
void CALENDAR_0_init(void);

void USART_1_PORT_init(void);
void USART_1_CLOCK_init(void);
void USART_1_init(void);

void USART_0_PORT_init(void);
void USART_0_CLOCK_init(void);
void USART_0_init(void);

void IO_BUS_PORT_init(void);
void IO_BUS_CLOCK_init(void);
void IO_BUS_init(void);

void delay_driver_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
