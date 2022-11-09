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

#include <hal_calendar.h>

#include <hal_usart_os.h>

#include <hal_usart_os.h>

#include <hal_usart_os.h>

#include <hal_mci_sync.h>

#define RADIO_BUFFER_SIZE 16

#define SBG_BUFFER_SIZE 16

#define COMPUTER_BUFFER_SIZE 16

extern struct calendar_descriptor CALENDAR_0;

extern struct usart_os_descriptor RADIO;
extern uint8_t                    RADIO_buffer[];

extern struct usart_os_descriptor SBG;
extern uint8_t                    SBG_buffer[];

extern struct usart_os_descriptor COMPUTER;
extern uint8_t                    COMPUTER_buffer[];

extern struct mci_sync_desc IO_BUS;

void CALENDAR_0_CLOCK_init(void);
void CALENDAR_0_init(void);

void RADIO_PORT_init(void);
void RADIO_CLOCK_init(void);
void RADIO_init(void);

void SBG_PORT_init(void);
void SBG_CLOCK_init(void);
void SBG_init(void);

void COMPUTER_PORT_init(void);
void COMPUTER_CLOCK_init(void);
void COMPUTER_init(void);

void IO_BUS_PORT_init(void);
void IO_BUS_CLOCK_init(void);
void IO_BUS_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
