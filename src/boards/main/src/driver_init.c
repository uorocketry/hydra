/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

/* The priority of the peripheral should be between the low and high interrupt priority set by chosen RTOS,
 * Otherwise, some of the RTOS APIs may fail to work inside interrupts
 * In case of FreeRTOS,the Lowest Interrupt priority is set by configLIBRARY_LOWEST_INTERRUPT_PRIORITY and
 * Maximum interrupt priority by configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, So Interrupt Priority of the peripheral
 * should be between configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY and configLIBRARY_LOWEST_INTERRUPT_PRIORITY
 */
#define PERIPHERAL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY - 1)

struct crc_sync_descriptor  CRC_0;
struct can_async_descriptor CAN_0;
struct can_async_descriptor CAN_1;

struct calendar_descriptor CALENDAR_0;

struct usart_os_descriptor USART_1;
uint8_t                    USART_1_buffer[USART_1_BUFFER_SIZE];

struct usart_os_descriptor USART_0;
uint8_t                    USART_0_buffer[USART_0_BUFFER_SIZE];
struct io_descriptor *ioU0;

struct mci_sync_desc IO_BUS;

/**
 * \brief CRC initialization function
 *
 * Enables CRC peripheral, clocks and initializes CRC driver
 */
void CRC_0_init(void)
{
	hri_mclk_set_APBBMASK_DSU_bit(MCLK);
	crc_sync_init(&CRC_0, DSU);
}

void CALENDAR_0_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_RTC_bit(MCLK);
}

void CALENDAR_0_init(void)
{
	CALENDAR_0_CLOCK_init();
	calendar_init(&CALENDAR_0, RTC);
}

void USART_1_PORT_init(void)
{

	gpio_set_pin_function(PA16, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_function(PA17, PINMUX_PA17C_SERCOM1_PAD1);
}

void USART_1_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBAMASK_SERCOM1_bit(MCLK);
}

void USART_1_init(void)
{

	USART_1_CLOCK_init();
	uint32_t irq = SERCOM1_0_IRQn;
	for (uint32_t i = 0; i < 4; i++) {
		NVIC_SetPriority((IRQn_Type)irq, PERIPHERAL_INTERRUPT_PRIORITY);
		irq++;
	}
	usart_os_init(&USART_1, SERCOM1, USART_1_buffer, USART_1_BUFFER_SIZE, (void *)NULL);
	usart_os_enable(&USART_1);
	USART_1_PORT_init();
}

void USART_0_PORT_init(void)
{

	gpio_set_pin_function(PB16, PINMUX_PB16C_SERCOM5_PAD0);

	gpio_set_pin_function(PB17, PINMUX_PB17C_SERCOM5_PAD1);
}

void USART_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBDMASK_SERCOM5_bit(MCLK);
}

void USART_0_init(void)
{

	USART_0_CLOCK_init();
	uint32_t irq = SERCOM5_0_IRQn;
	for (uint32_t i = 0; i < 4; i++) {
		NVIC_SetPriority((IRQn_Type)irq, PERIPHERAL_INTERRUPT_PRIORITY);
		irq++;
	}
	usart_os_init(&USART_0, SERCOM5, USART_0_buffer, USART_0_BUFFER_SIZE, (void *)NULL);
	usart_os_enable(&USART_0);
	USART_0_PORT_init();
}

void IO_BUS_PORT_init(void)
{

	gpio_set_pin_direction(PB11,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PB11,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PB11,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB11,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PB11I_SDHC0_SDCK"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PB11I_SDHC0_SDCK);

	gpio_set_pin_direction(PA08,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA08,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA08,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA08,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA08I_SDHC0_SDCMD"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA08I_SDHC0_SDCMD);

	gpio_set_pin_direction(PA09,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA09,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA09,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA09,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA09I_SDHC0_SDDAT0"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA09I_SDHC0_SDDAT0);

	gpio_set_pin_direction(PA10,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA10,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA10,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA10,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA10I_SDHC0_SDDAT1"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA10I_SDHC0_SDDAT1);

	gpio_set_pin_direction(PA11,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA11,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA11,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA11,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA11I_SDHC0_SDDAT2"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA11I_SDHC0_SDDAT2);

	gpio_set_pin_direction(PB10,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PB10,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PB10,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB10,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PB10I_SDHC0_SDDAT3"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PB10I_SDHC0_SDDAT3);
}

void IO_BUS_CLOCK_init(void)
{
	hri_mclk_set_AHBMASK_SDHC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC0_GCLK_ID, CONF_GCLK_SDHC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC0_GCLK_ID_SLOW, CONF_GCLK_SDHC0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void IO_BUS_init(void)
{
	IO_BUS_CLOCK_init();
	mci_sync_init(&IO_BUS, SDHC0);
	IO_BUS_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

void CAN_0_PORT_init(void)
{

	gpio_set_pin_function(PA23, PINMUX_PA23I_CAN0_RX);

	gpio_set_pin_function(PA22, PINMUX_PA22I_CAN0_TX);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_0_init(void)
{
	// !! You'll need to configure the bit rate based on the CAN clock.
	// We need to set CCCR.Test to 1 to enable test modes and operation.

	hri_mclk_set_AHBMASK_CAN0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, CONF_GCLK_CAN0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_can_write_CCCR_TEST_bit(CAN0, true);
	hri_can_write_TEST_LBCK_bit(CAN0, true); // should be the same as toggle.
	can_async_init(&CAN_0, CAN0);
	CAN_0_PORT_init();
}

void CAN_1_PORT_init(void)
{

	gpio_set_pin_function(PB13, PINMUX_PB13H_CAN1_RX);

	gpio_set_pin_function(PB12, PINMUX_PB12H_CAN1_TX);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_1_init(void)
{
	hri_mclk_set_AHBMASK_CAN1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN1_GCLK_ID, CONF_GCLK_CAN1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_1, CAN1);
	CAN_1_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA14

	gpio_set_pin_level(LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED, GPIO_PIN_FUNCTION_OFF);

	CRC_0_init();

	CALENDAR_0_init();

	USART_1_init();

	USART_0_init();

	IO_BUS_init();

	delay_driver_init();
	CAN_0_init();
	CAN_1_init();
}
