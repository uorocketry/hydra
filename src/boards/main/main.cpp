#include <atmel_start.h>


int main()
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&USART_0, &io);
	usart_sync_enable(&USART_0);

	/* Replace with your application code */
	while (1) {
		io_write(io, (uint8_t *)"Hello World!\r\n", 14);
		gpio_toggle_pin_level(LED);
		delay_ms(1000);
	}
}
