#include <atmel_start.h>

int main()
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	/* Replace with your application code */
	while (1) {
		delay_ms(20);
		gpio_toggle_pin_level(LED);
	}
}
