#include <atmel_start.h>

int main()
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	while (1) {
		FREERTOS_V1000_0_example();
	}
}
