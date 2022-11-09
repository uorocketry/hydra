#include <atmel_start.h>

#define TASK_EXAMPLE_STACK_SIZE (128*3 / sizeof(portSTACK_TYPE))
#define TASK_STATEMACHINE_STACK_SIZE (128*3 / sizeof(portSTACK_TYPE))
#define TASK_USART_STACK_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY + 2)

static TaskHandle_t      xLEDTask;
static TaskHandle_t      xUSARTTask;

/**
 * LED task
 *
 * \param[in] p The void pointer for OS task Standard model.
 *
 */
void led_task(void *p)
{
	for(;;) {
		gpio_toggle_pin_level(LED);
		os_sleep(500);
	}
	vTaskDelete(NULL);
}

/**
 * Example task of using COMPUTER to echo using the IO abstraction.
 */
void COMPUTER_example_task(void *p)
{
	struct io_descriptor *io;
	uint8_t data[] = "Hello World\r\n";

	(void)p;

	usart_os_get_io(&COMPUTER, &io);

	for(;;) {
		io->write(io, (uint8_t *)&data, sizeof(data));		
		os_sleep(1000);
	}
	vTaskDelete(NULL);
}

int main()
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	xTaskCreate(COMPUTER_example_task, "USART", TASK_EXAMPLE_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, &xUSARTTask);
	xTaskCreate(led_task, "LED", TASK_EXAMPLE_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, &xLEDTask);
	vTaskStartScheduler();
	while(1) {
		// Something has gone wrong if we end here.
	}
}
