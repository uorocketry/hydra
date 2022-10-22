/*
 * RTOS task configuration 
 */
#include "atmel_start.h"
#include "rtos_start.h"

#define TASK_EXAMPLE_STACK_SIZE (128*3 / sizeof(portSTACK_TYPE))
#define TASK_EXAMPLE_STACK_PRIORITY (tskIDLE_PRIORITY + 1)
static TaskHandle_t      xCreatedExampleTask;
static SemaphoreHandle_t disp_mutex;

/**
 * CAN task
 *
 * \param[in] p The void pointer for OS task Standard model.
 *
 */
static void can_task(void *p)
{
	(void)p;
	while (1) {
		if (xSemaphoreTake(disp_mutex, ~0)) {
			/* add your code */
			uint8_t data[] = "Hello";
			CAN_0_sendMessage(data);
			CAN_0_readMessage();
			xSemaphoreGive(disp_mutex);
		}
		os_sleep(10);
	}
}

/**
 * LED task
 *
 * \param[in] p The void pointer for OS task Standard model.
 *
 */
static void led_task(void *p)
{
	(void)p;
	while (1) {
		if (xSemaphoreTake(disp_mutex, ~0)) {
			/* add your code */
			gpio_toggle_pin_level(LED);
			xSemaphoreGive(disp_mutex);
		}
		os_sleep(1000);
	}
}

/*
 * Example
 */
void FREERTOS_V1000_0_example(void)
{
	disp_mutex = xSemaphoreCreateMutex();

	if (disp_mutex == NULL) {
		while (1) {
			;
		}
	}

	if (xTaskCreate(
	        can_task, "Can", TASK_EXAMPLE_STACK_SIZE, NULL, TASK_EXAMPLE_STACK_PRIORITY, xCreatedExampleTask)
	    != pdPASS) {
		while (1) {
			;
		}
	}

	if (xTaskCreate(
	        led_task, "LED", TASK_EXAMPLE_STACK_SIZE, NULL, TASK_EXAMPLE_STACK_PRIORITY, xCreatedExampleTask)
	    != pdPASS) {
		while (1) {
			;
		}
	}

	vTaskStartScheduler();

	return;
}
