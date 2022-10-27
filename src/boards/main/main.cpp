#include <atmel_start.h>
#include <HotFire.h>

#define TASK_EXAMPLE_STACK_SIZE (128*3 / sizeof(portSTACK_TYPE))
#define TASK_STATEMACHINE_STACK_SIZE (128*10 / sizeof(portSTACK_TYPE))
#define TASK_EXAMPLE_STACK_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_CAN_STACK_PRIORITY (tskIDLE_PRIORITY + 2)

static TaskHandle_t      xCanTask;
static TaskHandle_t      xStateMachineTask;

/**
 * CAN task
 *
 * \param[in] p The void pointer for OS task Standard model.
 *
 */
void can_task(void *p)
{
	uint8_t data[] = "Hello";
	for(;;) {
		CAN_0_sendMessage(data);
		CAN_0_readMessage();
		os_sleep(10);
	}
}

/**
 * STATE task
 *
 * \param[in] p The void pointer for OS task Standard model.
 *
 */
void StateMachine_task(void *p)
{
	HotFire();
	for(;;) {
		gpio_toggle_pin_level(LED);
		os_sleep(1000);
	}
}

int main()
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	usart_os_get_io(&USART_0, &ioU0);
	xTaskCreate(StateMachine_task, "STATE", TASK_STATEMACHINE_STACK_SIZE, NULL, TASK_EXAMPLE_STACK_PRIORITY, &xStateMachineTask);
	xTaskCreate(can_task, "CAN", TASK_EXAMPLE_STACK_SIZE, NULL, TASK_EXAMPLE_STACK_PRIORITY, &xCanTask);
	vTaskStartScheduler();
	while (1) {
		// Something has gone wrong if we end here.
		gpio_toggle_pin_level(LED);
		delay_ms(2000);
	}
}
