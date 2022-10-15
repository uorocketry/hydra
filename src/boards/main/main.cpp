#include <atmel_start.h>

void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	USART_0_example();
	(void)descr;
}
void CAN_0_rx_callback(struct can_async_descriptor *const descr, io_descriptor *io)
{
	struct can_message msg;
	uint8_t            data[64];
	msg.data = data;
	can_async_read(descr, &msg);
	io_write(io, msg.data, msg.len);
	// io_write(io, msg.data, msg.len);
	io_write(io, (uint8_t *)"\r\n", 2);
	// USART_0_example();
	return;
}

int main()
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&USART_0, &io);
	usart_sync_enable(&USART_0);
	struct can_message msg;
	struct can_filter  filter;
	uint8_t            send_data[4];
	send_data[0] = 0x30;
	send_data[1] = 0x32;
	send_data[2] = 0x32;
	send_data[3] = 0x33;
	// USART_0_example();
	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);
	can_async_enable(&CAN_0);
	/* Replace with your application code */
	while (1) {
		/**
		 * CAN_0_tx_callback callback should be invoked after call
		 * can_async_write, and remote device should recieve message with ID=0x45A
		 */
		can_async_write(&CAN_0, &msg);

		// msg.id  = 0x100000A5;
		// msg.fmt = CAN_FMT_EXTID;
		// /**
		//  * remote device should recieve message with ID=0x100000A5
		//  */
		// can_async_write(&CAN_0, &msg);

		/**
		 * CAN_0_rx_callback callback should be invoked after call
		 * can_async_set_filter and remote device send CAN Message with the same
		 * content as the filter.
		 */

		can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
		filter.id   = 0x45A;
		filter.mask = 0;
		can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
		// filter.id   = 0x100000A5;
		// filter.mask = 0;
		// can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);
		CAN_0_rx_callback(&CAN_0, io);
		gpio_toggle_pin_level(LED);
		delay_ms(1000);
	}
}
