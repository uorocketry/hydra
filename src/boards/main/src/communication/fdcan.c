#include <fdcan.h>

void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	(void)descr;
}
void CAN_0_rx_callback(struct can_async_descriptor *const descr)
{
	struct can_message msg;
	uint8_t            data[64];
	msg.data = data;
	can_async_read(descr, &msg);
	ioU0->write(ioU0, (uint8_t *)&msg.data, msg.len);
	ioU0->write(ioU0, (uint8_t *)"\r\n", 2);
	return;
}

void CAN_0_sendMessage(uint8_t data[]) {
	struct can_message msg;
	// USART_0_example();
	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = data;
	msg.len  = sizeof(data);
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);
	can_async_enable(&CAN_0);

	/**
	 * CAN_0_tx_callback callback should be invoked after call
	 * can_async_write, and remote device should recieve message with ID=0x45A
	 */
	can_async_write(&CAN_0, &msg);
}

void CAN_0_readMessage(void) {
	struct can_filter  filter;
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
	CAN_0_rx_callback(&CAN_0);
}

/**
 * Example of using CAN_0 to Encrypt/Decrypt datas.
 */
void CAN_0_example(void)
{
	struct can_message msg;
	struct can_filter  filter;
	uint8_t            send_data[4];
	send_data[0] = 0x00;
	send_data[1] = 0x01;
	send_data[2] = 0x02;
	send_data[3] = 0x03;
	// USART_0_example();
	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);
	can_async_enable(&CAN_0);

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
	CAN_0_rx_callback(&CAN_0);
}
