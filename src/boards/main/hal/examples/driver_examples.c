/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

/* CRC Data in flash */
COMPILER_ALIGNED(4)
static const uint32_t crc_datas[] = {0x00000000,
                                     0x11111111,
                                     0x22222222,
                                     0x33333333,
                                     0x44444444,
                                     0x55555555,
                                     0x66666666,
                                     0x77777777,
                                     0x88888888,
                                     0x99999999};

/**
 * Example of using CRC_0 to Calculate CRC32 for a buffer.
 */
void CRC_0_example(void)
{
	/* The initial value used for the CRC32 calculation usually be 0xFFFFFFFF,
	 * but can be, for example, the result of a previous CRC32 calculation if
	 * generating a common CRC32 of separate memory blocks.
	 */
	uint32_t crc = 0xFFFFFFFF;
	uint32_t crc2;
	uint32_t ind;

	crc_sync_enable(&CRC_0);
	crc_sync_crc32(&CRC_0, (uint32_t *)crc_datas, 10, &crc);

	/* The read value must be complemented to match standard CRC32
	 * implementations or kept non-inverted if used as starting point for
	 * subsequent CRC32 calculations.
	 */
	crc ^= 0xFFFFFFFF;

	/* Calculate the same data with subsequent CRC32 calculations, the result
	 * should be same as previous way.
	 */
	crc2 = 0xFFFFFFFF;
	for (ind = 0; ind < 10; ind++) {
		crc_sync_crc32(&CRC_0, (uint32_t *)&crc_datas[ind], 1, &crc2);
	}
	crc2 ^= 0xFFFFFFFF;

	/* The calculate result should be same. */
	while (crc != crc2)
		;
}

/**
 * Example of using CALENDER_INTERFACE.
 */
static struct calendar_alarm alarm;

static void alarm_cb(struct calendar_descriptor *const descr)
{
	/* alarm expired */
}

void CALENDER_INTERFACE_example(void)
{
	struct calendar_date date;
	struct calendar_time time;

	calendar_enable(&CALENDER_INTERFACE);

	date.year  = 2000;
	date.month = 12;
	date.day   = 31;

	time.hour = 12;
	time.min  = 59;
	time.sec  = 59;

	calendar_set_date(&CALENDER_INTERFACE, &date);
	calendar_set_time(&CALENDER_INTERFACE, &time);

	alarm.cal_alarm.datetime.time.sec = 4;
	alarm.cal_alarm.option            = CALENDAR_ALARM_MATCH_SEC;
	alarm.cal_alarm.mode              = REPEAT;

	calendar_set_alarm(&CALENDER_INTERFACE, &alarm, alarm_cb);
}

/**
 * Example of using USART_0 to write "Hello World" using the IO abstraction.
 */
void USART_0_example(void)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&USART_0, &io);
	usart_sync_enable(&USART_0);

	io_write(io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using USART_1 to write "Hello World" using the IO abstraction.
 */
void USART_1_example(void)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&USART_1, &io);
	usart_sync_enable(&USART_1);

	io_write(io, (uint8_t *)"Hello World!", 12);
}

void delay_example(void)
{
	delay_ms(5000);
}

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
	return;
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

	msg.id  = 0x100000A5;
	msg.fmt = CAN_FMT_EXTID;
	/**
	 * remote device should recieve message with ID=0x100000A5
	 */
	can_async_write(&CAN_0, &msg);

	/**
	 * CAN_0_rx_callback callback should be invoked after call
	 * can_async_set_filter and remote device send CAN Message with the same
	 * content as the filter.
	 */
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	filter.id   = 0x469;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);

	filter.id   = 0x10000096;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);
}

void CAN_1_tx_callback(struct can_async_descriptor *const descr)
{
	(void)descr;
}
void CAN_1_rx_callback(struct can_async_descriptor *const descr)
{
	struct can_message msg;
	uint8_t            data[64];
	msg.data = data;
	can_async_read(descr, &msg);
	return;
}

/**
 * Example of using CAN_1 to Encrypt/Decrypt datas.
 */
void CAN_1_example(void)
{
	struct can_message msg;
	struct can_filter  filter;
	uint8_t            send_data[4];
	send_data[0] = 0x00;
	send_data[1] = 0x01;
	send_data[2] = 0x02;
	send_data[3] = 0x03;

	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_1, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_1_tx_callback);
	can_async_enable(&CAN_1);

	/**
	 * CAN_1_tx_callback callback should be invoked after call
	 * can_async_write, and remote device should recieve message with ID=0x45A
	 */
	can_async_write(&CAN_1, &msg);

	msg.id  = 0x100000A5;
	msg.fmt = CAN_FMT_EXTID;
	/**
	 * remote device should recieve message with ID=0x100000A5
	 */
	can_async_write(&CAN_1, &msg);

	/**
	 * CAN_1_rx_callback callback should be invoked after call
	 * can_async_set_filter and remote device send CAN Message with the same
	 * content as the filter.
	 */
	can_async_register_callback(&CAN_1, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_1_rx_callback);
	filter.id   = 0x469;
	filter.mask = 0;
	can_async_set_filter(&CAN_1, 0, CAN_FMT_STDID, &filter);

	filter.id   = 0x10000096;
	filter.mask = 0;
	can_async_set_filter(&CAN_1, 1, CAN_FMT_EXTID, &filter);
}
