The USART RTOS Driver
=====================

The universal synchronous and asynchronous receiver and transmitter
(USART) is usually used to transfer data from one device to the other.

The transfer functions of USART RTOS driver are optimized for RTOS support.
That is, the transfer functions will not work without RTOS, the transfer
functions should only be called in a RTOS task or thread.

The USART RTOS driver use a ring buffer to store received data. When the USART
raise the data received interrupt, this data will be stored in the ring buffer
at the next free location. When the ring buffer is full, the next reception
will overwrite the oldest data stored in the ring buffer. When initialize the
driver, the ring buffer must be allocated and passed to driver to use. The
size of the buffer must be the power of two, e.g., 32 or 64. When read data
through USART RTOS API, If the number of bytes asked for are more than currently
available in the ring buffer or more than ringbuf size, the task/thread will be
blocked until read done.If the number of bytes asked for is less than the available
bytes in the ring buffer, the remaining bytes will be kept until a new call.

On the other hand, when sending data over USART, the data is not copied to an
internal buffer, but the data buffer supplied by the user is used. Then the
task/thread will be blocked to wait until all data is sent.

During data transfer, the USART TX/RX process is not protected, so that a more
flexible way can be chosen in application.

User can set action for flow control pins by driver API if the flow control is
enabled. All the available states are defined in union
usart_flow_control_state.

Note that user can set state of flow control pins only if automatic support of
the flow control is not supported by the hardware.

Features
--------

* Initialization/de-initialization
* Enabling/disabling
* Control of the following settings:

  * Baudrate
  * UART or USRT communication mode
  * Character size
  * Data order
  * Flow control
* Data transfer: transmission, reception

Applications
------------

They are commonly used in a terminal application or low-speed communication
between devices.

Dependencies
------------

* USART capable hardware, with interrupt on each character sent or received.
* RTOS

Concurrency
-----------

Limitations
-----------

The driver does not support 9-bit character size.

Known issues and workarounds
----------------------------

N/A