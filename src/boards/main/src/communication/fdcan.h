/*
 * Implementation of FDCAN functions
 */

#ifndef FDCAN_H
#define FDCAN_H

#include <driver_init.h>
#include <usart.h>

#ifdef __cplusplus
extern "C" {
#endif

void CAN_0_sendMessage(uint8_t data[]);
void CAN_0_readMessage(void);
void CAN_0_example(void);

#ifdef __cplusplus
}
#endif
#endif // FDCAN_H
