/*
 * Implementation of FDCAN functions.
 */
#ifndef USART_H
#define USART_H

#include <driver_init.h>

#ifdef __cplusplus
extern "C" {
#endif

void USART_0_example(void);
void USART_0_write(uint8_t data[], uint8_t len);

#ifdef __cplusplus
}
#endif
#endif // USART_H
