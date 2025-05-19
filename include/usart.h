#ifndef USART_H
#define USART_H

#include <stdint.h>

void UART1_Init(void);
void UART1_Send(uint8_t *data, uint16_t len);
uint8_t UART1_Receive(uint8_t *buffer, uint16_t len);

#endif
