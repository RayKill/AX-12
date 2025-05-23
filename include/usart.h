#ifndef USART_H
#define USART_H

#include <stdint.h>
#include "stm32f7xx_hal.h"

void UART1_Init(void);
void UART1_Send(uint8_t *data, uint16_t size);
uint8_t UART1_Read(void);

#endif
