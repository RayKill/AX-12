#ifndef USART_H
#define USART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void UART1_Init(void);
void UART1_Send(uint8_t *data, uint16_t len);
int  UART1_Read_Response(uint8_t *buffer, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
