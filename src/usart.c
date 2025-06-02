#include "stm32f7xx_hal.h"
#include "usart.h"

UART_HandleTypeDef huart6;

void UART1_Init(void) {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PC7 = USART6_TX = D0
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 1000000;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_HalfDuplex_Init(&huart6);
}

void UART1_Send(uint8_t *data, uint16_t len) {
    HAL_UART_Transmit(&huart6, data, len, HAL_MAX_DELAY);
}

int UART1_Read_Response(uint8_t *buffer, uint16_t len) {
    return (HAL_UART_Receive(&huart6, buffer, len, 50) == HAL_OK);
}
