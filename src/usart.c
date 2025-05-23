#include "usart.h"

UART_HandleTypeDef huart1;

void UART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA9 = USART1 TX (relié à Data AX-12)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 1000000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX; // requis pour half-duplex
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_HalfDuplex_Init(&huart1);  // ✅ Active 1 fil Data
}

void UART1_Send(uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(&huart1, data, size, HAL_MAX_DELAY);
}

uint8_t UART1_Read(void)
{
    uint8_t data;
    HAL_UART_Receive(&huart1, &data, 1, HAL_MAX_DELAY);
    return data;
}
