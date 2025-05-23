#include "ax12.h"
#include "usart.h"

void ax12_ping(uint8_t id) {
    uint8_t buf[6];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 0x02;  // length
    buf[4] = 0x01;  // instruction: PING
    buf[5] = ~(id + 0x02 + 0x01);

    UART1_Send(buf, 6);
}

uint16_t ax12_read_position(uint8_t id) {
    uint8_t buf[8];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 0x04;
    buf[4] = 0x02;      // READ
    buf[5] = 0x24;      // Starting at address 0x24
    buf[6] = 0x02;      // 2 bytes
    buf[7] = ~(id + 0x04 + 0x02 + 0x24 + 0x02);

    UART1_Send(buf, 8);
    HAL_Delay(10);

    // lecture simplifiée (sans vérif)
    UART1_Read(); // 0xFF
    UART1_Read(); // 0xFF
    UART1_Read(); // ID
    UART1_Read(); // LENGTH
    UART1_Read(); // ERROR
    uint8_t low = UART1_Read();
    uint8_t high = UART1_Read();

    return (high << 8) | low;
}

void ax12_move_to_position(uint8_t id, uint16_t pos) {
    uint8_t low = pos & 0xFF;
    uint8_t high = (pos >> 8) & 0xFF;
    uint8_t length = 5;
    uint8_t checksum = ~(id + length + 0x03 + 0x1E + low + high);

    uint8_t buf[9] = {
        0xFF, 0xFF, id,
        length, 0x03, 0x1E,
        low, high, checksum
    };

    UART1_Send(buf, 9);
}
