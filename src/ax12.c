#include "ax12.h"
#include "usart.h"

void ax12_ping(uint8_t id) {
    uint8_t buf[6];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 0x02;
    buf[4] = 0x01; // PING
    buf[5] = ~(id + 0x02 + 0x01);

    UART1_Send(buf, 6);
}

void ax12_move_to_position(uint8_t id, uint16_t pos) {
    uint8_t low = pos & 0xFF;
    uint8_t high = (pos >> 8) & 0xFF;
    uint8_t length = 5;
    uint8_t instruction = 0x03;
    uint8_t address = 0x1E;

    uint8_t checksum = ~(id + length + instruction + address + low + high);

    uint8_t buf[11] = {
        0xFF, 0xFF,
        id,
        length,
        instruction,
        address,
        low,
        high,
        checksum
    };

    UART1_Send(buf, 9);
}
