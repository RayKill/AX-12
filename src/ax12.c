#include "ax12.h"
#include "usart.h"  // pour huart1

void ax12_send_packet(uint8_t* packet, uint8_t length) {
    HAL_UART_Transmit(&huart1, packet, length, HAL_MAX_DELAY);
    HAL_Delay(10);
}

uint8_t ax12_receive_response(uint8_t* buffer, uint8_t length) {
    return HAL_UART_Receive(&huart1, buffer, length, 100);
}

void ax12_ping(uint8_t id) {
    uint8_t packet[6] = {0xFF, 0xFF, id, 0x02, 0x01, ~(id + 2 + 0x01)};
    ax12_send_packet(packet, 6);
}

uint16_t ax12_read_position(uint8_t id) {
    uint8_t packet[8] = {
        0xFF, 0xFF, id, 0x04, 0x02, 0x24, 0x02,
        (uint8_t)(~(id + 4 + 0x02 + 0x24 + 0x02))
    };
    ax12_send_packet(packet, 8);

    uint8_t response[8] = {0};
    if (ax12_receive_response(response, 8) == HAL_OK) {
        uint8_t l = response[5];
        uint8_t h = response[6];
        return ((uint16_t)h << 8) | l;
    }
    return 0xFFFF;
}

void ax12_move_to_position(uint8_t id, uint16_t position) {
    uint8_t l = position & 0xFF;
    uint8_t h = (position >> 8) & 0xFF;
    uint8_t sum = id + 5 + 0x03 + 0x1E + l + h;
    uint8_t chk = ~sum;

    uint8_t packet[9] = {0xFF, 0xFF, id, 0x05, 0x03, 0x1E, l, h, chk};
    ax12_send_packet(packet, 9);
}
