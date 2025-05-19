#ifndef AX12_H
#define AX12_H

#include <stdint.h>

void ax12_ping(uint8_t id);
uint16_t ax12_read_position(uint8_t id);
void ax12_move_to_position(uint8_t id, uint16_t pos);

#endif
