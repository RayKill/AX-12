#ifndef INC_AX12_H_
#define INC_AX12_H_

#include "stm32f7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void ax12_ping(uint8_t id);
uint16_t ax12_read_position(uint8_t id);
void ax12_move_to_position(uint8_t id, uint16_t position);

#ifdef __cplusplus
}
#endif

#endif /* INC_AX12_H_ */
