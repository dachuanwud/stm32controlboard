#ifndef __CHANNEL_PARSE_H
#define __CHANNEL_PARSE_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"

extern uint8_t parse_chan_val(uint16_t* ch_val);
extern uint8_t parse_cmd_vel(uint8_t spl, uint8_t spr);

#ifdef __cplusplus
}
#endif

#endif /* __CHANNEL_PARSE_H */
