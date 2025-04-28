#ifndef __DRV_KEYA_H
#define __DRV_KEYA_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"

extern uint8_t intf_move_keya(int8_t speed_left, int8_t speed_right);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_KEYA_H */
