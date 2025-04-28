#ifndef __DRV_SHB_H
#define __DRV_SHB_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"

extern uint8_t intf_move_shb(int8_t speed_left, int8_t speed_right);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SHB_H */
