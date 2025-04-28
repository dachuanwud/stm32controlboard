/*
 * 根据各通道的值，判断履带车需要的动作。
 */

#include <stdio.h>
#include <stdlib.h>
#include "channel_parse.h"

// 条件编译，选择需要的驱动
// #define DRV_TOY
// #define DRV_SHB
#define DRV_KEYADOUBLE

#if defined DRV_TOY
  #include "drv_toy.h"
  uint8_t (*intf_move)(int8_t, int8_t) = intf_move_toy;
#elif defined DRV_SHB
  #include "drv_shb.h"
  uint8_t (*intf_move)(int8_t, int8_t) = intf_move_shb;
#elif defined DRV_KEYA
  #include "drv_keya.h"
  uint8_t (*intf_move)(int8_t, int8_t) = intf_move_keya;
#elif defined DRV_KEYADOUBLE
  #include "drv_keyadouble.h"
  uint8_t (*intf_move)(int8_t, int8_t) = intf_move_keyadouble;
#else
  #include "drv_real.h"
  uint8_t (*intf_move)(int8_t, int8_t) = intf_move_real;
#endif


static int8_t chg_val(uint16_t val)
{
  int8_t sp = (((int16_t)val - 1500) / 9 * 2) & 0xff;
  return sp;
}

static int8_t cal_offset(int8_t v1, int8_t v2)
{
  if (abs(v1) < abs(v2)) {
    return 0;
  }

  // 带上v1的符号
  if (v1 > 0) {
    return abs(v1) - abs(v2);
  } else {
    return abs(v2) - abs(v1);
  }
}

/*
 * 解析0~11通道的值，控制车运动
 * 1050~1950映射到-100~100上，1500对应0
 */
uint8_t parse_chan_val(uint16_t* ch_val)
{
  int8_t sp_fb = chg_val(ch_val[2]); // 前后分量，向前>0
  int8_t sp_lr = chg_val(ch_val[0]); // 左右分量，向右>0
  
  if (ch_val[6] == 1950) {
    // 启动单手模式，仅用左边拨杆控制
    sp_lr = chg_val(ch_val[3]); // 左右分量，向右>0
  }
  
  if (ch_val[7] == 1950) {
    // 启动低速模式，速度减半
    sp_fb /= 2;
    sp_lr /= 2;   
  }
  
  if (sp_fb == 0) {
    if (sp_lr == 0) {
      // 停止
      intf_move(0, 0);
    } else {
      // 原地转向
      intf_move(sp_lr, (-1) * sp_lr);
    }
  } else {
    if (sp_lr == 0) {
      // 前进或后退
      intf_move(sp_fb, sp_fb);
    } else if (sp_lr > 0) {
      // 差速右转
      intf_move(sp_fb, cal_offset(sp_fb, sp_lr));
    } else {
      // 差速左转
      intf_move(cal_offset(sp_fb, sp_lr), sp_fb);
    } 
  } 
  return 0;  
}

uint8_t parse_cmd_vel(uint8_t spl, uint8_t spr)
{
  intf_move((int8_t)spl, (int8_t)spr);
  return 0; 
}
