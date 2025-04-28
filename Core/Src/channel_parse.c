/*
 * ���ݸ�ͨ����ֵ���ж��Ĵ�����Ҫ�Ķ�����
 */

#include <stdio.h>
#include <stdlib.h>
#include "channel_parse.h"

// �������룬ѡ����Ҫ������
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

  // ����v1�ķ���
  if (v1 > 0) {
    return abs(v1) - abs(v2);
  } else {
    return abs(v2) - abs(v1);
  }
}

/*
 * ����0~11ͨ����ֵ�����Ƴ��˶�
 * 1050~1950ӳ�䵽-100~100�ϣ�1500��Ӧ0
 */
uint8_t parse_chan_val(uint16_t* ch_val)
{
  int8_t sp_fb = chg_val(ch_val[2]); // ǰ���������ǰ>0
  int8_t sp_lr = chg_val(ch_val[0]); // ���ҷ���������>0
  
  if (ch_val[6] == 1950) {
    // ��������ģʽ��������߲��˿���
    sp_lr = chg_val(ch_val[3]); // ���ҷ���������>0
  }
  
  if (ch_val[7] == 1950) {
    // ��������ģʽ���ٶȼ���
    sp_fb /= 2;
    sp_lr /= 2;   
  }
  
  if (sp_fb == 0) {
    if (sp_lr == 0) {
      // ֹͣ
      intf_move(0, 0);
    } else {
      // ԭ��ת��
      intf_move(sp_lr, (-1) * sp_lr);
    }
  } else {
    if (sp_lr == 0) {
      // ǰ�������
      intf_move(sp_fb, sp_fb);
    } else if (sp_lr > 0) {
      // ������ת
      intf_move(sp_fb, cal_offset(sp_fb, sp_lr));
    } else {
      // ������ת
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
