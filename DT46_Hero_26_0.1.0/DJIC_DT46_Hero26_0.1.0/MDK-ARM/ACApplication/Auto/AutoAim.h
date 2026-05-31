#ifndef AUTOAIM_H_
#define AUTOAIM_H_

#include "A_MCommon.h"

/* 纯整数 int16_t 自瞄+手动融合函数（无浮点、最稳）*/
int16_t AutoAim_WeightFusion(int16_t manual, int16_t auto_val, uint8_t aim_valid, int16_t min_out, int16_t max_out);

#endif // AUTOAIM_H_