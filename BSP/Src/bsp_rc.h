#ifndef BSP_RC_H
#define BSP_RC_H

#include "struct_typedef.h"

//定义外部函数 RC遥控初始化。
extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num); 

#endif
