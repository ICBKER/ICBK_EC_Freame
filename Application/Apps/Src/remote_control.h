#ifndef REMOTE_CONTROL_H

#define REMOTE_CONTROL_H

#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 32u //缓冲区数据长度
#define RC_FRAME_LENGTH 18u //遥控器一帧数据长度

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024) //遥控器中值
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- 遥控器开关挡位定义----------------------------- */
//挡位映射
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
//判定挡位
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/* ----------------------- 遥控器数据结构------------------------------------- */
typedef __packed struct 
{
    __packed struct 
    {
        int16_t remote_channel[5];
        char switch_channel[2];

    }rc;

    __packed struct 
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_left;
        uint8_t press_right;
    }mouse;

    __packed struct 
    {
        uint16_t value;
    }keyboard;
    
}RC_ctrl_t;

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);

/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
extern const RC_ctrl_t *get_remote_control_point(void);

#endif
