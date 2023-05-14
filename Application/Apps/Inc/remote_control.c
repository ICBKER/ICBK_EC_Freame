/**
  ****************************(C) COPYRIGHT 2023 ICBK****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输。
  *             利用DMA传输方式节约CPU资源，利用串口空闲中断来拉起处理函数。
  *             同时提供一些掉线重启DMA，串口的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *  V1.0.1     Mar-27-2023     Arthurlehao     2. 本地自有化
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ICBK****************************
  */

#include "remote_control.h"
#include "main.h"

extern  UART_HandleTypeDef huart3;
extern  DMA_HandleTypeDef hdma_usart3_rx;

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
 static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

 //遥控器控制变量
RC_ctrl_t rc_ctrl;

//获取遥控器数据指针
const RC_ctrl_t *get_remote_control_point(void)
{
  return &rc_ctrl;
}

//定义原始数据缓冲区，接收原始数据，为18个字节的数据，这里给了36个字节的长度，防止DMA传输越界。
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

//遥控器初始化
void remote_control_init(void)
{
  RC_init(sbus_rx_buf[0],sbus_rx_buf[1],SBUS_RX_BUF_NUM);
}

//串口中断
void USART3_IRQHandler(void)
{
  if(huart3.Instance->SR & UART_FLAG_RXNE)// 收到数据
  {
    __HAL_UART_CLEAR_PEFLAG(&huart3);    
  }
  else if(USART3->SR & UART_FLAG_IDLE)
  {
    static uint16_t this_time_rx_len = 0;
    
    __HAL_UART_CLEAR_PEFLAG(&huart3); 

    //判断是0号缓冲区 还是1号缓冲区
    if((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
      /*这里使用缓冲区0*/
      //失效DMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);
      //获取接收数据长度 接收数据长度=设定长度-剩余长度
      this_time_rx_len =  SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
      //重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
      //设定缓冲区1
      hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
      //使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      //处理遥控器数据 如果数据等于一帧数据长度 就进行遥控器数据解析
      if(this_time_rx_len == RC_FRAME_LENGTH)
      {
        sbus_to_rc(sbus_rx_buf[0],&rc_ctrl);
      }
    }
    else
    {
      /*这里使用缓冲区1*/
      //失效DMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);
      //获取接收数据长度 接收数据长度=设定长度-剩余长度
      this_time_rx_len =  SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
      //重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
      //设定缓冲区0
      hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_CT;
      //使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);
      
      //处理遥控器数据 如果数据等于一帧数据长度 就进行遥控器数据解析
      if(this_time_rx_len == RC_FRAME_LENGTH)
      {
        sbus_to_rc(sbus_rx_buf[1],&rc_ctrl);
      }
    }
  }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
 static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
  if(sbus_rx_buf == NULL | rc_ctrl == NULL)
  {
    return;
  }
	//遥感通道
  rc_ctrl->rc.remote_channel[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
  rc_ctrl->rc.remote_channel[1] =((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
  rc_ctrl->rc.remote_channel[2] =((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;
  rc_ctrl->rc.remote_channel[3] =((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
  //波轮通道
	rc_ctrl->rc.remote_channel[4] =sbus_buf[16] | (sbus_buf[17] << 8);
	
  //开关通道
  rc_ctrl->rc.switch_channel[0] = ((sbus_buf[5] >> 4) & 0x0003);   
  rc_ctrl->rc.switch_channel[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; 
	//鼠标移动
  rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
  rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); 
  rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8); 
	//鼠标左右键
  rc_ctrl->mouse.press_left =  sbus_buf[12];
  rc_ctrl->mouse.press_right = sbus_buf[13]; 
  //键盘数据
  rc_ctrl->keyboard.value =sbus_buf[14] | (sbus_buf[15] << 8); 
  
  rc_ctrl->rc.remote_channel[0] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.remote_channel[1] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.remote_channel[2] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.remote_channel[3] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.remote_channel[4] -= RC_CH_VALUE_OFFSET;

}
