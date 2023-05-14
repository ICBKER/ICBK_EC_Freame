/**
  ****************************(C) COPYRIGHT 2023 ICBK****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.0.0     Apr-4-2023      Arthurlehao     2.添加注释
  *
  @verbatim
  ==============================================================================
    这一PID代码来自RM官方A型机器人开源代码
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 ICBK****************************
  */

#include "pid.h"
#include "main.h"

/*最大限幅处理*/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: 
  *                 PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: 
  *                 PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode; //选择普通PID算法还是差分PID算法
    
    pid->Kp = PID[0]; //P值 比例系数
    pid->Ki = PID[1]; //I值 积分系数
    pid->Kd = PID[2]; //D值 差分系数
    pid->max_out = max_out; //最大输出
    pid->max_iout = max_iout; //最大积分输出

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;  //微分项
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;  //误差项
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据 当前值
  * @param[in]      set: 设定值 目标值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    /*更新误差项*/
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    
    pid->set = set; //当前值
    pid->fdb = ref; //目标值
    
    pid->error[0] = set - ref;  //最新误差 = 当前值 - 目标值
    
    if (pid->mode == PID_POSITION) //普通PID算法
    {
        /*比例输出项和积分输出项计算*/
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        /*更新微分项*/
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        /*最新微分项= 当前误差 - 上一次误差*/
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
         /*计算微分输出项*/
        pid->Dout = pid->Kd * pid->Dbuf[0];
        
        /*限制积分项输出*/
        LimitMax(pid->Iout, pid->max_iout);
        
        /*计算总PID反馈输出*/
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        /*限制总PID反馈输出*/
        LimitMax(pid->out, pid->max_out);
        
        /*
          pid = kp * error + ki *error + kd * △error
        */
    }
    
    else if (pid->mode == PID_DELTA)  //差分PID算法
    {
         /*比例输出项和积分输出项计算*/
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        /*更新微分项*/
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        /*最新微分项= 当前误差 - 2*上一次误差 + 上上次误差*/
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        /*计算微分输出项*/
        pid->Dout = pid->Kd * pid->Dbuf[0];
        /*计算总PID反馈输出*/
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        /*限制总PID反馈输出*/
        LimitMax(pid->out, pid->max_out);
        
        /*
          pid = kp * △error + ki * error + kd * (error[0] - 2 * error[1] + error[2])  0当前 1上次 2上上次
        */
    }
   
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f; //清零误差项
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;  //清零微分项
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;  //清零各项输出
    pid->fdb = pid->set = 0.0f; //清零当前值与反馈值
}
