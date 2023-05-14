/**
  ****************************(C) COPYRIGHT 2023 ICBK****************************
  * @file      	chassis_task.c/h
  * @brief      四轮全向轮底盘控制任务
  * @note       freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V0.0.1     Mar-27-2023     Arthurlehao     Done
  @verbatim
  ==============================================================================
  底盘电机ID顺序 45度角四轮:
                             前
                          1-------0
                             | |    
                          2-------3
                              后
  代码及结构参考：
    RM官方A型步兵开源代码
    深圳大学Pilot战队2021英雄代码开源
    杭州电子科技大学全向轮开源代码
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ICBK****************************
  */
 /*
  .............................................................................................................. 
  .......................................................   ............  ...................................... 
  ...................................................           .......       .................................. 
  ...............................................        ...       . ..  .       ............................... 
  ...........................................        ...........    ...  .....   ............................... 
  ........................................       ......       ....  ...  ......  ............................... 
  ....................................       ......                 ...  ......  ............................... 
  ..................................      .....        .......      ...  ......  ............................... 
  ..................................  ......       ............... . ..  ......  ............................... 
  ..................................  ......   .......................   ......   .............................. 
  ..................................  ......  .....................     ......   ............................... 
  ..................................  ......  ...................     .. ..     ................................ 
  ..................................  ......  ................     ......     .................................. 
  ..................................  ......  ..............     .. ...     ..... .............................. 
  ..................................  ......  ............    ......     .....   ............................... 
  ..................................  ......  .........     ......     .....     ............................... 
  ..................................  .....   .......    ......     .....     .  ............................... 
  ..................................  ..     .....     ......     .....     ..   ............................... 
  ..................................       .....     .....     ........  ......  ............................... 
  ..................................    .....     ......     ..........  ......  ............................... 
  ................................... .....     .....     .............  ......  ............................... 
  ......................................     ......     ...............  ......  ............................... 
  ....................................     ......    ..................  ......  ............................... 
  ..................................    ... ..     ....................  ......  ............................... 
  ..................................  ......     ......................  ......  ............................... 
  .................................   ......  ......................     ......   .............................. 
  ..................................  ......  ...    ...........        ......   ............................... 
  ..................................  ......  ...        ...        ......       ............................... 
  ..................................  ......  ...  ..           ......        .................................. 
  .................................   ......  ...  ......   ......        ...................................... 
  ..................................    ....  ...      .......        .......................................... 
  ...................................         .. ..               .............................................. 
  ......................................      .........       .................................................. 
  .............................................................................................................. 
*/

/*------头文件嵌入------*/
#include "remote_control.h"
#include "CAN_receive.h"
#include "pid.h"

#include "chassis_task.h"
/*-------宏定义-------*/

/*------函数声明------*/

//底盘初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//遥控器模式选择
static void chassis_mode_choose(chassis_move_t *chassis_move_mode_choose);
//控制模式设定
static void chassis_mode_set(chassis_move_t *chassis_move_mode_set);
//控制量计算
static void chassis_control_cal(chassis_move_t *chassis_move_control_cal);
//全向轮运动解算
static void chassis_vector_to_omni_wheel_speed(chassis_move_t *chassis_vector_to_motor_speed);

/*------变量定义------*/

chassis_move_t chassis_move_data;  //底盘运动数据

/*------四轮全向轮底盘控制任务------*/

void chassis_task(void const *pvParameters) //底盘任务
{
  chassis_init(&chassis_move_data);//底盘初始化

  while (1)
  {
    chassis_mode_choose(&chassis_move_data);  //遥控器选择模式模式
    chassis_mode_set(&chassis_move_data); //控制模式设定
    chassis_control_cal(&chassis_move_data);//控制量计算

    CAN_cmd_chassis(chassis_move_data.chassis_motor[0].give_current,
                    chassis_move_data.chassis_motor[1].give_current,
                    chassis_move_data.chassis_motor[2].give_current,
                    chassis_move_data.chassis_motor[3].give_current);//计算过后的控制电流发送

  }
}

/*------函数定义------*/

/*=-=-=-=-=-=-=-=-=-=-=底盘初始化=-=-=-=-=-=-=-=-=-=-=*/
static void chassis_init(chassis_move_t *chassis_move_init)
{
  int8_t i;

  chassis_move_init->chassis_RC = get_remote_control_point(); //遥控器指针获取

  /*PID控制器初始化*/
  const static fp32 motor_speed_pid[3] = {CHASSIS_MOTOR_SPEED_PID_KP, CHASSIS_MOTOR_SPEED_PID_KI, CHASSIS_MOTOR_SPEED_PID_KD};  //底盘速度环pid值
  for (i = 0; i < 4; i++)
  {
    PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, CHASSIS_MOTOR_SPEED_PID_MAX_OUT, CHASSIS_MOTOR_SPEED_PID_MAX_IOUT);
  }

  /*底盘电机数据初始化*/
  for (i = 0; i < 4; i++)
  {
    chassis_move_init->chassis_motor[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
  }

}

/*=-=-=-=-=-=-=-=-=-=-=遥控器选择模式=-=-=-=-=-=-=-=-=-=-=*/
static void chassis_mode_choose(chassis_move_t *chassis_move_mode_choose)
{

  if(switch_is_down(chassis_move_mode_choose->chassis_RC->rc.switch_channel[0])) //下档模式
  {
    chassis_move_mode_choose->chassis_behaviour_mode = CHASSIS_INABILITY; //底盘无力
  }
  else if(switch_is_mid(chassis_move_mode_choose->chassis_RC->rc.switch_channel[0])) //中档模式
  {
    chassis_move_mode_choose->chassis_behaviour_mode = CHASSIS_SPIN_MODE; //小陀螺模式
  }
  else if (switch_is_up(chassis_move_mode_choose->chassis_RC->rc.switch_channel[0])) //上档模式
  {
    chassis_move_mode_choose->chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL; //底盘跟随云台
  }
  
}

/*=-=-=-=-=-=-=-=-=-=-=控制模式设定=-=-=-=-=-=-=-=-=-=-=*/
static void chassis_mode_set(chassis_move_t *chassis_move_mode_set)
{
  switch(chassis_move_mode_set->chassis_behaviour_mode) 
  {
    case CHASSIS_INABILITY: //底盘无力
      chassis_move_mode_set->vx_set = 0.0f;
      chassis_move_mode_set->vy_set = 0.0f;
      chassis_move_mode_set->vw_set = 0.0f;
      break;

    case CHASSIS_NO_FOLLOW_GIMBAL:  //底盘不跟随云台
      chassis_move_mode_set->vx_set = chassis_move_mode_set->chassis_RC->rc.remote_channel[CHASSIS_X_CHANNEL] * RC_TO_SPEED_RATIO;
      chassis_move_mode_set->vy_set = chassis_move_mode_set->chassis_RC->rc.remote_channel[CHASSIS_Y_CHANNEL] * RC_TO_SPEED_RATIO;
      chassis_move_mode_set->vw_set = chassis_move_mode_set->chassis_RC->rc.remote_channel[CHASSIS_W_CHANNEL] * RC_TO_SPEED_RATIO;
      
      break;
    
    /*添加模式：
    在chassis_task.h中 底盘运动行为模式 中添加模式名称
    然后在chassis_mode_set中witch内后添

    case 模式名称
          ········
      break
    
    以完成模式添加的操作
    */

    default:
      break;
  }
}

/*=-=-=-=-=-=-=-=-=-=-=控制量计算=-=-=-=-=-=-=-=-=-=-=*/
static void chassis_control_cal(chassis_move_t *chassis_move_control_cal)
{
  int8_t i;

  //底盘映射速度值转化为各个电机的速度值
  chassis_vector_to_omni_wheel_speed(chassis_move_control_cal);
    
  //将各个电机速度值计算转化为电机电流值(PID计算)
  for (i = 0; i < 4; i++)
  {
    PID_calc(&chassis_move_control_cal->motor_speed_pid[i], chassis_move_control_cal->chassis_motor[i].current_speed_fedback, chassis_move_control_cal->chassis_motor[i].speed_set);
  }
  
  //底盘功率控制
  //chassis_power_limit();
  
  //赋值电流值
  for (i = 0; i < 4; i++)
  {
    chassis_move_control_cal->chassis_motor[i].give_current = (int16_t)(chassis_move_control_cal->motor_speed_pid[i].out);
  }

}

/*=-=-=-=-=-=-=-=-=-=-=全向轮运动解算=-=-=-=-=-=-=-=-=-=-=*/
static void chassis_vector_to_omni_wheel_speed(chassis_move_t *chassis_vector_to_motor_speed) 
{   
    int8_t i;
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*3.14f)*CHASSIS_DECELE_RATIO*1000; //车轮转速比 = 60/(轮子周长*3.14) *底盘减速比 *1000

    wheel_rpm[0] = (chassis_vector_to_motor_speed->vx_set - chassis_vector_to_motor_speed->vy_set + chassis_vector_to_motor_speed->vw_set * ((CHASSIS_LENGTH/2)+(CHASSIS_WIDTH/2))) * wheel_rpm_ratio;
    wheel_rpm[1] = (chassis_vector_to_motor_speed->vx_set + chassis_vector_to_motor_speed->vy_set - chassis_vector_to_motor_speed->vw_set * ((CHASSIS_LENGTH/2)+(CHASSIS_WIDTH/2))) * wheel_rpm_ratio;
    wheel_rpm[2] = (chassis_vector_to_motor_speed->vx_set - chassis_vector_to_motor_speed->vy_set + chassis_vector_to_motor_speed->vw_set * ((CHASSIS_LENGTH/2)+(CHASSIS_WIDTH/2))) * wheel_rpm_ratio;
    wheel_rpm[3] = (chassis_vector_to_motor_speed->vx_set + chassis_vector_to_motor_speed->vy_set - chassis_vector_to_motor_speed->vw_set * ((CHASSIS_LENGTH/2)+(CHASSIS_WIDTH/2))) * wheel_rpm_ratio;
    
    for ( i = 0; i < 4; i++)
    {
      chassis_vector_to_motor_speed->chassis_motor[i].speed_set = wheel_rpm[i];
    }
    
}

