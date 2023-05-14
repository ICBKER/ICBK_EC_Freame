#ifndef CHASSIS_TASK_H

#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "config_freame.h"
#include "remote_control.h"
#include "pid.h"


/*遥控器死区大小设置*/
#define CHASSIS_RC_DEADZONE 10  //死区RC通道值
#define RC_TO_SPEED_RATIO 1000 //RC通道值转化速度比 = 最大速度/最大通道值

//底盘3508最大can发送电流值
#define MOTOR_M3508_CAN_MAX_CURRENT CONFIG_MOTOR_M3508_CAN_MAX_CURRENT

/*遥控器通道值*/
#define CHASSIS_X_CHANNEL 1 
#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_W_CHANNEL 4

/*底盘机械物理参数*/
#define WHEEL_PERIMETER CONFIG_WHEEL_PERIMETER  //轮子周长
#define CHASSIS_DECELE_RATIO CONFIG_CHASSIS_DECELE_RATIO //电机减速比
#define CHASSIS_LENGTH CONFIG_CHASSIS_LENGTH //底盘长度
#define CHASSIS_WIDTH CONFIG_CHASSIS_WIDTH  //底盘宽度

/*底盘M3508电机速度环PID参数*/
#define CHASSIS_MOTOR_SPEED_PID_KP CONFIG_CHASSIS_MOTOR_SPEED_PID_KP
#define CHASSIS_MOTOR_SPEED_PID_KI CONFIG_CHASSIS_MOTOR_SPEED_PID_KI
#define CHASSIS_MOTOR_SPEED_PID_KD CONFIG_CHASSIS_MOTOR_SPEED_PID_KD
#define CHASSIS_MOTOR_SPEED_PID_MAX_OUT CONFIG_CHASSIS_MOTOR_SPEED_PID_MAX_OUT //将3508最大CAN发送电流值作为最大输出
#define CHASSIS_MOTOR_SPEED_PID_MAX_IOUT CONFIG_CHASSIS_MOTOR_SPEED_PID_MAX_IOUT

/*--------底盘运动行为模式--------*/
typedef enum
{
  CHASSIS_INABILITY,
  CHASSIS_FOLLOW_GIMBAL,
  CHASSIS_NO_FOLLOW_GIMBAL,
  CHASSIS_SPIN_MODE,

}chassis_mode_e;

/*--------底盘电机数据结构--------*/
typedef struct
{
  const motor_measure_t *chassis_motor_measure; //接收的电机数据
  fp32 speed_set; //计算后的电机速度(pid 速度环目标值)
  fp32 current_speed_fedback; //当前的电机速度值（pid 速度环反馈值）
  int16_t give_current; //给定电机电流值
} chassis_motor_t;


/*--------底盘运动数据结构--------*/
typedef struct
{
  const RC_ctrl_t *chassis_RC;  //底盘使用的遥控器指针
  chassis_mode_e chassis_behaviour_mode;  //底盘运动行为模式
  chassis_motor_t chassis_motor[4]; //底盘电机数据
  pid_type_def motor_speed_pid[4];  //底盘电机速度环pid

  fp32 vx_set;
  fp32 vy_set;
  fp32 vw_set;

} chassis_move_t;

extern void chassis_task(void const *pvParameters);

#endif
