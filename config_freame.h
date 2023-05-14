#ifndef CONFIG_FREAME_H

#define CONFIG_FREAME_H
/*
	统一配置文件config.h
	通过 #define CONFIG_XXX_XXX_···· ···· 来进行参数的统一
*/

/* 底盘参数 */
//底盘3508最大can发送电流值
#define CONFIG_MOTOR_M3508_CAN_MAX_CURRENT 16000.0f

/* 底盘机械物理参数 */
#define CONFIG_WHEEL_PERIMETER 10 //轮子周长
#define CONFIG_CHASSIS_DECELE_RATIO 10 //电机减速比
#define CONFIG_CHASSIS_LENGTH 10 //底盘长度
#define CONFIG_CHASSIS_WIDTH 10  //底盘宽度

/*底盘M3508电机速度环PID参数*/
#define CONFIG_CHASSIS_MOTOR_SPEED_PID_KP 15000.0f
#define CONFIG_CHASSIS_MOTOR_SPEED_PID_KI 10.0f
#define CONFIG_CHASSIS_MOTOR_SPEED_PID_KD 0.0f
#define CONFIG_CHASSIS_MOTOR_SPEED_PID_MAX_OUT CONFIG_MOTOR_M3508_CAN_MAX_CURRENT //将3508最大CAN发送电流值作为最大输出
#define CONFIG_CHASSIS_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


#endif
