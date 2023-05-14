# ICBK_EC_Freame
ICBK电控代码框架
## 依赖工具及软硬件环境

`工具:`VSCode，Keil5

`软件环境:`Windows10

`硬件环境:`STM32F427

## 编译方式

C/C++编译

## 文件层次

* Application (系统应用层)
	* Apps(系统控制层)
	* Tasks(FreeRTOS任务层)
* Components(组件层)
	* Algorithm (各种算法层)
	* Controller(控制器层)
	* Communication(外部数据交流层)
	* Devices (外部设备层)
	* BSP (用户定义外设层)
*  config.h(统一配置文件)
*  Drivers (HAL库驱动层 Cube生成)
*  Core (主函数和中断层的头与原文件 Cube生成)
*  MDK-ARM (keil工程文件和编译文件 Cube生成)
*  Middlewares (Freertos层 Cube生成) 
