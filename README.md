# ICBK_EC_Freame
ICBK电控代码框架
依赖工具：
	工具：keil5 VScode
	软件环境：Windows10
	硬件环境：STM32F407IGHx

文件层次：
	-   .vscode (VScode配置)
	- 
	-   Application (系统应用层)
		- Apps(系统控制层)
		- Tasks(FreeRTOS任务层)
	-  Components(组件层)
		-   Algorithm (各种算法层)
		-   Controller(控制器层)
		-   Communication(外部数据交流层)
		-   Devices (外部设备层)
	-   BSP (用户定义外设层)
	- 
	-   config.h(统一配置文件)
	- 
	-   Drivers (HAL库驱动层 自带)
	-   Core (主函数和中断层的头与原文件 自带)
	-   MDK-ARM (keil工程文件和编译文件 自带)
	-   Middlewares (Freertos层 自带) 
