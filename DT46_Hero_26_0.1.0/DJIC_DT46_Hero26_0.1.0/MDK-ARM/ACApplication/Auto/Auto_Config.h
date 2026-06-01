#ifndef AUTO_CONFIG_H_
#define AUTO_CONFIG_H_

/* 系统相关 */

#define AUTO_TASK_TIME_MS 1     //自动任务系统循环时间 

//串口句柄
#define AUTO_USART_HANDLE       huart1    //串口使用到的句柄

#define AUTO_USART_HEADER       0x5A      //串口数据包头

/* 自瞄算法相关 */

//敌人颜色( 不知道能否自动获取值来更改 )
#define AUTOAIM_ENEMY_COLOR_RED       0     //红色
#define AUTOAIM_ENEMY_COLOR_BLUE      1     //蓝色

#define AUTOAIM_ENEMY_COLOR AUTOAIM_ENEMY_COLOR_RED //默认红色

//开关自瞄宏
#define AUTOAIM_OFF     0       //关闭自瞄
#define AUTOAIM_ON      1       //打开自瞄

//自瞄权重
#define AUTOAIM_WEIGHT_AUTO         80  
#define AUTOAIM_WEIGHT_MANUAL      (100 - AUTOAIM_WEIGHT_AUTO)



#endif // AUTO_CONFIG_H_
