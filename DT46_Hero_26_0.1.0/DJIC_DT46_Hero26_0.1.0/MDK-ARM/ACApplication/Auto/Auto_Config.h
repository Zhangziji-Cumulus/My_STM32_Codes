#ifndef AUTO_CONFIG_H_
#define AUTO_CONFIG_H_

#define AUTO_TASK_TIME_MS 1     //自动任务系统循环时间 

//开关自瞄宏
#define AUTOAIM_OFF     0       //关闭自瞄
#define AUTOAIM_ON      1       //打开自瞄

//自瞄权重
#define AUTOAIM_WEIGHT_AUTO        80  
#define AUTOAIM_WEIGHT_MANUAL      (100 - AUTOAIM_WEIGHT_AUTO)

#endif // AUTO_CONFIG_H_
