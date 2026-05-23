#ifndef MOTORCTRL_DJI_STOP_H_
#define MOTORCTRL_DJI_STOP_H_

#include "A_MCommon.h"

#define DJI_MOTOR_STOP_THRESHOLD  1.0f   //电机急停的误差阈值
#define DJI_MOTOR_STOP_TIME_MS    500    //连续进入停止模式的时间阈值，单位毫秒

void DJI_MOTOR_EmergencySTOP_ALL(PID_HandleTypeDef* Motor_STOP,DJI_MotorFeedback_t DJI_MFeedback[],CAN_HandleTypeDef *hcan,float MError);

#endif // MOTORCTRL_DJI_STOP_H_