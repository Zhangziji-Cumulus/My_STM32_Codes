#ifndef MOTORCTRL_DJI_STOP_H_
#define MOTORCTRL_DJI_STOP_H_

#include "A_MCommon.h"

void DJI_MOTOR_EmergencySTOP_ALL(PID_HandleTypeDef* Motor_STOP,DJI_MotorFeedback_t DJI_MFeedback[],CAN_HandleTypeDef *hcan,float MError);

#endif // MOTORCTRL_DJI_STOP_H_