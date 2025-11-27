#ifndef __PID_H
#define __PID_H

// PID控制器结构体，包含所有相关变量
typedef struct {
    // 核心参数（需根据实际系统调试设定）
    float kp;         // 比例系数
    float ki;         // 积分系数
    float kd;         // 微分系数

    // 目标与当前状态
    float target;     // 目标值（设定值）
    float current;    // 当前测量值
    float error;      // 当前误差（target - current）
    float last_error; // 上一次误差（用于计算微分）
    float error_sum;  // 误差积分累计（用于积分项计算）

    // 输出限制（防止执行器饱和）
    float output_min; // 输出最小值
    float output_max; // 输出最大值

    // 积分限幅（防止积分饱和）
    float integral_min; // 积分项最小值
    float integral_max; // 积分项最大值
	
	float Output;
} PID_HandleTypeDef;

typedef struct
{
	PID_HandleTypeDef In_PID;
	PID_HandleTypeDef Ex_PID;
}PID_DoubleDef;

typedef struct
{
	PID_HandleTypeDef In_PID;
	PID_HandleTypeDef Min_PID;
	PID_HandleTypeDef Ex_PID;
}PID_Triple;


void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max);
float PID_Calculate(PID_HandleTypeDef *pid, float current_val, float target_val);
float PID_Double_Caculate(PID_HandleTypeDef* PID_In,
						  PID_HandleTypeDef* PID_Ex,
						  float Tartget,
						  float Current_In,
					      float Current_Ex,
						  float MError);
float PID_Triple_Calculate(PID_HandleTypeDef* PID_Angle,
                           PID_HandleTypeDef* PID_Speed,
                           PID_HandleTypeDef* PID_Current,
                           float target_angle,
                           float current_angle,
                           float current_speed,
                           float current_current,
                           float max_angle_error);
float PID_Calculate_CycleAngle(PID_HandleTypeDef *pid, float current, float target);
float PID_Double_CycleAngle(PID_HandleTypeDef* PID_In,PID_HandleTypeDef* PID_Ex,float Tartget,float Current_In,float Current_Ex,float MError);

#endif
			  