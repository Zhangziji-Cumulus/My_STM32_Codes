#ifndef ___MY_MATH_H
#define ___MY_MATH_H

#include "math.h"
#define MY_PI 3.14159265358979323846

float MyMath_Limit_Float(float value, float min, float max, int is_cycle);
float MyMath_Map_Range(float input, float input_min, float input_max, float target_min, float target_max);
float Degrees_To_Radians(float degrees);
float get_accumulated_angle(float current_angle);
float convert_angle(float angle);

#endif 
