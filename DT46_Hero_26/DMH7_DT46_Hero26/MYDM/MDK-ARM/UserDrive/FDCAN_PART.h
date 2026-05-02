#ifndef __FDCAN_PART_H
#define __FDCAN_PART_H

//** ############################################# **//
//** ================= 引用头文件 ================= **//
//** ############################################# **//

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "fdcan.h"

//** ########################################## **//
//** ================= 宏定义 ================= **//
//** ########################################## **//

//转速限制//
#define DJI_M2006_MAX_RPM 2000



//** ############################################### **//
//** ================= 对外函数声明 ================= **//
//** ############################################### **//


//** --------------------------------------------- **//
//** ================= CAN初始化 ================= **//
//** --------------------------------------------- **//

// 函数声明
HAL_StatusTypeDef FDCAN_Filter_AcceptAllID(FDCAN_HandleTypeDef *hfdcan, uint32_t filter_index);
void fdcan_filter_init(void);
void FDCAN_Send_STD(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t* data);
bool FDCAN_SendFloatArray(FDCAN_HandleTypeDef* hfdcan, float* data, uint8_t length, uint16_t ID);

#endif
