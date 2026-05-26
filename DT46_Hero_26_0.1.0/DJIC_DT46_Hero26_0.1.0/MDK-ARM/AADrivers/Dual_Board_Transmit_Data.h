#include "CMDCenter.h"

//** ================================================================================ **//
//** ============================= 定义双板通信数据结构体 ============================ **//
//** ================================================================================ **//


#if(BOARD_MODE == BOARD_MODE_DUAL)

#if(BOARD_ID == GIMBAL_BOARD)

typedef struct{

    CMD_t CMD;

}BoardTransmit_Gimbal_TX_t;

typedef struct{

    uint8_t test;

}BoardTransmit_Gimbal_RX_t;

#endif

#if(BOARD_ID == CHASSIS_BOARD)

typedef struct{

    uint8_t test;
    
}BoardTransmit_Chassis_TX_t;

typedef struct{

    CMD_t CMD;

}BoardTransmit_Chassis_RX_t;

extern BoardTransmit_Chassis_RX_t  BoardCRX;

#endif

#endif
