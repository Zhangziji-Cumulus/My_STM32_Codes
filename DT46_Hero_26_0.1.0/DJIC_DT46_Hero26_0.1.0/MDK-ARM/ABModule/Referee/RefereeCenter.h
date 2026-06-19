#ifndef REFEREECENTER_H_
#define REFEREECENTER_H_
//** #################################################################################################### **//
//** =============================================== 宏定义 ============================================= **//
//** #################################################################################################### **//

/* ====================== 机器人实体ID ====================== */
// 红方机器人ID
#define ROBOT_RED_HERO              1U
#define ROBOT_RED_ENGINEER          2U
#define ROBOT_RED_INFANTRY_3        3U
#define ROBOT_RED_INFANTRY_4        4U
#define ROBOT_RED_INFANTRY_5        5U
#define ROBOT_RED_AERIAL            6U
#define ROBOT_RED_SENTRY            7U
#define ROBOT_RED_DART              8U
#define ROBOT_RED_RADAR             9U
#define ROBOT_RED_OUTPOST           10U
#define ROBOT_RED_BASE              11U

// 蓝方机器人ID
#define ROBOT_BLUE_HERO             101U
#define ROBOT_BLUE_ENGINEER         102U
#define ROBOT_BLUE_INFANTRY_3       103U
#define ROBOT_BLUE_INFANTRY_4       104U
#define ROBOT_BLUE_INFANTRY_5       105U
#define ROBOT_BLUE_AERIAL           106U
#define ROBOT_BLUE_SENTRY           107U
#define ROBOT_BLUE_DART             108U
#define ROBOT_BLUE_RADAR            109U
#define ROBOT_BLUE_OUTPOST          110U
#define ROBOT_BLUE_BASE             111U

/* ====================== 选手端通信ID(十六进制) ====================== */
// 红方选手端ID
#define CLIENT_RED_HERO             0x0101U
#define CLIENT_RED_ENGINEER         0x0102U
#define CLIENT_RED_INFANTRY_3       0x0103U
#define CLIENT_RED_INFANTRY_4       0x0104U
#define CLIENT_RED_INFANTRY_5       0x0105U
#define CLIENT_RED_AERIAL           0x0106U

// 蓝方选手端ID
#define CLIENT_BLUE_HERO            0x0165U
#define CLIENT_BLUE_ENGINEER        0x0166U
#define CLIENT_BLUE_INFANTRY_3      0x0167U
#define CLIENT_BLUE_INFANTRY_4      0x0168U
#define CLIENT_BLUE_INFANTRY_5      0x0169U
#define CLIENT_BLUE_AERIAL          0x016AU

// 裁判系统服务器ID(哨兵、雷达自主决策指令)
#define CLIENT_JUDGE_SERVER         0x8080U


//** #################################################################################################### **//
//** ============================================= 枚举、结构体 ========================================== **//
//** #################################################################################################### **//

struct
{
    struct{

    }Power;

}RefereeFeedback_t;

#endif // REFEREECENTER_H_