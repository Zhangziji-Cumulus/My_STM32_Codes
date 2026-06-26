/*
 *   基于机甲大师高校系列赛通信协议V1.3.1编写
 *
 *
 */

#ifndef REFEREEPROTOCOL_H_
#define REFEREEPROTOCOL_H_

#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************* 硬件配置常量 *************************/
#define REFEREE_UART_BAUD_NORMAL    115200U   // 常规链路波特率
#define REFEREE_UART_BAUD_AIR       921600U   // 图传链路波特率
#define UART_DATA_BIT                8U
#define UART_STOP_BIT                1U
#define UART_PARITY_NONE             0U

/************************* 帧基础常量 *************************/
#define FRAME_SOF                    0xA5U     // 帧起始字节
#define FRAME_HEADER_LEN             5U        // 帧头总长度
#define FRAME_CMD_ID_LEN             2U        // 命令码长度
#define FRAME_CRC16_LEN              2U        // CRC16长度
#define FRAME_MIN_LEN                (FRAME_HEADER_LEN + FRAME_CMD_ID_LEN + FRAME_CRC16_LEN)
#define FRAME_MAX_DATA_LEN           118U      // 数据段最大长度
#define FRAME_MAX_PACK_LEN           127U      // 整包最大长度

/************************* 机器人ID定义 *************************/
// 红方
#define ROBOT_RED_HERO               1U
#define ROBOT_RED_ENGINEER           2U
#define ROBOT_RED_INFANTRY_3         3U
#define ROBOT_RED_INFANTRY_4         4U
#define ROBOT_RED_INFANTRY_5         5U
#define ROBOT_RED_AERIAL             6U
#define ROBOT_RED_SENTRY             7U
#define ROBOT_RED_DART               8U
#define ROBOT_RED_RADAR              9U
#define ROBOT_RED_OUTPOST            10U
#define ROBOT_RED_BASE               11U

// 蓝方
#define ROBOT_BLUE_HERO              101U
#define ROBOT_BLUE_ENGINEER          102U
#define ROBOT_BLUE_INFANTRY_3        103U
#define ROBOT_BLUE_INFANTRY_4        104U
#define ROBOT_BLUE_INFANTRY_5        105U
#define ROBOT_BLUE_AERIAL            106U
#define ROBOT_BLUE_SENTRY            107U
#define ROBOT_BLUE_DART              108U
#define ROBOT_BLUE_RADAR             109U
#define ROBOT_BLUE_OUTPOST           110U
#define ROBOT_BLUE_BASE              111U

/************************* 选手端ID *************************/
#define CLIENT_RED_HERO              0x0101U
#define CLIENT_RED_ENGINEER          0x0102U
#define CLIENT_RED_INFANTRY_3        0x0103U
#define CLIENT_RED_INFANTRY_4        0x0104U
#define CLIENT_RED_INFANTRY_5        0x0105U
#define CLIENT_RED_AERIAL            0x0106U

#define CLIENT_BLUE_HERO             0x0165U
#define CLIENT_BLUE_ENGINEER         0x0166U
#define CLIENT_BLUE_INFANTRY_3       0x0167U
#define CLIENT_BLUE_INFANTRY_4       0x0168U
#define CLIENT_BLUE_INFANTRY_5       0x0169U
#define CLIENT_BLUE_AERIAL           0x016AU

#define CLIENT_JUDGE_SERVER          0x8080U  // 裁判服务器ID

/************************* 命令码 CMD_ID *************************/
// ========== 上位机下发（接收解析） ==========
#define CMD_GAME_STATUS              0x0001U  // 比赛状态 1Hz
#define CMD_GAME_RESULT              0x0002U  // 比赛结果 触发
#define CMD_ROBOT_HP_DATA            0x0003U  // 全队血量 3Hz
#define CMD_FIELD_EVENT              0x0101U  // 场地事件 1Hz
#define CMD_REF_WARNING              0x0104U  // 裁判警告 触发
#define CMD_DART_INFO                0x0105U  // 飞镖状态 1Hz
#define CMD_ROBOT_STATUS             0x0201U  // 本机性能 10Hz
#define CMD_POWER_HEAT_DATA          0x0202U  // 能量热量 10Hz
#define CMD_ROBOT_POS                0x0203U  // 本机坐标 1Hz
#define CMD_ROBOT_BUFF               0x0204U  // 增益状态 3Hz
#define CMD_DAMAGE_DATA              0x0206U  // 受伤害 触发
#define CMD_SHOOT_DATA               0x0207U  // 射击数据 触发
#define CMD_PROJECTILE_ALLOW         0x0208U  // 剩余弹量 10Hz
#define CMD_RFID_STATUS              0x0209U  // RFID状态 3Hz
#define CMD_DART_CLIENT_CMD          0x020AU  // 飞镖选手指令 3Hz
#define CMD_GROUND_ROBOT_POS         0x020BU  // 地面机器人坐标 1Hz(哨兵)
#define CMD_RADAR_MARK_PROG          0x020CU  // 雷达标记进度 1Hz
#define CMD_SENTRY_AUTO_INFO         0x020DU  // 哨兵自主同步 1Hz
#define CMD_RADAR_AUTO_INFO          0x020EU  // 雷达自主同步 1Hz

// ========== 交互/图传链路（双向） ==========
#define CMD_ROBOT_INTERACT           0x0301U  // 机器人交互 30Hz上限
#define CMD_CUSTOM_CTRL_ROBOT        0x0302U  // 自定义控制器→机器人
#define CMD_MAP_CLICK_CMD            0x0303U  // 小地图点击指令
#define CMD_RADAR_MAP_DATA           0x0305U  // 雷达地图数据
#define CMD_CUSTOM_CTRL_CLIENT       0x0306U  // 键鼠模拟选手端
#define CMD_SENTRY_PATH_DATA         0x0307U  // 哨兵路径数据
#define CMD_ROBOT_TO_CLIENT_MSG      0x0308U  // 机器人→选手端文字
#define CMD_ROBOT_TO_CUSTOM_CTRL     0x0309U  // 机器人→自定义控制器
#define CMD_ROBOT_TO_CUSTOM_CLIENT   0x0310U  // 机器人→客户端大数据
#define CMD_CUSTOM_CLIENT_TO_ROBOT   0x0311U  // 客户端→机器人指令

// ========== 雷达无线链路（信号发射源发送） ==========
#define CMD_RADAR_ENEMY_POS          0x0A01U  // 敌方坐标 10Hz
#define CMD_RADAR_ENEMY_HP           0x0A02U  // 敌方血量 10Hz
#define CMD_RADAR_ENEMY_BULLET       0x0A03U  // 敌方弹量 10Hz
#define CMD_RADAR_GLOBAL_STATUS      0x0A04U  // 敌方全局状态 10Hz
#define CMD_RADAR_ENEMY_BUFF         0x0A05U  // 敌方增益 10Hz
#define CMD_RADAR_ENEMY_KEY          0x0A06U  // 敌方密钥 10Hz

/************************* 0x0301子命令ID *************************/
#define SUB_CMD_ROBOT_COMM           0x0200U  // 机器人自定义通信
#define SUB_CMD_MAP_DEL_LAYER        0x0100U  // 删除图层
#define SUB_CMD_MAP_DRAW_1           0x0101U  // 绘制1个图形
#define SUB_CMD_MAP_DRAW_2           0x0102U  // 绘制2个图形
#define SUB_CMD_MAP_DRAW_5           0x0103U  // 绘制5个图形
#define SUB_CMD_MAP_DRAW_7           0x0104U  // 绘制7个图形
#define SUB_CMD_MAP_DRAW_TEXT        0x0110U  // 绘制字符
#define SUB_CMD_SENTRY_AUTO_CMD      0x0120U  // 哨兵自主指令
#define SUB_CMD_RADAR_AUTO_CMD       0x0121U  // 雷达自主指令

/************************* 编译器字节对齐配置 *************************/
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#pragma pack(1)
#define PACKED
#elif defined(__GNUC__)
#define PACKED __attribute__((packed))
#else
#define PACKED
#endif

// ========== 帧头结构体 ==========
typedef struct
{
    uint8_t  sof;               // 帧起始标志 固定0xA5
    uint16_t data_len;          // 数据段字节长度
    uint8_t  seq;               // 包序号自增
    uint8_t  crc8;              // 帧头CRC8校验
} PACKED referee_frame_header_t;

// ========== 完整帧缓存结构体 ==========
typedef struct
{
    referee_frame_header_t header;
    uint16_t cmd_id;
    uint8_t  data[FRAME_MAX_DATA_LEN];
    uint16_t crc16;
} PACKED referee_full_frame_t;

/************************* 各命令码数据结构体 *************************/
// 0x0001 比赛状态
typedef struct
{
    uint8_t  game_type : 4;     // 比赛类型
    uint8_t  game_progress : 4; // 比赛阶段
    uint16_t stage_remain_time; // 阶段剩余时间(秒)
    uint64_t sync_time_stamp;   // UNIX时间戳
} PACKED game_status_t;

// 0x0002 比赛结果
typedef struct
{
    uint8_t winner;             // 0平局 1红胜 2蓝胜
} PACKED game_result_t;

// 0x0003 全队血量
typedef struct
{
    uint16_t ally_1_robot_HP;   // 1号英雄血量
    uint16_t ally_2_robot_HP;   // 2号工程血量
    uint16_t ally_3_robot_HP;   // 3号步兵血量
    uint16_t ally_4_robot_HP;   // 4号步兵血量
    uint16_t reserved;          // 保留
    uint16_t ally_7_robot_HP;   // 7号哨兵血量
    uint16_t ally_outpost_HP;   // 前哨站血量
    uint16_t ally_base_HP;      // 基地血量
} PACKED game_robot_HP_t;

// 0x0104 裁判警告
typedef struct
{
    uint8_t level;              // 判罚等级
    uint8_t robot_id;           // 违规机器人ID
    uint8_t count;              // 累计违规次数
} PACKED referee_warning_t;

// 0x0105 飞镖信息
typedef struct
{
    uint8_t  dart_remaining_time; // 发射冷却剩余秒数
    uint16_t dart_info;           // 击中目标/计数/选中目标位域
} PACKED dart_info_t;

// 0x0201 机器人性能状态
typedef struct
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t  power_management_gimbal_output  : 1;
    uint8_t  power_management_chassis_output : 1;
    uint8_t  power_management_shooter_output : 1;
} PACKED robot_status_t;

// 0x0202 缓冲能量与热量
typedef struct
{
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} PACKED power_heat_data_t;

// 0x0203 机器人坐标
typedef struct
{
    float x;
    float y;
    float angle;
} PACKED robot_pos_t;

// 0x0207 射击数据
typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float   initial_speed;
} PACKED shoot_data_t;

// 0x0208 允许发弹量
typedef struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} PACKED projectile_allowance_t;

// 0x020A 飞镖选手端指令
typedef struct
{
    uint8_t  dart_launch_opening_status;
    uint8_t  reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} PACKED dart_client_cmd_t;

// 0x020B 地面机器人坐标
typedef struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved1;
    float reserved2;
} PACKED ground_robot_position_t;

// 0x020D 哨兵自主信息
typedef struct
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} PACKED sentry_info_t;

// 0x020E 雷达自主信息
typedef struct
{
    uint8_t radar_info;
} PACKED radar_info_t;

// 0x0301 机器人交互数据头
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t  user_data[112];
} PACKED robot_interaction_data_t;

// 0x0303 小地图点击指令
typedef struct
{
    float   target_position_x;
    float   target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
} PACKED map_command_t;

// 0x0306 键鼠模拟数据
typedef struct
{
    uint16_t key_value;
    uint16_t x_position   : 12;
    uint16_t mouse_left   : 4;
    uint16_t y_position   : 12;
    uint16_t mouse_right  : 4;
    uint16_t reserved;
} PACKED custom_client_data_t;

// 0x0307 哨兵路径数据
typedef struct
{
    uint8_t  intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t   delta_x[49];
    int8_t   delta_y[49];
    uint16_t sender_id;
} PACKED map_data_t;

// 子命令0x0120 哨兵自主指令
typedef struct
{
    uint32_t sentry_cmd;
} PACKED sentry_cmd_t;

// 子命令0x0121 雷达自主指令
typedef struct
{
    uint8_t radar_cmd;
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
} PACKED radar_cmd_t;

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#pragma pack()
#endif

/************************* 全数据指针总结构体 *************************/
typedef struct
{
    // 基础比赛数据指针
    game_status_t            *p_game_status;
    game_result_t            *p_game_result;
    game_robot_HP_t          *p_ally_hp;
    uint8_t                  *p_field_event;
    referee_warning_t        *p_ref_warning;
    dart_info_t              *p_dart_info;

    // 本机状态指针
    robot_status_t           *p_robot_status;
    power_heat_data_t        *p_power_heat;
    robot_pos_t              *p_robot_pos;
    uint8_t                  *p_buff_buf;
    uint8_t                  *p_damage_buf;
    shoot_data_t             *p_shoot_data;
    projectile_allowance_t   *p_bullet_data;
    uint8_t                  *p_rfid_buf;
    dart_client_cmd_t        *p_dart_client_cmd;
    ground_robot_position_t  *p_ally_ground_pos;
    uint16_t                 *p_radar_mark_bit;
    sentry_info_t            *p_sentry_sync;
    radar_info_t             *p_radar_sync;

    // 交互数据指针
    robot_interaction_data_t *p_robot_inter;
    map_command_t            *p_map_click;
    custom_client_data_t     *p_mouse_key;
    map_data_t               *p_sentry_path;

    // 子指令指针
    sentry_cmd_t             *p_sentry_auto_cmd;
    radar_cmd_t              *p_radar_auto_cmd;

    // 更新标志位联合体
    union
    {
        uint32_t all_flag;
        struct
        {
            uint32_t game_status_upd     : 1;
            uint32_t game_result_upd     : 1;
            uint32_t ally_hp_upd         : 1;
            uint32_t field_event_upd     : 1;
            uint32_t ref_warn_upd        : 1;
            uint32_t dart_info_upd       : 1;
            uint32_t robot_status_upd    : 1;
            uint32_t power_heat_upd      : 1;
            uint32_t robot_pos_upd       : 1;
            uint32_t buff_upd            : 1;
            uint32_t damage_upd          : 1;
            uint32_t shoot_upd           : 1;
            uint32_t bullet_upd          : 1;
            uint32_t rfid_upd            : 1;
            uint32_t dart_client_upd     : 1;
            uint32_t ally_ground_upd     : 1;
            uint32_t radar_mark_upd      : 1;
            uint32_t sentry_sync_upd     : 1;
            uint32_t radar_sync_upd      : 1;
            uint32_t robot_inter_upd     : 1;
            uint32_t map_click_upd       : 1;
            uint32_t mouse_key_upd       : 1;
            uint32_t sentry_path_upd     : 1;
            uint32_t reserved            : 8;
        } bit;
    } update;
} referee_all_data_t;

/************************* 解析状态机 *************************/
typedef enum
{
    WAIT_SOF = 0,
    WAIT_HEADER_REST,
    WAIT_DATA_AND_CRC16
} ref_parse_state_e;

// 解析器句柄
typedef struct
{
    ref_parse_state_e  state;
    uint16_t           data_cnt;
    referee_full_frame_t frame_buf;
} referee_parser_t;

/************************* 全局变量声明 *************************/
extern referee_all_data_t g_ref_data;

/************************* CRC校验函数声明 *************************/
uint8_t  Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void     Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void     Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/************************* 解析器函数声明 *************************/
void Referee_Parser_Init(referee_parser_t *parser);
void Referee_Parser_Byte(referee_parser_t *parser, uint8_t byte);
void Referee_Frame_Callback(referee_full_frame_t *frame); // 弱定义，用户可重写

/************************* 打包发送函数声明（下位机发送用） *************************/
// 通用打包函数：填充帧头+命令码+数据+CRC，返回整包长度
uint16_t Referee_Pack_Frame(uint16_t cmd_id, uint8_t *data, uint16_t data_len, uint8_t seq, uint8_t *out_buf);

// 常用发送封装
uint16_t Referee_Pack_Interaction(uint16_t sub_cmd, uint16_t sender_id, uint16_t receiver_id,
                                  uint8_t *user_data, uint8_t data_len, uint8_t seq, uint8_t *out_buf);
uint16_t Referee_Pack_Sentry_Cmd(uint32_t cmd_val, uint16_t sender_id, uint8_t seq, uint8_t *out_buf);
uint16_t Referee_Pack_Radar_Cmd(uint8_t cmd_val, uint8_t *password, uint16_t sender_id, uint8_t seq, uint8_t *out_buf);
uint16_t Referee_Pack_Custom_Ctrl(uint8_t *data, uint8_t seq, uint8_t *out_buf);
uint16_t Referee_Pack_Mouse_Key(custom_client_data_t *mk_data, uint8_t seq, uint8_t *out_buf);

#ifdef __cplusplus
}
#endif

#endif // REFEREEPROTOCOL_H_