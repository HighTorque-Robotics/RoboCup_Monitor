#pragma once

#include <string>
#include <vector>
#include <memory>
#include <chrono>   // 用于替换 ros::Time
#include <array>    // 用于替换 boost::array
#include <cstdint>  // 用于 uint8_t, uint16_t 等

// 定义一个标准的时间点类型，用来替换 ros::Time
using StdTimePoint = std::chrono::time_point<std::chrono::system_clock>;

/**
 * @brief dmonitor 命名空间
 */
namespace dmonitor {
    /**
     * @brief 替换 geometry_msgs/Vector3
     */
    struct Point3D {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    /**
     * @brief 替换 geometry_msgs/Vector3 (当只使用 x 和 y 时)
     */
    struct Point2D {
        double x = 0.0;
        double y = 0.0;
    };
} // namespace dmonitor


/**
 * @brief dmsgs 命名空间
 * 包含所有从 ROS .msg 文件迁移过来的 C++ 结构体
 */
namespace dmsgs
{

/**
 * @brief 替换 dmsgs/GCInfo.h
 * 严格从 auto-generated ROS header 迁移。
 */
struct GCInfo
{
    // --- Enums (来自 GCInfo.h) ---
    enum {
        STATE_INITIAL = 0u,
        STATE_READY = 1u,
        STATE_SET = 2u,
        STATE_PLAYING = 3u,
        STATE_FINISHED = 4u
    };
    enum State2 {
        STATE2_NORMAL = 0u,
        STATE2_PENALTYSHOOT = 1u,
        STATE2_OVERTIME = 2u,
        STATE2_TIMEOUT = 3u,
        STATE2_DIRECT_FREEKICK = 4u,
        STATE2_INDIRECT_FREEKICK = 5u,
        STATE2_PENALTYKICK = 6u,
        STATE2_CORNER_KICK = 7u,
        STATE2_GOAL_KICK = 8u,
        STATE2_THROW_IN = 9u
    };

    // --- Struct Members (来自 GCInfo.h) ---
    // 注意：ROS 的 'bool' 消息类型在 C++ 中经常被实现为 uint8_t
    uint8_t connected = 0;
    uint8_t gameType = 0;
    uint8_t state = 0; // 将使用 STATE_... 枚举
    uint8_t firstHalf = 0;
    uint8_t kickoff = 0;
    uint8_t secondaryState = 0; // 将使用 STATE2_... 枚举
    uint16_t secsRemaining = 0;
    uint16_t secondaryTime = 0;
    uint8_t ourIndirectFreeKick = 0;
    uint8_t ourDirectFreeKick = 0;
    uint8_t ourPenaltyKick = 0;
    uint8_t ourCornerKick = 0;
    uint8_t ourGoalKick = 0;
    uint8_t ourThrowIn = 0;
    uint8_t enemyIndirectFreeKick = 0;
    uint8_t enemyDirectFreeKick = 0;
    uint8_t enemyPenaltyKick = 0;
    uint8_t enemyCornerKick = 0;
    uint8_t enemyGoalKick = 0;
    uint8_t enemyThrowIn = 0;
    uint8_t state2Ready = 0;
    uint8_t state2Freeze = 0;
    uint8_t penalised = 0;
    uint8_t secsTillUnpenalised = 0;
    uint8_t teamCyan = 0;
    uint8_t ourScore = 0;
    uint8_t enemyScore = 0;
    
    // 默认构造函数
    GCInfo() = default;
};


/**
 * @brief 替换 dmsgs/BehaviorInfo.h
 * 严格从 auto-generated ROS header 迁移。
 */
struct BehaviorInfo
{
    // --- Enums (来自 BehaviorInfo.h) ---
    enum Role {
        ROLE_STRIKER = 1u,
        ROLE_DEFENDER = 2u,
        ROLE_SUPPORTER = 3u,
        ROLE_GOALIE = 4u,
        ROLE_OTHER = 5u
    };

    // --- Struct Members (来自 BehaviorInfo.h) ---
    uint8_t current_role = 0; // 将使用 Role 枚举
    std::string skill;
    dmonitor::Point3D dest;         // 替换 ::geometry_msgs::Vector3_
    dmonitor::Point3D final_dest;   // 替换 ::geometry_msgs::Vector3_
    dmonitor::Point3D attack_target;// 替换 ::geometry_msgs::Vector3_
    
    // ROS 的 'bool' 消息类型在 C++ 中经常被实现为 uint8_t
    uint8_t save_image = 0;
    uint8_t more_lines = 0;
    uint8_t vision_correct_yaw = 0;
    uint8_t attack_right = 0;
    
    uint8_t team_play_state = 0;
    uint8_t kicker_id = 0;
    uint8_t team_play_priority = 0;
    
    // 'bool[6]' 在 ROS C++ 中变为 'boost::array<uint8_t, 6>'
    // 用 'std::array' 替换 'boost::array'
    std::array<uint8_t, 6> mates_online{}; 
    
    float time_since_last_kick = 0.0f;

    // 默认构造函数
    BehaviorInfo() = default;
};


/**
 * @brief 替换 dmsgs/TeamInfo.h
 * 此结构体基于 teaminfo.msg 和 monitor.cpp 中的 JSON 函数
 * 以确保日志回放功能的兼容性。
 */
struct TeamInfo
{
    // --- Enums (来自 teaminfo.msg) ---
    enum StateEnum {
        INACTIVE = 0,
        PLAYING = 1,
        BALL_HANDLING = 2,
        PLACING = 3,
        SEARCHING = 4,
        INITIALIZING = 5,
        ASSISTING = 6
    };
    // enum PriorityEnum {
    //     LOW_PRIORITY = 0,
    //     NORMAL_PRIORITY = 1,
    //     HIGH_PRIORITY = 2
    // };

    // --- Struct Members (来自 teaminfo.msg 和 monitor.cpp) ---

    // general info (来自 .msg)
    StdTimePoint txp_timestamp;     // 替换 'time'
    StdTimePoint recv_timestamp;    // 替换 'time'
    uint8_t player_number = 0;
    uint8_t team_number = 0;
    uint8_t test = 0;
    bool incapacitated = false;

    // Behavior info (来自 .msg)
    uint8_t role = 0; // 使用 BehaviorInfo::Role 枚举
    bool attack_right = 0;

    dmonitor::Point3D dest;         // 替换 geometry_msgs/Vector3
    dmonitor::Point3D final_dest;   // 替换 geometry_msgs/Vector3
    dmonitor::Point3D attack_target;// 替换 geometry_msgs/Vector3
    float time_since_last_kick = 0.0f;

    // playing state of robot (来自 .msg)
    uint8_t state = 0; // 使用 StateEnum

    // priority of current behavior (来自 .msg)
    bool priority = false; 
    uint8_t kicker_id = 0;
    bool mates_online[6];

    // Vision info (来自 .msg)
    bool see_ball = false;
    bool see_circle = false;
    bool see_goal = false;
    dmonitor::Point3D robot_pos;    // 替换 geometry_msgs/Vector3
    dmonitor::Point3D ball_field;   // 替换 geometry_msgs/Vector3
    dmonitor::Point3D ball_global;  // 替换 geometry_msgs/Vector3
    dmonitor::Point3D circle_field; // 替换 geometry_msgs/Vector3
    dmonitor::Point3D circle_global;// 替换 geometry_msgs/Vector3
    dmonitor::Point3D goal_field; // 替换 geometry_msgs/Vector3
    dmonitor::Point3D goal_global;// 替换 geometry_msgs/Vector3

    // Quality (来自 .msg)
    float ball_quality = 0.0f;
    float field_quality = 0.0f;
    float field_consistency = 0.0f;

    // GCInfo (来自 .msg)
    bool gc_connected = false;
    uint8_t gc_state = 0;   // 使用 GCInfo::State 枚举
    uint8_t gc_state2 = 0;  // 使用 GCInfo::State2 枚举

    // 默认构造函数
    TeamInfo() = default;
};

} // namespace dmsgs