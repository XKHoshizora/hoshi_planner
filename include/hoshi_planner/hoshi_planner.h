#ifndef HOSHI_PLANNER_H
#define HOSHI_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

namespace hoshi_planner {

    /*
     * @class HoshiPlanner
     * @brief 这是一个 HoshiPlanner 类，实现了 nav_core::BaseLocalPlanner 接口
     * @details 这个类实现了 HoshiPlanner 算法，用于在机器人移动过程中规划速度指令
     */
    class HoshiPlanner : public nav_core::BaseLocalPlanner {

    public:

        HoshiPlanner();  // 构造函数
        ~HoshiPlanner();  // 析构函数

        /*
         * @brief 初始化函数，用于初始化 HoshiPlanner
         * @param name 节点名称
         * @param tf 坐标变换对象
         * @param costmap_ros 代价地图对象
         */
        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

        /*
         * @brief 设置跟踪路径（全局路径）
         * @param plan 全局路径
         * @return 是否设置成功
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /*
         * @brief 计算速度指令
         * @param cmd_vel 速度指令
         * @return 是否计算成功
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /*
         * @brief 判断是否到达目标点
         * @details 会被 move_base 循环调用
         * @return 是否到达目标点，未到达时返回 false，已到达则返回 true
         */
        bool isGoalReached();

    };

}  // namespace hoshi_planner

#endif // HOSHI_PLANNER_H