#include <hoshi_planner/hoshi_planner.h>

namespace hoshi_planner {

    HoshiPlanner::HoshiPlanner(){
        setlocal(LC_ALL, "");  // 设置为空字符串，避免中文乱码
    }
    HoshiPlanner::~HoshiPlanner(){

    }

    // 初始化函数
    void HoshiPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_INFO("Initializing HoshiPlanner");
        ROS_WARN("是时候展现 HoshiPlanner 真正的力量了！");
    }

    // 设置跟踪路径（全局路径）
    bool HoshiPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
        return true;
    }

    // 计算速度指令
    bool HoshiPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        return true;
    }

    // 判断是否到达目标点（会被 move_base 循环调用，未到达时返回 false，已到达则返回 true）
    bool HoshiPlanner::isGoalReached(){
        return false;
    }

}  // namespace hoshi_planner

// 注册为 nav_core 插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hoshi_planner::HoshiPlanner, nav_core::BaseLocalPlanner)