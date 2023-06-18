#ifndef AGV_DISTANCE_DETECTION_H
#define AGV_DISTANCE_DETECTION_H

#include "mm_local_motion_planning/head_file.h"

using namespace fcl;

namespace agv_distance_detection
{
    class agv_distance_detection
    {
    public:
        agv_distance_detection(const string& agv_group_name_);
        double distanceToObstacles(const std::vector<double>& agv_joint_values);
        // 给定当前时刻移动机器人的坐标，计算其到最近障碍物的法向量。其中dim指定法向量计算是是2还是3，2的话只考虑(x,y)，3的话考虑(x,y,yaw)
        std::vector<double> getObsNormalDirection(const std::vector<double>& current_agv_joint_values, const int& dim, 
                                                  const double& agv_movement_delta, const double& current_agv_distance);
    private:
        // moveit相关变量
        string PLANNING_GROUP;
        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr robot_model; 
        robot_state::RobotStatePtr robot_state;
        const robot_state::JointModelGroup* joint_model_group;
        moveit::planning_interface::MoveGroupInterface move_group;
        bool update = true;

        // 配置相关的参数，用户需要指定移动机器人对应的规划组的名称，移动机器人基座连杆名称和环境连杆名称
        // 之所以这样做，是为了兼容多机器人，因为每个机器人的连杆名称有所区别
        // 注意如果想使用该文件，以下三个都不能为空，即agv,env在urdf中都不能为空
        std::unordered_set<string> agv_link_name_set;
        std::unordered_set<string> env_link_name_set;   
        std::unordered_set<string> link_name_set;

        // fcl相关变量
        collision_detection::FCLManager agv_manager;
        collision_detection::FCLObject env_object_;
        std::vector<collision_detection::FCLGeometryConstPtr> geoms_;
        std::vector<collision_detection::FCLCollisionObjectConstPtr> fcl_objs_;
        // function
        bool loadGeomAndObj();
        void allocSelfCollisionBroadPhase(robot_state::RobotState& state, collision_detection::FCLManager& agv_manager);
        void constructFCLObject(robot_state::RobotState& state, collision_detection::FCLObject& agv_fcl_obj, collision_detection::FCLObject& env_fcl_obj);
    };
}
#endif