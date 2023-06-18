#ifndef AGV_COLLISION_MAP_H
#define AGV_COLLISION_MAP_H

#include "mm_local_motion_planning/agv_distance_detection.h"

/*
1. 为移动机器人构建环境地图
2. 加载已经构建好的环境地图，便于为移动机器人进行碰撞检测
*/

namespace agv_collision_map
{
    class agv_collision_map
    {
    public:
        agv_collision_map(const double& maxX_, const double& maxY_, const double& resolution_, 
                          const double& originX_, const double& originY_, const double& collisionThreshold_);
        agv_collision_map(const string& mapName);
        void publishMap();
        bool generateCollisionMap(const string& mapName, const string& agvGroupName);
        bool isCollision(const double& x_, const double& y_);
    private:
        // 注意地图是以环境的左下角为起点的
        double originX;                   // 栅格原点在世界坐标系下面的坐标x
        double originY;                   // 栅格原点在世界坐标系下面的坐标y
        double resolution;                // 栅格地图分辨率
        double collisionThreshold;        // 碰撞的阈值，即距离物体多远算碰撞
        int width;                        // 地图栅格宽度
        int height;                       // 地图栅格高度
        std::vector<bool> collisionMap;   // 栅格地图

        bool loadCollisionMap(const string& mapName);
    };
}

#endif