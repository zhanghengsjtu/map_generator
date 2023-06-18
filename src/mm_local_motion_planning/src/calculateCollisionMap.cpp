#include "mm_local_motion_planning/agv_collision_map.h"

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        ROS_ERROR("You should specify write or read the the collison map");
        ROS_ERROR_STREAM("The first param generates the map, the second param reads the map, and the third (optional) param specifies the map name!");
        ROS_ERROR_STREAM("Example1 rosrun mm_local_motion_planning calculateCollisionMap 1 1 envA");
        ROS_ERROR_STREAM("Example2 rosrun mm_local_motion_planning calculateCollisionMap 0 1 envA.yaml");
        return -1;
    }

    ros::init(argc, argv, "collisionMap");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const int write = atoi(argv[1]);
    const int read = atoi(argv[2]);
    string mapName = "collisionMap.yaml";
    if(argc == 4)
    {
        mapName = argv[3];
        if(mapName.find("yaml") == std::string::npos)
        {
            mapName += ".yaml";
        }
    }
    if(write == 1)
    {
        // agv_collision_map(const double maxX_, const double maxY_, const double resolution_, const double originX_, const double originY_, const double collisionThreshold_);
        agv_collision_map::agv_collision_map acm(25, 15, 0.05, -3, -7.5, 0.2);
        acm.generateCollisionMap(mapName, "robot1_agv");
    }
    if(read == 1)
    {
        agv_collision_map::agv_collision_map acm(mapName);
        acm.publishMap();
    }
    spinner.stop();
    ros::shutdown();
    return 0;
}