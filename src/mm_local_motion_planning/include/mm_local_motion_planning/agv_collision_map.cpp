#include "mm_local_motion_planning/agv_collision_map.h"

namespace agv_collision_map
{
    agv_collision_map::agv_collision_map(const double& maxX_, const double& maxY_, const double& resolution_, 
                                         const double& originX_, const double& originY_, const double& collisionThreshold_)
    {
        resolution = resolution_;
        originX = originX_;
        originY = originY_;
        collisionThreshold = collisionThreshold_;
        width = maxX_ / resolution;
        height = maxY_ / resolution;
    }

    agv_collision_map::agv_collision_map(const string& mapName)
    {
        loadCollisionMap(mapName);
    }

    bool agv_collision_map::loadCollisionMap(const string& mapName)
    {
        // 加载环境能碰撞地图，同时获取元数据
        ROS_WARN("Loading collision map, please wait");
        std::string dir_package = ros::package::getPath("mm_local_motion_planning");
        std::string dir_param_file = dir_package + "/map/" + mapName;
               
        YAML::Node config = YAML::LoadFile(dir_param_file);

        // read meta data
        resolution = config["resolution"].as<double>();
        originX = config["originX"].as<double>();
        originY = config["originY"].as<double>();
        width = config["width"].as<double>();
        height = config["height"].as<double>();

        collisionMap = config["map"].as<std::vector<bool>>();
        
        ROS_WARN_STREAM("Collision map " << mapName << " has been loaded!");

        return true;
    }

    void agv_collision_map::publishMap()
    {
        nav_msgs::OccupancyGrid rosMap;
        rosMap.info.resolution = resolution;
        rosMap.info.width = width;
        rosMap.info.height = height;
        rosMap.info.origin.position.x = originX;
        rosMap.info.origin.position.y = originY;
        rosMap.info.origin.orientation.w = 1;

        for(int i = 0; i < collisionMap.size(); ++i)
        {
            if(collisionMap[i] == true)
                rosMap.data.push_back(99);
            else
                rosMap.data.push_back(0);
        }
        
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("gridMap", 1, true);
        ROS_WARN("Publishing collision map");

        ros::Rate loop_rate(1);
        while (ros::ok())
        {
            rosMap.header.stamp = ros::Time::now();
            rosMap.header.frame_id = "world";
            pub.publish(rosMap);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    bool agv_collision_map::generateCollisionMap(const string& mapName, const string& agvGroupName)
    {
        ROS_WARN("Generating collision map, please wait!");
        collisionMap.clear();
        agv_distance_detection::agv_distance_detection add(agvGroupName);
        ros::Time startTime = ros::Time::now();
        for(int row = 0; row < height; ++row)         // 先循环行
        {
            const double y = row * resolution + originY;
            for(int col = 0; col < width; ++col)     // 再循环列
            {
                // 计算栅格点在世界坐标系下面的坐标
                const double x = col * resolution + originX;
                std::vector<double> agv_joint_values = {x, y, 0};   // agv的碰撞模型用圆形表示，因此在建立环境地图的时候暂时不考虑旋转，如果机器人的形状和圆形差距较大，则需要考虑旋转
                double distance = add.distanceToObstacles(agv_joint_values);
                bool collisionState = distance > collisionThreshold ? true : false;
                collisionMap.push_back(collisionState);
            }
        }
        ROS_INFO_STREAM("Time cost for generating the map: "<<(ros::Time::now() - startTime).toSec()<<"s");
        // save the collision map
        std::string dir_package = ros::package::getPath("mm_local_motion_planning");
        std::string dir_param_file = dir_package + "/map/" + mapName;
        std::ofstream fout(dir_param_file);

        YAML::Emitter out(fout);
        out << YAML::BeginMap;

        // meta data
        out << YAML::Key << "resolution";
        out << YAML::Value << resolution;

        out << YAML::Key << "originX";
        out << YAML::Value << originX;
    
        out << YAML::Key << "originY";
        out << YAML::Value << originY;

        out << YAML::Key << "width";
        out << YAML::Value << width;
    
        out << YAML::Key << "height";
        out << YAML::Value << height;

        out << YAML::Key << "map";
        out << YAML::BeginSeq;
        for(int i = 0; i < collisionMap.size(); ++i)
            out << YAML::Value << collisionMap[i];
        out << YAML::EndSeq;

        out << YAML::EndMap;
        fout.close();
        ROS_INFO_STREAM("map size: "<<collisionMap.size());
        ROS_INFO_STREAM("The map has been writen to "<<dir_param_file);
    }

    bool agv_collision_map::isCollision(const double& x_, const double& y_)
    {
        // 将给定的世界坐标系下的坐标转换成栅格坐标系下面的行和列
        const int row = (y_ - originY) / resolution;
        const int col = (x_ - originX) / resolution;
        if(row < 0 || col < 0 || row > height || col > width)
        {
            return false;
        }
        const int index = col + row * width;
        return collisionMap[index];
    }
}
