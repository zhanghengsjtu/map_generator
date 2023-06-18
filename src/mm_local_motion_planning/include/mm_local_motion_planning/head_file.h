#ifndef DATA_TYPE_H
#define DATA_TYPE_H

// head file for bacis C++ function
#include <iostream>
#include "fstream"
#include "iomanip"
#include <vector>
#include <string>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

// head file for basic ros function
#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <moveit/planning_scene/planning_scene.h>
#include "nav_msgs/OccupancyGrid.h"

// head file for fcl
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include "fcl/config.h"

using std::cout;
using std::endl;
using std::string;
using std::setw;

#endif