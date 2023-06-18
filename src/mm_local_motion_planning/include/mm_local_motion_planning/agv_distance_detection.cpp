#include "mm_local_motion_planning/agv_distance_detection.h"

namespace agv_distance_detection
{
    agv_distance_detection::agv_distance_detection(const string& agv_group_name_):
                            PLANNING_GROUP(agv_group_name_), 
                            robot_model_loader("robot_description"),
                            robot_model(robot_model_loader.getModel()), 
                            robot_state(new robot_state::RobotState(robot_model)),
                            joint_model_group(robot_state->getJointModelGroup(PLANNING_GROUP)) 
                            ,move_group(PLANNING_GROUP)
    {
        robot_state = move_group.getCurrentState(); 
        std::vector<string> link_names = joint_model_group->getLinkModelNames();
        for(auto name:link_names)
        {
            if(name.find("agv_base_link") != string::npos)  // 移动机器人的模型名字中必须包含agv_base_link
            {
                agv_link_name_set.insert(name);
                link_name_set.insert(name);
            }
        }
        if(agv_link_name_set.empty())
        {
            ROS_ERROR("There are no link name including agv_base_link");
            exit(-1);
        }
        link_names = robot_state->getRobotModel()->getLinkModelNamesWithCollisionGeometry();   // 返回urdf中所有具有物理模型的连杆名称
        for(auto name:link_names)
        {
            if(name.find("env") != string::npos)    // 注意移动机器人只和环境进行碰撞检测，而且环境的模型名字中必须包含env，因此可以把障碍物的名字中也包含env
            {
                env_link_name_set.insert(name);
                link_name_set.insert(name);
            }
        }
        if(env_link_name_set.empty())
        {
            ROS_ERROR("There are no link name including env");
            exit(-1);
        }
        // 用户在设置连杆名称的时候需要特别注意，连杆名称具有可搜索性和唯一性
        loadGeomAndObj();
    }

    double agv_distance_detection::distanceToObstacles(const std::vector<double>& agv_joint_values)
    {
        if(agv_joint_values.size() != 3)    {ROS_ERROR("Joint value format is wrong!"); return 0;}
        robot_state->setJointGroupPositions(joint_model_group, agv_joint_values);
        robot_state->update(true);   // 必须强制更新，否则距离检测模型出错
        allocSelfCollisionBroadPhase(*robot_state, agv_manager);
        collision_detection::DistanceRequest request;
        collision_detection::DistanceResult result;
        collision_detection::DistanceData drd(&request, &result); 
        agv_manager.manager_->distance(env_object_.collision_objects_[0].get(), &drd, &collision_detection::distanceCallback);
        double distance = drd.res->minimum_distance.distance;
        return distance;
    }

    void agv_distance_detection::allocSelfCollisionBroadPhase(robot_state::RobotState& state, collision_detection::FCLManager& agv_manager)
    {
        agv_manager.manager_.reset(new fcl::DynamicAABBTreeCollisionManager());
        constructFCLObject(state, agv_manager.object_, env_object_); 
        agv_manager.object_.registerTo(agv_manager.manager_.get());
    }

    void agv_distance_detection::constructFCLObject(robot_state::RobotState& state, collision_detection::FCLObject& agv_fcl_obj, collision_detection::FCLObject& env_fcl_obj)
    {
        agv_fcl_obj.collision_objects_.clear();
        env_fcl_obj.collision_objects_.clear();
        std::vector<string> link_names(geoms_.size());
        link_names = state.getRobotModel()->getLinkModelNamesWithCollisionGeometry();

        fcl::Transform3f fcl_tf;
        for (std::size_t i = 0; i < geoms_.size(); ++i)
        {
            if (link_name_set.find(link_names[i]) != link_name_set.end() && geoms_[i] && geoms_[i]->collision_geometry_)
            {
                collision_detection::transform2fcl(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link, geoms_[i]->collision_geometry_data_->shape_index), fcl_tf);
                std::shared_ptr<fcl::CollisionObject> collObj(new fcl::CollisionObject(*fcl_objs_[i])); // 智能指针内部会自动引用计数，并自动释放,之前用new的方法可能会造成内存泄漏
                collObj->setTransform(fcl_tf);
                collObj->computeAABB();
                if(agv_link_name_set.find(link_names[i]) != agv_link_name_set.end())     
                    agv_fcl_obj.collision_objects_.push_back(collObj);
                if(env_link_name_set.find(link_names[i]) != env_link_name_set.end())                
                    env_fcl_obj.collision_objects_.push_back(collObj);
            }
        }
    }

    bool agv_distance_detection::loadGeomAndObj()
    {
        const std::vector<const robot_model::LinkModel*>& links = robot_model->getLinkModelsWithCollisionGeometry();
        geoms_.resize(robot_model->getLinkGeometryCount());
        fcl_objs_.resize(robot_model->getLinkGeometryCount());
        for (auto link : links)
        {
            collision_detection::FCLGeometryConstPtr g = collision_detection::createCollisionGeometry(link->getShapes()[0], 1.0, 0, link, 0);
            std::size_t index = link->getFirstCollisionBodyTransformIndex();
            geoms_[index] = g;
            fcl_objs_[index] = collision_detection::FCLCollisionObjectConstPtr(new fcl::CollisionObject(g->collision_geometry_));
        }
    }
}
