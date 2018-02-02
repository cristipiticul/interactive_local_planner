#ifndef INTERACTIVE_LOCAL_PLANNER_H_
#define INTERACTIVE_LOCAL_PLANNER_H_

#include <nav_core/base_local_planner.h>

namespace interactive_local_planner
{

class InteractiveLocalPlanner: public nav_core::BaseLocalPlanner
{
public:
    InteractiveLocalPlanner();
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::vector<geometry_msgs::PoseStamped> plan_;
    std::vector<tf::Pose> plan_tf_;
};

}

#endif