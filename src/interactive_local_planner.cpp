#include "interactive_local_planner/interactive_local_planner.h"

#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(interactive_local_planner::InteractiveLocalPlanner, nav_core::BaseLocalPlanner)

namespace interactive_local_planner
{

InteractiveLocalPlanner::InteractiveLocalPlanner()
{
    std::cout << "constructor!\n";
}

bool InteractiveLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    tf::Stamped<tf::Pose> robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
        ROS_WARN("Could not get robot pose!");
        return false;
    }

    size_t closestPointIndex = 0;
    double minDist2 = tf::tfDistance2(plan_tf_[0].getOrigin(), robotPose.getOrigin());
    for (size_t i = 1; i < plan_.size(); i++)
    {
        double dist2 = tf::tfDistance2(plan_tf_[i].getOrigin(), robotPose.getOrigin());
        if (dist2 < minDist2)
        {
            minDist2 = dist2;
            closestPointIndex = i;
        }
    }

    tf::Pose destination;
    if (closestPointIndex < plan_tf_.size() - 1)
    {
        destination = plan_tf_[closestPointIndex + 1];
    }
    else
    {
        destination = plan_tf_[closestPointIndex];
    }

    cmd_vel.linear.x = 0.1;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = tf::getYaw(destination.getRotation()) - tf::getYaw(robotPose.getRotation());

    std::cout << "computeVelocityCommands!\n";
    return true;
}

bool InteractiveLocalPlanner::isGoalReached()
{
    std::cout << "isGoalReached!\n";
    return false;
}

bool InteractiveLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    /*
    for (int i = 0; i < plan.size(); i++)
    {
        std::cout << plan[i].header.frame_id << "\n";
        std::cout << plan[i].pose.position.x << " " << plan[i].pose.position.y << " " << plan[i].pose.position.z << "\n";
    }
    std::cout << "setPlan!\n";
    */
    plan_ = plan;
    plan_tf_.resize(plan.size());
    for (size_t i = 0; i < plan.size(); i++)
    {
        tf::poseMsgToTF(plan[i].pose, plan_tf_[i]);
    }
    return true;
}

void InteractiveLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    costmap_ros_ = costmap_ros;
}

}