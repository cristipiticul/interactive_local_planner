#ifndef INTERACTIVE_LOCAL_PLANNER_H_
#define INTERACTIVE_LOCAL_PLANNER_H_


#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner/dwa_planner.h>

#include <sensor_msgs/PointCloud2.h>

#include "interactive_local_planner/obstacle_classifier.h"
#include "interactive_local_planner/global_planner_interaction.h"

namespace interactive_local_planner
{

enum InteractiveLocalPlannerState
{
    RUNNING, WAITING_FOR_OBSTACLE_TO_MOVE, GOING_BACK_AND_FORTH, GOING_AROUND_OBSTACLE
};

class InteractiveLocalPlanner: public nav_core::BaseLocalPlanner
{
public:
    InteractiveLocalPlanner();
    ~InteractiveLocalPlanner();
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool dwaComputeVelocityCommands(tf::Stamped<tf::Pose>& global_pose, geometry_msgs::Twist& cmd_vel, base_local_planner::Trajectory& resulting_trajectory);
    bool computeVelocityCommandsIgnoringObstacles(tf::Stamped<tf::Pose>& global_pose,
        geometry_msgs::Twist& cmd_vel, base_local_planner::Trajectory& resulting_trajectory);

    bool isInitialized()
    {
        return initialized_;
    }
private:
    bool collisionPoseIsFar(const base_local_planner::Trajectory& path_empty_costmap,
        Eigen::Vector2d& first_collision_pose);
    std::vector<geometry_msgs::PoseStamped> createLocalPlanFromTrajectory(
        const base_local_planner::Trajectory& trajectory);
    bool obstacleWasAvoided();
    void doNothing(geometry_msgs::Twist& cmd_vel);
    /**
     * @brief Callback to update the local planner's parameters based on dynamic reconfigure
     */
    void reconfigureCB(dwa_local_planner::DWAPlannerConfig &config, uint32_t level);

    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

    // for visualisation, publishers of global and local plan
    ros::Publisher g_plan_pub_, l_plan_pub_;

    base_local_planner::LocalPlannerUtil planner_util_;

    boost::shared_ptr<dwa_local_planner::DWAPlanner> dp_; ///< @brief The trajectory controller

    costmap_2d::Costmap2DROS* costmap_ros_;

    dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig> *dsrv_;
    dwa_local_planner::DWAPlannerConfig default_config_;
    bool setup_;
    tf::Stamped<tf::Pose> current_pose_;

    base_local_planner::LatchedStopRotateController latchedStopRotateController_;


    bool initialized_;

    // Threshold distance (in meters). If the robot is closer than this value to
    // a colliding pose (a pose in which the robot is in collision with an obstacle),
    // the robot stops.
    double min_distance_to_obstacle_;
    // Waiting time (in seconds) for the obstacle to move
    double time_to_wait_for_obstacle_to_move_;
    // The minimum distance after which we consider the robot has successfully avoided
    // the obstacle. The distance is measured in meters. The distance is measured from
    // the first robot pose that is in collision with an obstacle.
    double distance_after_which_obstacle_is_avoided_;
    // The minimum probability for which the robot will attempt to wait for it to move.
    // If the obstacle is less likely than this threshold to move, then it will be
    // avoided from the beginning.
    double min_probability_to_wait_;


    base_local_planner::OdometryHelperRos odom_helper_;
    std::string odom_topic_;


    InteractiveLocalPlannerState current_state_;
    // If the robot encounters an obstacle, it records the first time the obstacle is met.
    ros::Time wait_start_time_;
    ros::Time going_back_and_forth_start_time_;

    boost::shared_ptr<base_local_planner::ObstacleCostFunction> obstacle_cost_function_;
    costmap_2d::Costmap2D empty_costmap_;
    base_local_planner::LocalPlannerUtil planner_util_empty_costmap_;
    boost::shared_ptr<dwa_local_planner::DWAPlanner> dp_empty_costmap_;
    Eigen::Vector2d first_collision_pose_;

    ObstacleClassifier obstacle_classifier_;
    GlobalPlannerInteraction global_planner_interaction_;
};

}

#endif