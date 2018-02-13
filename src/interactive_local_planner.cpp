#include "interactive_local_planner/interactive_local_planner.h"

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <nav_msgs/Path.h>

// Threshold distance (in meters). If the robot is closer than this value to
// a colliding pose (a pose in which the robot is in collision with an obstacle),
// the robot stops.
#define MIN_DISTANCE_TO_FIRST_OBSTACLE 0.1
// Waiting time (in seconds) for the obstacle to move
#define WAIT_FOR_OBSTACLE_TO_MOVE_TIME 5.0

PLUGINLIB_EXPORT_CLASS(interactive_local_planner::InteractiveLocalPlanner, nav_core::BaseLocalPlanner)

namespace interactive_local_planner
{

using namespace dwa_local_planner;
using namespace base_local_planner;

  void InteractiveLocalPlanner::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_trans_vel = config.max_trans_vel;
      limits.min_trans_vel = config.min_trans_vel;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_rot_vel = config.max_rot_vel;
      limits.min_rot_vel = config.min_rot_vel;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_limit_trans = config.acc_limit_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.rot_stopped_vel = config.rot_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);
      planner_util_empty_costmap_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
      dp_empty_costmap_->reconfigure(config);

      obstacle_cost_function_->setScale(costmap_ros_->getCostmap()->getResolution() * config.occdist_scale);
      obstacle_cost_function_->setParams(config.max_trans_vel, config.max_scaling_factor, config.scaling_speed);
      obstacle_cost_function_->setSumScores(true);
  }

  InteractiveLocalPlanner::InteractiveLocalPlanner() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }

  void InteractiveLocalPlanner::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      // Copy the costmap, but clear the contents
      empty_costmap_ = *costmap;
      empty_costmap_.resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());
      planner_util_empty_costmap_.initialize(tf, &empty_costmap_, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));
      obstacle_cost_function_ = boost::shared_ptr<ObstacleCostFunction>(new ObstacleCostFunction(costmap_ros_->getCostmap()));
      obstacle_cost_function_->setFootprint(costmap_ros_->getRobotFootprint());
      dp_empty_costmap_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name + "_empty_costmap", &planner_util_empty_costmap_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      initialized_ = true;

      current_state_ = RUNNING;

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&InteractiveLocalPlanner::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool InteractiveLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan) && dp_empty_costmap_->setPlan(orig_global_plan);
  }

  bool InteractiveLocalPlanner::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    // TODO: check: this was the initial line:
    // if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
    if(latchedStopRotateController_.isGoalReached(&planner_util_empty_costmap_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void InteractiveLocalPlanner::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void InteractiveLocalPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  InteractiveLocalPlanner::~InteractiveLocalPlanner(){
    //make sure to clean things up
    delete dsrv_;
  }



  bool InteractiveLocalPlanner::dwaComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel)
  {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();
    
    // we need to call dp_->findBestPath at least once before checkTrajectory()
    // to set up the footprint
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds, costmap_ros_->getRobotFootprint());
    
    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if (path.cost_ < 0)
    {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p =
              tf::Stamped<tf::Pose>(tf::Pose(
                      tf::createQuaternionFromYaw(p_th),
                      tf::Point(p_x, p_y, 0.0)),
                      ros::Time::now(),
                      costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    publishLocalPlan(local_plan);
    return true;
  }

  bool InteractiveLocalPlanner::computeVelocityCommandsIgnoringObstacles(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel)
  {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    empty_costmap_.updateOrigin(costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getOriginY());

    //compute what trajectory to drive along
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    // call with updated footprint
    base_local_planner::Trajectory path_empty_costmap = dp_empty_costmap_->findBestPath(global_pose, robot_vel, drive_cmds, costmap_ros_->getRobotFootprint());

    // there is an obstacle on the path
    if (obstacle_cost_function_->scoreTrajectory(path_empty_costmap) < 0)
    {
      // Check if the obstacle is close to the robot
      // Find the first point of the trajectory that is in collision
      Eigen::Vector2d first_obstacle;
      Trajectory partial_trajectory(path_empty_costmap.xv_, path_empty_costmap.yv_, path_empty_costmap.thetav_, path_empty_costmap.time_delta_, path_empty_costmap.getPointsSize());
      for (size_t i = 0; i < path_empty_costmap.getPointsSize(); i++)
      {
        double x, y, th;
        path_empty_costmap.getPoint(i, x, y, th);
        partial_trajectory.addPoint(x, y, th);
        if (obstacle_cost_function_->scoreTrajectory(partial_trajectory) < 0)
        {
          first_obstacle[0] = x;
          first_obstacle[1] = y;
          break;
        }
      }

      Eigen::Vector2d first_point;
      double not_used_theta;
      path_empty_costmap.getPoint(0, first_point[0], first_point[1], not_used_theta);
      Eigen::Vector2d difference_vector(first_obstacle - first_point);
      double distance = std::sqrt(difference_vector.dot(difference_vector));
      if (distance < MIN_DISTANCE_TO_FIRST_OBSTACLE)
      {
        return false;
      }
    }
    
    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if (path_empty_costmap.cost_ < 0)
    {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path_empty_costmap.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path_empty_costmap.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p =
              tf::Stamped<tf::Pose>(tf::Pose(
                      tf::createQuaternionFromYaw(p_th),
                      tf::Point(p_x, p_y, 0.0)),
                      ros::Time::now(),
                      costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
  }




  bool InteractiveLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    base_local_planner::LocalPlannerUtil* current_planner_util;
    boost::shared_ptr<dwa_local_planner::DWAPlanner> current_dp;
    if (current_state_ == GOING_AROUND_OBSTACLE)
    {
      current_planner_util = &planner_util_;
      current_dp = dp_;
    }
    else
    {
      current_planner_util = &planner_util_empty_costmap_;
      current_dp = dp_empty_costmap_;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( !current_planner_util->getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan);
    dp_empty_costmap_->updatePlanAndLocalCosts(current_pose_, transformed_plan);
    
    if (latchedStopRotateController_.isPositionReached(current_planner_util, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = current_planner_util->getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          current_dp->getSimPeriod(),
          current_planner_util,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, current_dp, _1, _2, _3));
    } else {
      if (current_state_ == RUNNING)
      {
        if (computeVelocityCommandsIgnoringObstacles(current_pose_, cmd_vel))
        // no obstacles
        {
          return true;
        }
        else
        {
          ROS_INFO("Found an obstacle on the path. Waiting for it to move...");
          current_state_ = WAITING_FOR_OBSTACLE_TO_MOVE;
          wait_time_start_ = ros::Time::now();
        }
      }

      if (current_state_ == WAITING_FOR_OBSTACLE_TO_MOVE)
      {
        // Check if the obstacle moved...
        if (computeVelocityCommandsIgnoringObstacles(current_pose_, cmd_vel))
        {
          ROS_INFO("The obstacle moved! We continue pursuing the trajectory...");
          current_state_ = RUNNING;
          return true;
        }

        if (ros::Time::now() - wait_time_start_ <= ros::Duration(WAIT_FOR_OBSTACLE_TO_MOVE_TIME))
        {
          cmd_vel.linear.x = 0;
          cmd_vel.linear.y = 0;
          cmd_vel.angular.z = 0;

          return true;
        }
        else
        {
          ROS_INFO("The obstacle did not move. Going around it...");
          current_state_ = GOING_AROUND_OBSTACLE;
        }
      }

      if (current_state_ == GOING_AROUND_OBSTACLE)
      {
        // TODO: check when the obstacle was successfuly avoided
        // Tip: maybe use a fixed distance between the first collision point
        //      and robot's current pose.
        bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
        if (isOk) {
          publishGlobalPlan(transformed_plan);
        } else {
          ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
          std::vector<geometry_msgs::PoseStamped> empty_plan;
          publishGlobalPlan(empty_plan);
        }
        return isOk;
      }

      ROS_ERROR("InteractiveLocalPlanner: Invalid state! This point should never be reached.");
      return false;
    }
  }
}
