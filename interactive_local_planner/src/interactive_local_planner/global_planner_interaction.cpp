#include "interactive_local_planner/global_planner_interaction.h"

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// see force_replanning_thread_
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/chrono.hpp>

namespace interactive_local_planner
{
  void GlobalPlannerInteraction::initialize()
  {
    ros::NodeHandle nh;
    is_obstacle_layer_enabled_ = true;
    obstacle_enabled_service_client_ =
      nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/global_costmap/obstacle_layer/set_parameters");

    state_ = READY;
    
    force_replanning_ = false;
    force_replanning_thread_ = NULL;
    force_replanning_service_client_ =
      nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
  }

  bool GlobalPlannerInteraction::isObstacleLayerEnabled()
  {
    return is_obstacle_layer_enabled_;
  }

  void GlobalPlannerInteraction::setObstacleLayerEnabled(bool enabled)
  {
    is_obstacle_layer_enabled_ = enabled;
    //todo: change state!
    
    // Use dynamic reconfigure to disable/enable obstacle layer
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter enabled_param;
    dynamic_reconfigure::Config conf;

    enabled_param.name = "enabled";
    enabled_param.value = enabled;
    conf.bools.push_back(enabled_param);

    srv.request.config = conf;
    obstacle_enabled_service_client_.call(srv);
  }

  void GlobalPlannerInteraction::update()
  {
/* todo

        if (global_costmap_has_obstacle_layer_enabled_) {
          global_planner_interaction_.setObstacleLayerEnabled(false);
          
          // force re-planning of global planner
          global_planner_interaction_.setForceGlobalReplanning(true);
          return false;
        }
        if (force_global_replanning_) {
          if (!global_planner_interaction_.setForceGlobalReplanning(false)) {
            return false;
          }
        }
        */
  }

  bool GlobalPlannerInteraction::isReady()
  {
    return state_ == READY;
  }

  bool GlobalPlannerInteraction::setForceGlobalReplanning(bool enable)
  {
    if (force_replanning_thread_ != NULL) {
      if (!force_replanning_thread_->try_join_for(boost::chrono::milliseconds(100))) {
        ROS_WARN("InteractiveLocalPlanner: Force global replanning thread has not finished yet!");
        return false;
      }
      force_replanning_thread_ = NULL;
    }
    force_replanning_ = enable;
    if (enable) {
      if (!ros::param::get("/move_base/controller_patience", old_controller_patience_)) {
        ROS_WARN("InteractiveLocalPlanner: Could not get controller_patience. Using default 3.0.");
        old_controller_patience_ = 3.0;
      }
    }
    double new_patience = enable ? 0.0 : old_controller_patience_;
    force_replanning_thread_ = new boost::thread(
      boost::bind(
        &GlobalPlannerInteraction::reallySetForceGlobalReplanning, this, new_patience
    ));
    return true;
  }

  void GlobalPlannerInteraction::reallySetForceGlobalReplanning(double patience)
  {
    // Use dynamic reconfigure to disable/enable obstacle layer
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::DoubleParameter controller_patience_param;
    dynamic_reconfigure::Config conf;

    controller_patience_param.name = "controller_patience";
    controller_patience_param.value = patience;
    ROS_DEBUG("InteractiveLocalPlanner: setting controller_patience to %f", controller_patience_param.value);
    conf.doubles.push_back(controller_patience_param);

    srv.request.config = conf;
    force_replanning_service_client_.call(srv);
  }
}