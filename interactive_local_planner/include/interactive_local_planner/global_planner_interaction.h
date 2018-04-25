#ifndef GLOBAL_PLANNER_INTERACTION_H_
#define GLOBAL_PLANNER_INTERACTION_H_

#include <ros/ros.h>
#include <boost/thread.hpp>

namespace interactive_local_planner
{
  enum GlobalPlannerInteractionsState {
    READY
  };
  
  class GlobalPlannerInteraction
  {
  public:
    void initialize();
    bool isObstacleLayerEnabled();
    void setObstacleLayerEnabled(bool enabled);
    bool isReady();
    void update();
  private:
    bool setForceGlobalReplanning(bool enable);
    void reallySetForceGlobalReplanning(double patience);

    bool is_obstacle_layer_enabled_;
    ros::ServiceClient obstacle_enabled_service_client_;

    volatile GlobalPlannerInteractionsState state_;

    bool force_replanning_;
    boost::thread* force_replanning_thread_;
    ros::ServiceClient force_replanning_service_client_;
    double old_controller_patience_;
  };
}

#endif