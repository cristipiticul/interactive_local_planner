#ifndef OBSTACLE_CLASSIFIER_H_
#define OBSTACLE_CLASSIFIER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>

namespace interactive_local_planner
{

class ObstacleClassifier {
public:
    ObstacleClassifier();
    ~ObstacleClassifier();
    
    void initialize(std::string name);
    int classifyObstacle(const Eigen::Vector3d& collidedPose);
    bool isInitialized();
private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    std::string cloud_topic_;
    ros::NodeHandle node_handle_;
    ros::Subscriber point_cloud_subscriber_;
    double mean_color_radius_;

    // The XYZ coordinates of the robot where there is a collision, in the camera frame.
    Eigen::Vector3d collided_pose_;
    
    bool initialized_;
    volatile bool got_cloud_;
    volatile bool processed_cloud_;
    volatile int result_;
};

}

#endif