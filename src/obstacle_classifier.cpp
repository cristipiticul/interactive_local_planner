#include "interactive_local_planner/obstacle_classifier.h"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace interactive_local_planner {

#define DEFAULT_CLOUD_TOPIC "/camera/depth/points"

using namespace std;
using namespace ros;
using namespace sensor_msgs;

ObstacleClassifier::ObstacleClassifier(): node_handle_(), initialized_(false) {
}

ObstacleClassifier::~ObstacleClassifier() {
}

void ObstacleClassifier::initialize(string node_name) {
    if (!isInitialized()) {
        NodeHandle private_node_handle("~/" + node_name);
        if (!private_node_handle.getParam("cloud_topic", cloud_topic_)) {
            ROS_ERROR("ObstacleClassifier: No cloud topic provided! Please set the /InteractiveLocalPlanner/cloud_topic parameter.");
        }
        if (!private_node_handle.getParam("mean_color_radius", mean_color_radius_)) {
            ROS_ERROR("ObstacleClassifier: No cloud topic provided! Please set the /InteractiveLocalPlanner/mean_color_radius parameter.");
        }
        initialized_ = true;
    } else {
        ROS_WARN("ObjectClassifier is already initialized! Doing nothing.");
    }
}

void ObstacleClassifier::pointCloudCallback(const PointCloud2::ConstPtr& msg) {
    point_cloud_subscriber_.shutdown();
    if (!got_cloud_) {
        got_cloud_ = true;

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg<pcl::PointXYZRGB>(*msg, cloud);

        // Find the closest point to the collision point
        double min_dist = 999.0;
        Eigen::Vector3d min_dist_point;
        for (size_t i = 0; i != cloud.points.size(); i++) {
            pcl::PointXYZRGB p = cloud.points[i];
            Eigen::Vector3d p_eigen;
            p_eigen[0] = p.x;
            p_eigen[1] = p.y;
            p_eigen[2] = p.z;

            double dist = (p_eigen - collided_pose_).norm();
            if (dist <= min_dist) {
                min_dist = dist;
                min_dist_point = p_eigen;
            }
        }

        // Get all the points around the closest point
        for (size_t i = 0; i != cloud.points.size(); i++) {
            pcl::PointXYZRGB p = cloud.points[i];
            Eigen::Vector3d p_eigen;
            p_eigen[0] = p.x;
            p_eigen[1] = p.y;
            p_eigen[2] = p.z;

            double distance = (p_eigen - min_dist_point).norm();
            if (distance <= mean_color_radius_) {
                //TODO: add to a vector
            }
        }

        //TODO: average color + classify

        result_ = 0;

        processed_cloud_ = true;
    }
}

int ObstacleClassifier::classifyObstacle(const Eigen::Vector3d& collided_pose) {
    got_cloud_ = false;
    processed_cloud_ = false;
    collided_pose_ = collided_pose;
    point_cloud_subscriber_ = node_handle_.subscribe(cloud_topic_, 1, &ObstacleClassifier::pointCloudCallback, this);

    while (!processed_cloud_) {
        ros::Duration(0.1).sleep();
    }

    return result_;
}

bool ObstacleClassifier::isInitialized() {
    return initialized_;
}

}