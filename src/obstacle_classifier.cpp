#include "interactive_local_planner/obstacle_classifier.h"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//TODO: Use AR Markers to classify the obstacles

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
        bool ok = true;
        if (!private_node_handle.getParam("cloud_topic", cloud_topic_)) {
            ROS_ERROR("ObstacleClassifier: No cloud topic provided! Please set the %s/cloud_topic parameter.",
                node_name.c_str());
            ok = false;
        }
        if (!private_node_handle.getParam("mean_color_radius", mean_color_radius_)) {
            ROS_ERROR("ObstacleClassifier: No cloud topic provided! Please set the %s/mean_color_radius parameter.",
                node_name.c_str());
            ok = false;
        }
        if (!private_node_handle.getParam("camera_frame", camera_frame_)) {
            ROS_ERROR("ObstacleClassifier: No camera frame provided! Please set the %s/camera_frame parameter.",
                node_name.c_str());
            ok = false;
        }
        if (!private_node_handle.getParam("map_frame", map_frame_)) {
            ROS_ERROR("ObstacleClassifier: No map frame provided! Please set the %s/map_frame parameter.",
                node_name.c_str());
            ok = false;
        }
        initialized_ = ok;
    } else {
        ROS_WARN("ObjectClassifier is already initialized! Doing nothing.");
    }
}

void ObstacleClassifier::pointCloudCallback(const PointCloud2::ConstPtr& msg) {
    point_cloud_subscriber_.shutdown();
    if (!got_cloud_) {
        got_cloud_ = true;

        if (msg->header.frame_id != camera_frame_) {
            ROS_WARN("ObstacleClassifier: The camera frames are not the same. Cloud frame: \"%s\", from yaml: \"%s\"", 
                msg->header.frame_id.c_str(), camera_frame_.c_str());
        }

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
        vector<std_msgs::ColorRGBA> colors;
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

int ObstacleClassifier::classifyObstacle(const Eigen::Vector2d& collided_position_in_map_frame) {
    got_cloud_ = false;
    processed_cloud_ = false;
    if (!convertMapToCameraCoordinates(collided_position_in_map_frame, collided_pose_)) {
        ROS_WARN("ObstacleClassifier: Could not convert collision pose!");
        return -1;
    }
    point_cloud_subscriber_ = node_handle_.subscribe(cloud_topic_, 1, &ObstacleClassifier::pointCloudCallback, this);

    while (!processed_cloud_) {
        ros::Duration(0.1).sleep();
    }

    return result_;
}

bool ObstacleClassifier::convertMapToCameraCoordinates(const Eigen::Vector2d& position, Eigen::Vector3d& result) {
    geometry_msgs::PointStamped position_geom;
    geometry_msgs::PointStamped result_geom;
    bool ok = true;

    position_geom.header.frame_id = map_frame_;
    position_geom.header.stamp = ros::Time::now();
    position_geom.point.x = position[0];
    position_geom.point.y = position[1];
    position_geom.point.z = 0; //?
    try {
        tf_listener_.waitForTransform(camera_frame_, map_frame_, ros::Time::now(), ros::Duration(0.5));
        tf_listener_.transformPoint(camera_frame_, position_geom, result_geom);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("ObstacleClassifier: could not convert map to camera coordinates: %s", ex.what());
        ok = false;
    }
    result[0] = result_geom.point.x;
    result[1] = result_geom.point.y;
    result[2] = result_geom.point.z;
    return ok;
}

bool ObstacleClassifier::isInitialized() {
    return initialized_;
}

}