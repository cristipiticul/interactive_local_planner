#include "interactive_local_planner/obstacle_classifier.h"

//TODO: update obstacle probability (variables + file write)

namespace interactive_local_planner {

using namespace std;
using namespace ros;
using namespace tf;

#define TF_TIMEOUT Duration(0.2)

ObstacleClassifier::ObstacleClassifier(): initialized_(false) {
}

ObstacleClassifier::~ObstacleClassifier() {
}

void ObstacleClassifier::initialize(string node_name) {
    if (!isInitialized()) {
        NodeHandle private_node_handle("~/" + node_name);
        bool ok = true;
        if (!private_node_handle.getParam("obstacles_frames", obstacles_frames_)) {
            ROS_ERROR("ObstacleClassifier: No obstacles frames provided! Please set the %s/obstacles_frames parameter.",
                node_name.c_str());
            ok = false;
        }
        if (!private_node_handle.getParam("obstacles_move_probabilities", obstacles_move_probabilities_)) {
            ROS_ERROR("ObstacleClassifier: No obstacles move probabilities provided! Please set the %s/obstacles_move_probabilities parameter.",
                node_name.c_str());
            ok = false;
        }
        if (!private_node_handle.getParam("map_frame", map_frame_)) {
            ROS_ERROR("ObstacleClassifier: No map frame provided! Please set the %s/map_frame parameter.",
                node_name.c_str());
            ok = false;
        }
        if (!private_node_handle.getParam("robot_base_radius", robot_base_radius_)) {
            ROS_ERROR("ObstacleClassifier: No robot base radius provided! Please set the %s/robot_base_radius parameter.",
                node_name.c_str());
            ok = false;
        }
        initialized_ = ok;
    } else {
        ROS_WARN("ObjectClassifier is already initialized! Doing nothing.");
    }
}

bool ObstacleClassifier::findObstacleCloseTo(const Eigen::Vector2d& collision_position,
    Obstacle& obstacle)
{
    if (!isInitialized()) {
        ROS_ERROR("ObstacleClassifier: classifyObstacle was called before initialization!");
        return 0.0;
    }
    vector<StampedTransform> obstacles;
    vector<bool> obstacles_found;
    int min_distance_i;
    double min_distance;
    Eigen::Vector2d obstacle_position;

    getObstaclePoses(obstacles, obstacles_found);
    
    bool found = findClosestObstacle(obstacles, obstacles_found, collision_position,
        min_distance_i, min_distance, obstacle_position);
    
    if (found && min_distance < robot_base_radius_) {
        obstacle.id = min_distance_i;
        obstacle.frame = obstacles_frames_[min_distance_i];
        obstacle.position = obstacle_position;
        obstacle.move_probability = obstacles_move_probabilities_[min_distance_i];
        return true;
    } else {
        return false;
    }
}

void ObstacleClassifier::getObstaclePoses(vector<StampedTransform>& obstacles,
    vector<bool>& obstacles_found)
{
    Time now = Time::now();
    obstacles.resize(obstacles_frames_.size());
    obstacles_found.resize(obstacles_frames_.size());
    for (size_t i = 0; i < obstacles_frames_.size(); i++) {
        try {
            tf_listener_.waitForTransform(map_frame_, obstacles_frames_[i], now, TF_TIMEOUT);
            tf_listener_.lookupTransform(map_frame_, obstacles_frames_[i], now, obstacles[i]);
            obstacles_found[i] = true;
        } catch (TransformException ex) {
            ROS_DEBUG("ObstacleClassifier: could not retrieve transform \"%s\"->\"%s\". Error: %s", obstacles_frames_[i].c_str(), map_frame_.c_str(), ex.what());
            ROS_DEBUG("This probably means the camera does not see the marker (it's ok)");
            obstacles_found[i] = false;
        }
    }
}

bool ObstacleClassifier::findClosestObstacle(const vector<StampedTransform>& obstacles,
    const vector<bool>& obstacles_found, const Eigen::Vector2d& search_point,
    int& min_distance_i, double& min_distance, Eigen::Vector2d& obstacle_position)
{
    bool found = false;
    for (size_t i = 0; i < obstacles.size(); i++) {
        if (obstacles_found[i]) {
            Eigen::Vector2d obstacle_eigen;
            obstacle_eigen << obstacles[i].getOrigin().getX(), obstacles[i].getOrigin().getY();
            double distance = (obstacle_eigen - search_point).norm();
            if (!found || distance < min_distance) {
                min_distance_i = i;
                min_distance = distance;
                obstacle_position = obstacle_eigen;
                found = true;
            }
        }
    }
    return found;
}

bool ObstacleClassifier::isInitialized() {
    return initialized_;
}

}