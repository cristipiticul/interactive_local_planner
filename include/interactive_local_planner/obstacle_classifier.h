#ifndef OBSTACLE_CLASSIFIER_H_
#define OBSTACLE_CLASSIFIER_H_
#include <ros/ros.h>
#include <Eigen/Core>
#include <tf/transform_listener.h>

namespace interactive_local_planner
{

struct Obstacle {
    int id;
    std::string frame;
    double move_probability;
    Eigen::Vector2d position;
};

class ObstacleClassifier {
public:
    ObstacleClassifier();
    ~ObstacleClassifier();
    
    void initialize(std::string name);
    bool findObstacleCloseTo(const Eigen::Vector2d& collision_position,
        Obstacle& obstacle);
    bool isInitialized();
private:
    void getObstaclePoses(std::vector<tf::StampedTransform>& obstacles,
        std::vector<bool>& obstacles_found);
    bool findClosestObstacle(const std::vector<tf::StampedTransform>& obstacles,
        const std::vector<bool>& obstacles_found, const Eigen::Vector2d& search_point,
        int& min_distance_i, double& min_distance, Eigen::Vector2d& obstacle_position);
    std::string map_frame_;
    std::vector<std::string> obstacles_frames_;
    std::vector<double> obstacles_move_probabilities_;
    double robot_base_radius_;
    
    bool initialized_;

    tf::TransformListener tf_listener_;
};

}

#endif