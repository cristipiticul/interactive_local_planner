#! /usr/bin/env python
import rospy
import math
import json
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, ApplyJointEffort, ApplyJointEffortRequest
import tf

MOVING_DURATION = rospy.Duration(3.0)
FREQUENCY = 5.0 # in Hz - how often to check the distance between robot and obstacle

def vector_length(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

class ObstacleController:
    def __init__(self, robot_link, obstacle_link, distance_threshold, joints_to_move):
        self.robot_link = robot_link
        self.obstacle_link = obstacle_link
        self.distance_threshold = distance_threshold
        self.joints_to_move = joints_to_move
        self.moving = False
        self.get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.apply_joint_effort_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Timer(rospy.Duration(1.0 / FREQUENCY), self.move_if_robot_is_close)

    def move_if_robot_is_close(self, event):
        request = GetLinkStateRequest()
        request.link_name = self.obstacle_link
        request.reference_frame = self.robot_link
        response = self.get_link_state_service.call(request)
        if not response.success:
            rospy.logerr('obstacle_controller: Problem when getting relative pose between robot and obstacle')
            rospy.logerr('obstacle_controller: Details: %s', response.status_message)
            return

        relative_position = response.link_state.pose.position
        distance = vector_length(relative_position)
        if not self.moving and distance <= self.distance_threshold:
            self.moving = True
            request = ApplyJointEffortRequest()
            for joint in self.joints_to_move:
                request.effort = 1.0
                request.joint_name = joint
                request.duration = MOVING_DURATION
                request.start_time = rospy.Time(0) # start now
                response = self.apply_joint_effort_service(request)
                if not response.success:
                    rospy.logerr('obstacle_controller: Problem when moving joint %s', joint)
                    rospy.logerr('obstacle_controller: Details: %s', response.status_message)
                    return
                rospy.sleep(MOVING_DURATION)
                self.moving = False

TF_PUBLISHER_FREQUENCY = 10.0 # in Hz - how often to publish the obstacle position to TF
class ObstaclePosePublisher:
    def __init__(self, robot_link, obstacle_link, robot_tf_frame, published_tf_frame):
        self.robot_link = robot_link
        self.obstacle_link = obstacle_link
        self.robot_tf_frame = robot_tf_frame
        self.published_tf_frame = published_tf_frame
        self.get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(1.0 / TF_PUBLISHER_FREQUENCY), self.publish_position)
    
    def publish_position(self):
        request = GetLinkStateRequest()
        request.link_name = self.obstacle_link
        request.reference_frame = self.robot_link
        response = self.get_link_state_service.call(request)
        if not response.success:
            rospy.logerr('obstacle_controller: Problem when getting relative pose between robot and obstacle')
            rospy.logerr('obstacle_controller: Details: %s', response.status_message)
            return

        #todo: self.tf_broadcaster.sendTransform(...)


def main():
    rospy.init_node('obstacle_controller')
    
    distance_threshold = rospy.get_param('~distance_threshold')
    robot_link = rospy.get_param('~robot_link')
    obstacle_link = rospy.get_param('~obstacle_link')
    joints_to_move_str = rospy.get_param('~joints_to_move')
    joints_to_move = json.loads(joints_to_move_str)

    robot_tf_frame = rospy.get_param('~robot_tf_frame')
    published_tf_frame = rospy.get_param('~published_tf_frame')
    
    rospy.loginfo('obstacle_controller: Robot link: %s' % (robot_link,))
    rospy.loginfo('obstacle_controller: Obstacle link: %s' % (obstacle_link,))
    rospy.loginfo('obstacle_controller: Distance threshold: %f' % (distance_threshold,))
    rospy.loginfo('obstacle_controller: Joints to move: %s' % (str(joints_to_move),))
    
    ObstacleController(robot_link, obstacle_link, distance_threshold, joints_to_move)
    ObstaclePosePublisher(robot_link, obstacle_link, robot_tf_frame, published_tf_frame)
    
    rospy.spin()
    
    return 0


if __name__ == '__main__':
    main()
