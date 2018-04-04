#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

def main():
    rospy.init_node('obstacle_controller')
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    request = GetLinkStateRequest()
    request.link_name = 'moving_obstacle::chassis'
    request.reference_frame = 'world'
    response = get_link_state.call(request)

    print(response)

    return 0


if __name__ == '__main__':
    main()
