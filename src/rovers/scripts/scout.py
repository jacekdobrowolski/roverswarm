#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from roversim.srv import Spawn, Kill
from geometry_msgs.msg import Twist


def spawn():
    rospy.loginfo('Waiting for service roversim/spawn')
    rospy.wait_for_service('spawn')
    rospy.loginfo('service Spawn available')
    start_position_param_name = rospy.search_param('start_position')
    start_posiotion = rospy.get_param(start_position_param_name)
    spawn_service = rospy.ServiceProxy('spawn', Spawn)
    try:
        resp1 = spawn_service(start_posiotion['x'], start_posiotion['y'], 0, rospy.get_name())
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))
        rospy.signal_shutdown()

    return resp1.name


def kill_rover():
    kill_service = rospy.ServiceProxy('kill', Kill)
    try:
        resp1 = kill_service(rospy.get_name())
        rospy.loginfo(f'Killed {rospy.get_name()}')
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))


def creep_forward():
    pub = rospy.Publisher(rover_name + '/cmd_vel', Twist, queue_size=1)
    twist = Twist()
    twist.linear.y = 0.1
    while not rospy.is_shutdown():
        pub.publish(twist)
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('scout')
    rospy.on_shutdown(kill_rover)
 
    rover_name = spawn()
    creep_forward()
    rospy.spin()
