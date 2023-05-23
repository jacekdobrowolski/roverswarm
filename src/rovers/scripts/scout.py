#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from roversim.msg import Pose

from roversim.srv import Kill, Spawn


class Scout:
    def __init__(self):
        rospy.loginfo(f"{dir(rospy)}")
        self.x, self.y = self.get_initial_position()
        self.command_publisher = self.attach_to_roversim()
        rospy.on_shutdown(self.detach_from_roversim)
        self.creep_forward()


    def attach_to_roversim(self):
        rospy.loginfo('Waiting for service roversim/spawn')
        rospy.wait_for_service('spawn')
        rospy.loginfo('service Spawn available')
        start_position_param_name = rospy.search_param('start_position')
        start_position = rospy.get_param(start_position_param_name)
        spawn_service = rospy.ServiceProxy('spawn', Spawn)
        try:
            rover_name = spawn_service(start_position['x'], start_position['y'], 0, rospy.get_name()).name
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            rospy.signal_shutdown()
        command_publisher = rospy.Publisher(rover_name + '/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber(rover_name + '/pose', Pose, self.update_position)
        return command_publisher


    def detach_from_roversim(self):
        kill_service = rospy.ServiceProxy('kill', Kill)
        try:
            _ = kill_service(rospy.get_name())
            rospy.loginfo(f'Killed {rospy.get_name()}')
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def get_initial_position(self):
        x_param_name = rospy.search_param('start_position/x')
        x = rospy.get_param(x_param_name)
        y_param_name = rospy.search_param('start_position/y')
        y = rospy.get_param(y_param_name)
        return x, y


    def update_position(self, pose):
        self.x = int(pose.x)
        self.y = int(pose.y)


    def creep_forward(self):
        twist = Twist()
        twist.linear.y = 10
        while not rospy.is_shutdown():
            if self.y < 600:
                self.command_publisher.publish(twist)
                rospy.sleep(1)
            else:
                rospy.signal_shutdown()


if __name__ == '__main__':
    rospy.init_node('scout')
    scout = Scout()
    rospy.spin()
