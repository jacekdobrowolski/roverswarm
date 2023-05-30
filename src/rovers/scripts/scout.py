#!/usr/bin/env python
from threading import Lock
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from roversim.msg import Pose
from roversim.srv import Kill, Spawn
from rovers.srv import AddToSwarm
from rovers.msg import Target


def log(message: str):
    rospy.loginfo(f'{rospy.get_name()} - {message}')


class Scout:
    def __init__(self):
        self.target_reached = False

        self.position_lock = Lock()
        self.position = self.get_initial_position()

        self.target_lock = Lock()
        self.target = self.get_initial_position()

        self.command_publisher = self.attach_to_roversim()
        self.attach_to_mission_planner()
        rospy.on_shutdown(self.detach_from_roversim)

        self.target_done_publisher = rospy.Publisher('/target_done', Target, queue_size=10)
        self.go_to_target()


    def attach_to_roversim(self):
        log(f'waiting for service roversim/spawn')
        rospy.wait_for_service('spawn')
        
        start_position_param_name = rospy.search_param('start_position')
        start_position = rospy.get_param(start_position_param_name)
        spawn_service = rospy.ServiceProxy('spawn', Spawn)
        try:
            self.rover_name = spawn_service(start_position['x'], start_position['y'], 0, rospy.get_name()).name
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            rospy.signal_shutdown(f'{rospy.get_name()} - spawn service exception')
        command_publisher = rospy.Publisher(self.rover_name + '/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber(f'{self.rover_name}/pose', Pose, self.update_position)
        log(f'attached to {self.rover_name}/cmd_vel and /pose')
        return command_publisher

    def attach_to_mission_planner(self):
        log(f'waiting for service add_to_swarm')
        rospy.wait_for_service('add_to_swarm')
        add_to_swarm_service = rospy.ServiceProxy('add_to_swarm', AddToSwarm)
        try:
            pose_channel = String(f'{self.rover_name}/pose')
            add_to_swarm_response = add_to_swarm_service(rover_pose_channel=pose_channel)
            rover_target_channel = str(add_to_swarm_response.rover_target_channel.data)
            rospy.Subscriber(rover_target_channel, Target, self.update_target)
            log(f'attached to {rover_target_channel}')
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            rospy.signal_shutdown(f'{rospy.get_name()} - add_to_swarm service exception')


    def detach_from_roversim(self):
        kill_service = rospy.ServiceProxy('kill', Kill)
        try:
            _ = kill_service(rospy.get_name())
            log(f'killed {rospy.get_name()}')
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))


    def get_initial_position(self):
        x_param_name = rospy.search_param('start_position/x')
        x = rospy.get_param(x_param_name)
        y_param_name = rospy.search_param('start_position/y')
        y = rospy.get_param(y_param_name)
        return np.array([x, y])


    def update_position(self, pose) -> np.ndarray:
        with self.position_lock:
            self.position[0] = int(pose.x)
            self.position[1] = int(pose.y)

    def update_target(self, target):
        with self.target_lock:
            self.target[0] = int(target.x)
            self.target[1] = int(target.y)
        self.target_reached = False
        with self.position_lock:
            log(f'new target {self.target} <- {self.position}')

    def creep_forward(self):
        twist = Twist()
        twist.linear.y = 10
        while not rospy.is_shutdown():
            if self.position[1] < 600:
                self.command_publisher.publish(twist)
                rospy.sleep(1)
            else:
                rospy.signal_shutdown()


    def go_to_target(self):
        twist = Twist()
        while not rospy.is_shutdown():
            if not self.target_reached:
                with self.target_lock:
                    with self.position_lock:
                        if np.allclose(self.position, self.target, atol=2):
                            twist.linear.x = 0
                            twist.linear.y = 0
                            self.target_done_publisher.publish(Target(*self.target))
                            self.target_reached = True
                            log(' target done let me goooooooooo')
                        else:
                            diff = self.target - self.position
                            diff = diff/np.linalg.norm(diff) * 20
                            twist.linear.x = diff[0]
                            twist.linear.y = diff[1]

            self.command_publisher.publish(twist)
            rospy.sleep(0.1)
        

if __name__ == '__main__':
    rospy.init_node('scout')
    scout = Scout()
    rospy.spin()
