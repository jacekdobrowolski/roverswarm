from dataclasses import dataclass
from threading import Lock
from functools import partial

import numpy as np
from scipy.optimize import linear_sum_assignment
import rospy

from std_msgs.msg import String
from rovers.msg import Target
from rovers.srv import AddToSwarm, AddToSwarmResponse
from roversim.msg import Pose


def log(message: str):
    rospy.loginfo(f'{rospy.get_name()} - {message}')

@dataclass
class Rover():
    rover_target_publisher: rospy.Publisher
    name: str
    position: np.ndarray
    lock: Lock = Lock()

class MissionPlanner():
    
    def pose_callback(self, rover, pose):

        with rover.lock:
            # log(f'updating rover {rover.name} pose {rover.position}')
            rover.position[0] = int(pose.x)
            rover.position[1] = int(pose.y)

    def add_to_swarm_callback(self, req):
        rover_name = str(req.rover_pose_channel.data).split('/')[2]
        rover_pose_channel = str(req.rover_pose_channel.data)
        rover_target_channel = f'{rospy.get_name()}/target/{rover_name}'
        log(f'rover_target_channel "{rover_target_channel}"')
        rover_target_publisher = rospy.Publisher(str(rover_target_channel), Target, queue_size=1)
        rover = Rover(rover_target_publisher, rover_name, np.zeros((2), dtype=np.int))
        rospy.Subscriber(rover_pose_channel, Pose, partial(self.pose_callback, rover))
        log(f'subscribed to {rover_pose_channel}')

        self.rovers.append(rover)
        log(f'{rover_name} added')
        return AddToSwarmResponse(String(rover_target_channel))
    
    def get_targets(self):
        targets_param_name = rospy.search_param('targets')
        with self.targets_lock:
            targets_from_params = rospy.get_param(targets_param_name)
            for target in targets_from_params:
                x = target['x']
                y = target['y']
                self.targets.append(np.asarray([x, y]))
            log(f'targets: {len(self.targets)}')

    def assign_targets(self):
        cost_matrix = np.zeros((len(self.rovers), len(self.targets)))
        for i_rover, rover in enumerate(self.rovers):
            with rover.lock:
                for i_target, target in enumerate(self.targets):
                    cost_matrix[i_rover][i_target] = np.linalg.norm(rover.position - target)
        rover_indexes, target_indexes = linear_sum_assignment(cost_matrix)
        for target_index, rover_index in zip(target_indexes, rover_indexes):
            rover = self.rovers[rover_index]
            target = Target(*self.targets[target_index])
            with rover.lock:
                rover.rover_target_publisher.publish(target)
                log(f'{rover.name} {rover.position} -> {self.targets[target_index]} = {cost_matrix[rover_index][rover_index]}')
        log(f'targets calculated cost: {cost_matrix[rover_indexes, target_indexes].sum()}')

    def subscribe_targets_done(self):
        rospy.Subscriber('/target_done', Target, self.target_done_callback)
        log(f'subscribed to target_done')

    def target_done_callback(self, target_done_request):
        target_done = np.zeros((2), dtype=int)
        target_done[0] = int(target_done_request.x)
        target_done[1] = int(target_done_request.y)
        with self.targets_lock:
            try:
                for i , target in enumerate(self.targets):
                    if np.allclose(target, target_done, atol=20):
                        self.targets.pop(i)
                        log(f'deleted {target_done} targets length: {len(self.targets)}')
                        self.assign_targets()
            except ValueError as e:
                rospy.logerr(f'{e}')

    def __init__(self):
        self.rovers = []
        self.targets = []
        self.targets_lock  = Lock()
        rospy.Service('add_to_swarm', AddToSwarm, self.add_to_swarm_callback)
        log(f'add_to_swarm service started')
        self.get_targets()
        rospy.sleep(2)
        with self.targets_lock:
            self.assign_targets()

        self.subscribe_targets_done()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('mission_planner')
    MissionPlanner = MissionPlanner()
    rospy.spin()
