#!/usr/bin/env python
from functools import partial
from queue import Empty, Queue
from threading import Thread

import cv2
import numpy as np
import rospy
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from roversim.msg import Pose


def callback(queue, rover, data):
    vision_radius = rospy.get_param_cached(rover + '/vision_radius')
    queue.put((int(data.x), int(data.y), int(vision_radius)))
    
    
def capture_topics(queue):
    subscribed_topics = set()

    while not rospy.is_shutdown():
        for topic, msg_type in rospy.get_published_topics():
            if msg_type == 'roversim/Pose' and topic not in subscribed_topics:
                rospy.loginfo(f"exploration_area - telemetry adding topic: {topic}")
                subscribed_topics.add(topic)
                rover = '/'.join(topic.split('/')[:-1])
                rospy.Subscriber(topic, Pose, partial(callback, queue, rover))
        
        rospy.sleep(1)


def accumulate(rover_movement_queue, area_request_queue, area_queue):

    def create_canvas():
        width = rospy.get_param('width')
        height = rospy.get_param('height')
        rospy.loginfo(f"exploration_area - accumulate Thread started with canvas shape: {width} {height}")
        return np.zeros((height, width), dtype=np.dtype('uint8'))
    
    canvas = create_canvas()
    message_counter = 0

    
    while not rospy.is_shutdown():
        try:
            x, y, vision_radius = rover_movement_queue.get(timeout=3)
            message_counter += 1
            cv2.circle(canvas, (x, y), radius=vision_radius, color=255, thickness=-1)
        except Empty:
            rospy.loginfo(f"exploration_area - accumulate rover_movement_queue timeout")
            plt.imshow(canvas)
        finally:
            try:
                area_request_queue.get(block=False)
                area_queue.put(np.sum(canvas)/255)
            except Empty:
                rospy.logdebug(f"exploration_area - accumulate area_request_queue Empty")


def draw_plot(area_request_queue, area_queue):
    rospy.loginfo(f"exploration_area - draw_plot Thread started")
    fig = plt.figure(figsize=(8, 6))
    x = [0]
    y = [0]
    ax = plt.subplot(111, title='Exploration area')

    def update(frames):
        area_request_queue.put(item=True)
        try:
            area = area_queue.get(timeout=3)
            x.append(x[-1] + 1)
            y.append(area)
        
            ax.plot(x, y, 'b-') 
        except Empty as e:
            rospy.loginfo(f"exploration_area - plot area_queue timeout")
        finally:
            return ax

    ani = FuncAnimation(fig, update, interval=1000)
    plt.show()
    

if __name__ == '__main__':
    rospy.init_node('exploration_area', anonymous=True)

    rover_movement_queue = Queue()
    area_request_queue = Queue(maxsize=1)
    area_queue = Queue()

    accumualte_thread = Thread(
        target=accumulate,
        args=(rover_movement_queue, area_request_queue, area_queue, )
    )
    accumualte_thread.start()

    draw_plot_thread = Thread(
        target=draw_plot,
        args=(area_request_queue, area_queue, )
    )
    draw_plot_thread.start()

    capture_topics(rover_movement_queue)
