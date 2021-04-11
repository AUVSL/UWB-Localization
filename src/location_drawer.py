#! /usr/bin/env python
# coding=utf-8

import matplotlib

matplotlib.use('Qt5Agg')

import rospy
from nav_msgs.msg import Odometry
from live_plotter import LivePlotter
from geometry_msgs.msg import Point
import sys


class PositionPlotter(object):
    def __init__(self, robot_name='jackal', position_links=None):
        if position_links is None:
            position_links = []

            topic_names = rospy.get_published_topics()

            for (topic, _) in topic_names:
                if topic.endswith('uwb/odom') or topic.endswith('odometry/filtered') or topic.endswith('ground_truth/state'):
                    position_links.append(topic)

            print(position_links)


        self.live_plotter = LivePlotter(alpha=0.5)
        self.live_plotter.ax.set_aspect("equal")

        self.robot_name = robot_name

        self.robot_position_topic = '/data_drawer/robot_pose'

        self.robot_position_sub = rospy.Subscriber(self.robot_position_topic, Point, self.add_robot_pose)

        self.subscribers = dict()

        for position_link in position_links:
            self.subscribers[position_link] = rospy.Subscriber(position_link, Odometry,
                                                               self.create_position_subscriber_func(position_link))

    def create_position_subscriber_func(self, name):
        def add_pose(msg):
            # type: (Odometry) -> None

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            self.live_plotter.add_data_point(name, x, y)

        return add_pose

    def add_robot_pose(self, msg):
        self.live_plotter.add_data_point(self.robot_name, msg.x, msg.y)

    def run(self):
        self.live_plotter.show()


if __name__ == "__main__":
    rospy.init_node("location_drawer_node")

    myargv = rospy.myargv(argv=sys.argv)[1:]

    if len(myargv) == 0:
        myargv = None

    data_plotter = PositionPlotter(position_links=myargv)
    data_plotter.run()
    rospy.spin()
