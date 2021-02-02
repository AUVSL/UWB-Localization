#! /usr/bin/env python

import matplotlib
matplotlib.use('Qt5Agg')

import rospy
from nav_msgs.msg import Odometry
from live_plotter import LivePlotter
from geometry_msgs.msg import Point


class PositionPlotter:
    def __init__(self, world_link="/world",robot_link='/base_link', robot_name='jackal', position_links=['/odometry/filtered', '/ground_truth/state', '/jackal/uwb/pose/0', '/jackal/uwb/pose/1' ]):
        self.live_plotter = LivePlotter(alpha=0.5)
        self.live_plotter.ax.set_aspect("equal")

        self.world_link = world_link
        self.robot_link = robot_link
        self.robot_name = robot_name

        self.robot_position_topic = '/data_drawer/robot_pose'

        self.robot_position_sub = rospy.Subscriber(self.robot_position_topic, Point, self.add_robot_pose)

        self.subscribers = dict()

        for position_link in position_links:
            self.subscribers[position_link] = rospy.Subscriber(position_link, Odometry, self.create_position_subscriber_func(position_link))

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

    data_plotter = PositionPlotter()
    data_plotter.run()
    rospy.spin()