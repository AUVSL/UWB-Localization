#! /usr/bin/env python

import matplotlib
matplotlib.use('Qt5Agg')

import rospy
import matplotlib.pyplot as plt
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
import tf
from live_plotter import LivePlotter
from geometry_msgs.msg import Point


class PositionPlotter:
    def __init__(self, world_link="/world",robot_link='/base_link', robot_name='jackal', position_links=['/odometry/filtered']):
        self.live_plotter = LivePlotter()
        self.live_plotter.ax.set_aspect("equal")

        self.world_link = world_link
        self.robot_link = robot_link
        self.robot_name = robot_name

        self.robot_position_topic = '/data_drawer/robot_pose'

        self.robot_position_sub = rospy.Subscriber(self.robot_position_topic, Point, self.add_robot_pose)

    def add_robot_pose(self, msg):
        self.live_plotter.add_data_point(self.robot_name, msg.x, msg.y)

    def run(self):
        self.live_plotter.show()

        
if __name__ == "__main__":
    rospy.init_node("location_drawer_node")

    data_plotter = PositionPlotter()
    data_plotter.run()
    rospy.spin()