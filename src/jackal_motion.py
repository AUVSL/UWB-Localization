#! /usr/bin/env python

from geometry_msgs.msg import Twist
import rospy

class JackalMotion:
    def __init__(self, namespace):
        self.v = 0
        self.w = 0

        self.cmd = Twist()

        self.vel_pub = rospy.Publisher(namespace + 'jackal_velocity_controller/cmd_vel', Twist, queue_size=1)

    def step(self):
        self.cmd.linear.x = self.v
        self.cmd.angular.z = self.w

        self.vel_pub.publish(self.cmd)

