#! /usr/bin/env python

import matplotlib
matplotlib.use('Qt5Agg')

import rospy
import matplotlib.pyplot as plt
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
import tf


class PositionPlotter:
    def __init__(self, world_link="/world",robot_link='/base_link', robot_name='jackal', position_links=['/odometry/filtered']):

        self.axes = plt.gca()
        self.axes.set_aspect('equal', adjustable='box')

        line = self.axes.plot([], [], label=robot_name)[0]

        self.data = {
        robot_name:(
            [],
            [],
            line
        )
        }


        self.axes.legend()

        self.tf_listener = tf.TransformListener()

        self.world_link = world_link
        self.robot_link = robot_link
        self.robot_name = robot_name

    def add_ranging(self, msg): 
        # type: (Ranging) -> None


        anchor_id = msg.anchorId
        tag_id = msg.tagId

        label = "tag{}_anchor{}".format(tag_id, anchor_id)

        if label not in self.data:

            line = self.axes.plot([], [], label=label)[0]

            self.data[label] = (
                [],
                [],
                line
            )

        time = rospy.Time.now().to_sec()


    def run(self):

        rate = rospy.Rate(1000)


        while not rospy.is_shutdown():
            print("H")
            # self.get_robot_position()



            # self.axes.legend()
            # print(self.data)
            self.axes.relim()      # make sure all the data fits
            self.axes.autoscale()  # auto-scale

            plt.draw()
            plt.pause(0.0000000001)
            # rate.sleep()

    def get_robot_position(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.world_link, self.robot_link, rospy.Time(0))

            x, y = trans[0], trans[1]

            self.data[self.robot_name][0].append(x)
            self.data[self.robot_name][1].append(y)

            self.data[self.robot_name][2].set_xdata(self.data[self.robot_name][0])
            self.data[self.robot_name][2].set_ydata(self.data[self.robot_name][1])

            print len(self.data[self.robot_name][1])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Error"
  

        
if __name__ == "__main__":
    rospy.init_node("location_drawer_node")

    plt.ion()
    plt.show()

    data_plotter = PositionPlotter()
    data_plotter.run()

    plt.show()