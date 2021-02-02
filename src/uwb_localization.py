#! /usr/bin/env python

import rospy
from fusion_ekf import FusionEKF
from visualization_msgs.msg import MarkerArray
from gtec_msgs.msg import Ranging
import numpy as np
from nav_msgs.msg import Odometry

class UWBLocalization:
    def __init__(self):
        self.kalman_filter = FusionEKF()

        self.anchor_poses = dict()

        anchors = '/gtec/toa/anchors'
        toa_ranging = '/gtec/toa/ranging'

        anchors_sub = rospy.Subscriber(anchors, MarkerArray, callback=self.add_anchors)
        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)

        publish = '/jackal/uwb/pose'

        self.estimated_pose = rospy.Publisher(publish, Odometry, queue_size=1)
        self.pose = Odometry()

    def add_anchors(self, msg):
        # type: (MarkerArray) -> None

        for marker in msg.markers:
            self.anchor_poses[marker.id] = np.array([marker.pose.position.x,marker.pose.position.y,marker.pose.position.z]) 

    def add_ranging(self, msg):
        # type: (Ranging) -> None

        if msg.tagId == 0:
            if msg.anchorId in self.anchor_poses:
                anchor_pose = self.anchor_poses[msg.anchorId]
                anchor_distance = msg.range / 1000.

                self.kalman_filter.process_measurement(anchor_pose, anchor_distance)

    def run(self):
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            self.pose.pose.pose.position.x = self.kalman_filter.kalman_filter.x[0]
            self.pose.pose.pose.position.y = self.kalman_filter.kalman_filter.x[1]
            self.pose.pose.pose.position.z = self.kalman_filter.kalman_filter.x[2]

            self.pose.twist.twist.linear.x = self.kalman_filter.kalman_filter.x[3]
            self.pose.twist.twist.linear.y = self.kalman_filter.kalman_filter.x[4]
            self.pose.twist.twist.linear.z = self.kalman_filter.kalman_filter.x[5]

            self.estimated_pose.publish(self.pose)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("uwb_localization_kalman")
    loc = UWBLocalization()
    loc.run()

    rospy.spin()


