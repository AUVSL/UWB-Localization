#! /usr/bin/env python
# coding=utf-8

import numpy as np
import rospy
import tf
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

from efk.fusion_ekf import FusionEKF


class UWBLocalization(object):
    def __init__(self):
        self.kalman_filter_tag_0 = FusionEKF()
        self.kalman_filter_tag_1 = FusionEKF()

        self.anchor_poses = dict()

        anchors = '/gtec/toa/anchors'
        toa_ranging = '/gtec/toa/ranging'

        anchors_sub = rospy.Subscriber(anchors, MarkerArray, callback=self.add_anchors)
        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)

        publish_tag_0 = '/jackal/uwb/pose/0'
        publish_tag_1 = '/jackal/uwb/pose/1'

        self.estimated_pose_tag_0 = rospy.Publisher(publish_tag_0, Odometry, queue_size=1)
        self.estimated_pose_tag_1 = rospy.Publisher(publish_tag_1, Odometry, queue_size=1)

        self.pose = Odometry()

        self.tf_listener = tf.TransformListener()

    def change_pose_ref(self, odom, tag, world='/world', base='/base_link'):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(world, tag, rospy.Time(0))

            pose = odom.pose.pose.position

            pose.x -= trans[0]
            pose.y -= trans[1]
            pose.z -= trans[2]

            (trans, rot) = self.tf_listener.lookupTransform(world, base, rospy.Time(0))

            pose.x += trans[0]
            pose.y += trans[1]
            pose.z += trans[2]

            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    def add_anchors(self, msg):
        # type: (MarkerArray) -> None

        for marker in msg.markers:
            self.anchor_poses[marker.id] = np.array(
                [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])

    def add_ranging(self, msg):
        # type: (Ranging) -> None

        if msg.anchorId in self.anchor_poses:
            anchor_pose = self.anchor_poses[msg.anchorId]
            anchor_distance = msg.range / 1000.

            if msg.tagId == 0:
                self.kalman_filter_tag_0.process_measurement(anchor_pose, anchor_distance)
            elif msg.tagId == 1:
                self.kalman_filter_tag_1.process_measurement(anchor_pose, anchor_distance)

    def run(self):
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            self.pose.pose.pose.position.x = self.kalman_filter_tag_0.kalman_filter.x[0]
            self.pose.pose.pose.position.y = self.kalman_filter_tag_0.kalman_filter.x[1]
            self.pose.pose.pose.position.z = self.kalman_filter_tag_0.kalman_filter.x[2]

            self.pose.twist.twist.linear.x = self.kalman_filter_tag_0.kalman_filter.x[3]
            self.pose.twist.twist.linear.y = self.kalman_filter_tag_0.kalman_filter.x[4]
            self.pose.twist.twist.linear.z = self.kalman_filter_tag_0.kalman_filter.x[5]

            # r = self.change_pose_ref(self.pose, '/right_tag')

            # if r:
            self.estimated_pose_tag_0.publish(self.pose)

            self.pose.pose.pose.position.x = self.kalman_filter_tag_1.kalman_filter.x[0]
            self.pose.pose.pose.position.y = self.kalman_filter_tag_1.kalman_filter.x[1]
            self.pose.pose.pose.position.z = self.kalman_filter_tag_1.kalman_filter.x[2]

            self.pose.twist.twist.linear.x = self.kalman_filter_tag_1.kalman_filter.x[3]
            self.pose.twist.twist.linear.y = self.kalman_filter_tag_1.kalman_filter.x[4]
            self.pose.twist.twist.linear.z = self.kalman_filter_tag_1.kalman_filter.x[5]

            # r = self.change_pose_ref(self.pose, '/left_tag')

            # if r:
            self.estimated_pose_tag_1.publish(self.pose)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("uwb_localization_kalman")
    loc = UWBLocalization()
    loc.run()

    rospy.spin()
