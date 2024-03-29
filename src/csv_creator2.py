#! /usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Point
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

from ukf.datapoint import DataType


class Recorder(object):
    def __init__(self, out="/home/marius/catkin_ws/src/UWB-Localization/out3.csv"):
        self.data = []
        self.out = out

        self.anchor_poses = dict()

        self.tag_offset = {
            1: [0, 0.162, 0.184],
            0: [0, -0.162, 0.184],

            3: [0, 0.162, 0.184],
            2: [0, -0.162, 0.184]
        }

        anchors = '/gtec/toa/anchors'
        toa_ranging = '/gtec/toa/ranging'

        self.anchors_sub = rospy.Subscriber(anchors, MarkerArray, callback=self.add_anchors)
        self.ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)

        id = 2

        # odometry = '/Jackal{}/odometry/filtered'.format(id)
        # self.odometry_filtered = rospy.Subscriber(odometry, Odometry,
        #                                           callback=self.create_odometry_callback(DataType.ODOMETRY))

        # odometry = '/Jackal{}/odometry/transformed'.format(id)
        # self.odometry_filtered = rospy.Subscriber(odometry, Odometry,
        #                                           callback=self.create_odometry_callback(DataType.ODOMETRY))

        odometry = '/Jackal{}/uwb/odom'.format(id)
        self.odometry_uwb = rospy.Subscriber(odometry, Odometry,
                                             callback=self.create_odometry_callback(DataType.ODOMETRY))

        odometry = '/Jackal{}/ground_truth/state'.format(id)
        self.odometry_gt = rospy.Subscriber(odometry, Odometry,
                                            callback=self.create_odometry_callback(DataType.GROUND_TRUTH))

        imu = '/Jackal{}/imu/data'.format(id)
        self.imu = rospy.Subscriber(imu, Imu, callback=self.add_imu)

        print("Ready")

    def add_imu(self, msg):
        # type: (Imu) -> None
        t = self.get_time()

        orien = msg.orientation
        ang_vel = msg.angular_velocity
        lin_acc = msg.linear_acceleration

        self.data.append((DataType.IMU, t, orien.x, orien.y, orien.z, orien.w, ang_vel.x, ang_vel.y, ang_vel.z,
                          lin_acc.x, lin_acc.y, lin_acc.z))

    def create_odometry_callback(self, id):

        def add_odometry(msg):
            t = self.get_time()

            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z

            v = msg.twist.twist.linear.x
            theta = euler_from_quaternion((
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ))[2]

            theta_yaw = msg.twist.twist.angular.z

            self.data.append((id, t, px, py, pz, v, theta, theta_yaw, msg.pose.pose.orientation.x,
                              msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z,
                              msg.pose.pose.orientation.w))

        return add_odometry

    def add_anchors(self, msg):
        # type: (MarkerArray) -> None

        for marker in msg.markers:
            self.anchor_poses[marker.id] = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]

    def get_time(self):
        return rospy.Time.now().to_nsec()

    def add_ranging(self, msg):
        # type: (Ranging) -> None
        t = self.get_time()

        if msg.anchorId in self.anchor_poses:
            anchor_pose = self.anchor_poses[msg.anchorId]
            anchor_distance = msg.range / 1000.

            tag = self.tag_offset[msg.tagId]

            self.data.append((DataType.UWB, t, anchor_distance,
                              anchor_pose[0], anchor_pose[1], anchor_pose[2],
                              tag[0], tag[1], tag[2]))

    def save(self):
        print("Saving", len(self.data), "datapoints")

        with open(self.out, "w") as file:
            file.writelines(",".join(map(str, d)) + '\n' for d in self.data)

        print "Finished saving"


if __name__ == "__main__":
    rospy.init_node("csv_recorder2", anonymous=True)

    rec = Recorder()

    rospy.spin()

    rec.save()
