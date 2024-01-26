#! /usr/bin/env python
# coding=utf-8
from __future__ import print_function

import json
import os

import numpy as np
import rospkg
import rospy
import tf
from geometry_msgs.msg import Point
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
from scipy.optimize import least_squares
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray

from ukf.datapoint import DataType, DataPoint
from ukf.fusion_ukf import FusionUKF


def get_time():
    return rospy.Time.now().to_nsec()


def get_anchors(tags_file="tag_ids.json"):
    rospack = rospkg.RosPack()
    package_location = rospack.get_path('uwb_localization')

    tags_file = os.path.join(package_location, 'src', tags_file)

    with open(tags_file, 'r') as f:
        tag_data = json.load(f)

    anchor_to_robot = dict()

    for key, values in tag_data.items():
        if values['anchor'] not in anchor_to_robot or anchor_to_robot[values['anchor']] == '/':
            anchor_to_robot[values['anchor']] = key

    return anchor_to_robot


class UKFUWBLocalization(object):
    def __init__(self, uwb_std=1, odometry_std=(1, 1, 1, 1, 1, 1), accel_std=1, yaw_accel_std=1, alpha=1, beta=0,
                 namespace=None, right_tag=0, left_tag=1, x_initial=0, y_initial=0, theta_initial=0, k=None):
        if namespace is None:
            namespace = '/'

        sensor_std = {
            DataType.UWB: {
                'std': [uwb_std],
                'nz': 1
            },
            DataType.ODOMETRY: {
                'std': odometry_std,
                'nz': 6
            }
        }

        self.namespace = namespace
        self.right_tag = right_tag
        self.left_tag = left_tag
        self.anchor_to_robot = get_anchors()

        self.ukf = FusionUKF(sensor_std, accel_std, yaw_accel_std, alpha, beta, k=k)

        self.anchor_poses = dict()
        self.tag_offset = self.retrieve_tag_offsets(
            {namespace + "left_tag": left_tag, namespace + "right_tag": right_tag}, namespace=namespace,
            right_tag=right_tag, left_tag=left_tag)

        # right: 0
        # left: 1
        # self.tag_offset = {
        #     0:np.array([0, -0.162, 0.184]),
        #     1:np.array([0, 0.162, 0.184])
        # }
        # print(self.tag_offset)

        anchors = '/gtec/toa/anchors'
        toa_ranging = '/gtec/toa/ranging'

        transformed_odom = namespace + 'odometry/transformed'

        if namespace is None:
            publish_odom = '/jackal/uwb/odom'
            odometry = '/odometry/filtered'
        else:
            publish_odom = namespace + 'uwb/odom'
            odometry = namespace + 'odometry/filtered'

        self.anchors_sub = rospy.Subscriber(anchors, MarkerArray, callback=self.add_anchors)
        self.ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)
        self.odometry = rospy.Subscriber(odometry, Odometry, callback=self.add_odometry)

        self.estimated_pose = rospy.Publisher(publish_odom, Odometry, queue_size=1)
        self.transformed_odom = rospy.Publisher(transformed_odom, Odometry, queue_size=1)
        self.odom = Odometry()

        self.sensor_data = []

        self.initialized = False
        self.start_translation = np.array([x_initial, y_initial])
        self.start_rotation = theta_initial

        self.cache_data = []

    def retrieve_tag_offsets(self, tags, base_link='base_link', namespace=None, right_tag=0, left_tag=1):
        transforms = dict()

        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)

        # right: 0
        # left: 1
        default = {
            right_tag: np.array([0, -0.162, 0.184]),
            left_tag: np.array([0, 0.162, 0.184])
        }

        if namespace is not None:
            base_link = namespace + base_link

        for tag in tags:
            timeout = 5

            while not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform(base_link, tag, rospy.Time(0))
                    transforms[tags[tag]] = np.array([trans[0], trans[1], trans[2]])
                    break

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    if timeout <= 0:
                        transforms[tags[tag]] = default[tags[tag]]
                        break
                    timeout -= 1

                rate.sleep()

        return transforms

    def add_odometry(self, msg):
        t = get_time()

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

        theta_yaw = theta_yaw
        px += self.start_translation[0]
        py += self.start_translation[1]

        theta = (theta + self.start_rotation) % (np.pi * 2)

        # print(px, py, self.start_translation, self.start_rotation)

        # px = np.cos(self.start_rotation) * px - np.sin(self.start_rotation) * py + self.start_translation[0]
        # py = np.cos(self.start_rotation) * px + np.sin(self.start_rotation) * py + self.start_translation[1]

        data = DataPoint(DataType.ODOMETRY, np.array([px, py, pz, v, theta, theta_yaw]), t)

        # self.ukf.update(data)

        self.sensor_data.append(data)

        tranformed_pose = Odometry()

        tranformed_pose.header = msg.header
        tranformed_pose.pose.pose.position.x = px
        tranformed_pose.pose.pose.position.y = py
        tranformed_pose.pose.pose.position.z = pz
        tranformed_pose.twist.twist.linear.x = v
        tranformed_pose.twist.twist.angular.z = theta_yaw
        angles = quaternion_from_euler(0, 0, theta)

        tranformed_pose.pose.pose.orientation.x = angles[0]
        tranformed_pose.pose.pose.orientation.y = angles[1]
        tranformed_pose.pose.pose.orientation.z = angles[2]
        tranformed_pose.pose.pose.orientation.w = angles[3]

        self.transformed_odom.publish(tranformed_pose)

    def add_anchors(self, msg):
        # type: (MarkerArray) -> None

        for marker in msg.markers:
            self.anchor_poses[marker.id] = np.array(
                [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])

    def is_localized(self, robot_name):
        parameter_name = robot_name + "is_localized"

        if rospy.has_param(parameter_name):
            return rospy.get_param(parameter_name)
        else:
            return True

    def add_ranging(self, msg):
        # type: (Ranging) -> None
        t = get_time()

        if not self.is_localized(self.namespace):
            return

        if msg.anchorId in self.anchor_poses:
            if msg.anchorId in self.anchor_to_robot:
                if not self.is_localized(self.anchor_to_robot[msg.anchorId]):
                    return

            if msg.tagId in self.tag_offset:
                anchor_pose = self.anchor_poses[msg.anchorId]
                anchor_distance = msg.range / 1000.

                # print("ADDED UWB")

                data = DataPoint(DataType.UWB, anchor_distance, t, extra={
                    "anchor": anchor_pose,
                    'sensor_offset': self.tag_offset[msg.tagId]
                    # 'sensor_offset': None
                })

                # self.ukf.update(data)

                self.sensor_data.append(data)

    def intialize(self, x, P):
        t = get_time()

        self.ukf.initialize(x, P, t)
        self.initialized = True

    def process_ukf_data(self):
        sorted_data = sorted(self.sensor_data, key=lambda x: x.timestamp)

        for data in sorted_data:
            self.ukf.update(data)

        self.clear_data()

        x, y, z, v, yaw, yaw_rate = self.ukf.x

        self.odom.pose.pose.position.x = x
        self.odom.pose.pose.position.y = y
        self.odom.pose.pose.position.z = z
        self.odom.twist.twist.linear.x = -v
        self.odom.twist.twist.angular.z = yaw_rate

        angles = quaternion_from_euler(0, 0, yaw)

        self.odom.pose.pose.orientation.x = angles[0]
        self.odom.pose.pose.orientation.y = angles[1]
        self.odom.pose.pose.orientation.z = angles[2]
        self.odom.pose.pose.orientation.w = angles[3]

        self.estimated_pose.publish(self.odom)

    def clear_data(self):
        del self.sensor_data[:]

    def set_initial(self, x_initial, y_initial, theta_initial):
        print("Setting initial", x_initial, y_initial, theta_initial)
        self.start_translation = np.array([x_initial, y_initial])
        self.start_rotation = theta_initial

    def step(self, initial_P=None):
        if not self.initialized:
            d = np.linalg.norm(self.tag_offset[self.right_tag] - self.tag_offset[self.left_tag])

            self.process_initial_data(self.cache_data, d, initial_P)
        else:
            self.process_ukf_data()

    def run(self, initial_P=None):
        if not self.initialized:
            self.initialize_pose(initial_P)

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.process_ukf_data()

            rate.sleep()

    def func(self, x, d, distances):
        # x[0] = left_x
        # x[1] = left_y
        # x[2] = right_x
        # x[3] = right_y

        x1, y1, x2, y2 = x

        residuals = [
            (x1 - x2) ** 2 + (y1 - y2) ** 2 - d ** 2,
        ]

        for distance in distances:
            anchor = distance['anchor']
            tag = distance['tag']
            distance = distance['dist']

            z = tag[2]

            if np.all(tag == self.tag_offset[self.left_tag]):
                x = x1
                y = y1
            else:
                x = x2
                y = y2

            residuals.append((x - anchor[0]) ** 2 + (y - anchor[1]) ** 2 + (z - anchor[2]) ** 2 - distance ** 2)

        return residuals

    def process_initial_data(self, uwb, d, initial_P=None):
        for s in self.sensor_data:
            if s.data_type == DataType.UWB:
                uwb.append({
                    'anchor': s.extra['anchor'],
                    'tag': s.extra['sensor_offset'],
                    'dist': s.measurement_data
                })

        print("UWB PROCESS LENGTH", len(uwb))
        if len(uwb) >= 3:
            # res = least_squares(self.func, [0, 0, 0, 0], args=(d, uwb))
            res = least_squares(self.func, [0, 0, 0, 0], args=(d, uwb), loss='soft_l1',
                                f_scale=10.0,
                                jac='3-point', )

            # print(res)

            left = res.x[0:2]
            right = res.x[2:4]

            center = (left + right) / 2
            v_ab = left - right
            theta = np.arccos(v_ab[1] / np.linalg.norm(v_ab))

            print(center, v_ab, theta, np.degrees(theta))

            del self.sensor_data[:]

            self.set_initial(center[0], center[1], theta)

            # self.start_translation = center
            # self.start_rotation = theta

            if initial_P is None:
                initial_P = np.identity(6)

            self.intialize(np.array([center[0], center[1], 0, 0, theta, 0]), initial_P)

    def initialize_pose(self, initial_P=None):

        delay = 1  # s
        rate = rospy.Rate(delay)

        d = np.linalg.norm(self.tag_offset[self.right_tag] - self.tag_offset[self.left_tag])

        uwb = []

        i = 0

        while not rospy.is_shutdown() and not self.initialized:
            if i > 4:
                x = np.zeros(6)

                for i in range(len(self.sensor_data)):
                    data = self.sensor_data[-(i + 1)]

                    if data.data_type == DataType.ODOMETRY:
                        x = data.measurement_data
                        break

                self.intialize(x, initial_P)

                break

            self.process_initial_data(uwb, d, initial_P)

            i += 1
            rate.sleep()


def get_tag_ids(ns, tags_file='tag_ids.json'):
    rospack = rospkg.RosPack()
    package_location = rospack.get_path('uwb_localization')

    tags_file = os.path.join(package_location, 'src', tags_file)

    with open(tags_file, 'r') as f:
        tag_data = json.load(f)

    print(tag_data)
    right_tag = tag_data[ns]['right_tag']
    left_tag = tag_data[ns]['left_tag']
    anchor = tag_data[ns]['anchor']
    print(right_tag, left_tag, anchor)

    return right_tag, left_tag, anchor


def to_p(ns):
    x = [6.0, 2.533101067702734, 2.415959286874128, 0.2661715221275878, 0.5665457615654477, -5.550914177849361,
         0.6256636346811915, 3.3333308521420166, 3.3179852572472988, 0.0001, 0.4345337428241794, 0.7550338410102531,
         0.01, 0.01, 0.036182385874357806, 0.08532954911231647, 4.172498592187061, 5.0, 2.7881495217747605]

    uwb_std = x[0]
    speed_noise_std = x[1]
    yaw_rate_noise_std = x[2]
    alpha = x[3]
    beta = x[4]
    k = x[5]
    P = np.diag([x[6], x[7], x[8], x[9], x[10], x[11]])
    odometry_std = x[12:18]

    ukf = UKFUWBLocalization(uwb_std, odometry_std, accel_std=speed_noise_std, yaw_accel_std=yaw_rate_noise_std,
                             alpha=alpha,
                             beta=beta, namespace=ns,
                             right_tag=right_tag, left_tag=left_tag, k=k)

    return ukf, P


if __name__ == "__main__":
    rospy.init_node("ukf_uwb_localization_kalman")

    ns = rospy.get_namespace()

    print("Namespace:", ns)

    right_tag, left_tag, _ = get_tag_ids(ns)

    intial_pose = rospy.wait_for_message(ns + 'ground_truth/state', Odometry)
    x, y, z = intial_pose.pose.pose.position.x, intial_pose.pose.pose.position.y, intial_pose.pose.pose.position.z
    v = 0.2
    theta = euler_from_quaternion((
        intial_pose.pose.pose.orientation.x,
        intial_pose.pose.pose.orientation.y,
        intial_pose.pose.pose.orientation.z,
        intial_pose.pose.pose.orientation.w
    ))[2]

    print("Actual Initial", x, y, v, theta, np.degrees(theta))
    #
    # p = [1.0001, 11.0, 14.0001, 20.9001, 1.0001, 1, 1, 3.9001 * 10, 4.9001 * 10, 1.0, 0, 0.0001, 0.0001, 0.0001, 2.0001,
    #      0.0001, 0.0001]
    #
    # p = np.array(p)
    # # p[0:7] *= 0.1
    #
    # # p = [1.0001, 1000, 1000, 1, 1.0001, 0.0001, 0.0001, 3.9001, 4.9001, 1.0, 0, 0.0001, 0.0001, 0.0001, 2.0001,
    # #      0.0001, 0.0001]
    #
    # loc = UKFUWBLocalization(p[0], p[1:7], accel_std=p[7], yaw_accel_std=p[8], alpha=p[9], beta=p[10], namespace=ns,
    #                          right_tag=right_tag, left_tag=left_tag)

    loc, P = to_p(ns)
    loc.set_initial(x, y, theta)
    loc.intialize(np.array([x, y, z, v, theta]),
                  P)

    loc.run()

    rospy.spin()
