#! /usr/bin/env python
# coding=utf-8

import json
import os

import numpy as np
import rospkg
import rospy
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
from scipy.optimize import least_squares
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

from jackal_motion import JackalMotion
from ukf_uwb_localization import UKFUWBLocalization, get_tag_ids, get_time


class Jackal(object):
    localized_param_key = "is_localized"
    jackal_publish_path = 'uwb/odom'

    def get_tags(self, tags_file="tag_ids.json"):
        rospack = rospkg.RosPack()
        package_location = rospack.get_path('uwb_localization')

        tags_file = os.path.join(package_location, 'src', tags_file)

        with open(tags_file, 'r') as f:
            tag_data = json.load(f)

        tag_to_robot = dict()
        anchor_to_robot = dict()

        for key, values in tag_data.items():

            if values['right_tag'] not in tag_to_robot or tag_to_robot[values['right_tag']] == '/':
                tag_to_robot[values['right_tag']] = key

            if values['left_tag'] not in tag_to_robot or tag_to_robot[values['left_tag']] == '/':
                tag_to_robot[values['left_tag']] = key

            if values['anchor'] not in tag_to_robot or tag_to_robot[values['anchor']] == '/':
                anchor_to_robot[values['anchor']] = key

        return tag_data, tag_to_robot, anchor_to_robot

    def __init__(self):
        p = [1.0001, 11.0, 14.0001, 20.9001, 1.0001, 0.0001, 0.0001, 3.9001, 4.9001, 1.0, 0, 0.0001, 0.0001, 0.0001,
             2.0001, 0.0001, 0.0001]

        self.ns = rospy.get_namespace()

        if self.ns == '/':
            self.ns = "/Jackal1/"
        self.ranging_data = []
        self.odometry_data = None
        self.odom_times = None
        self.right_tag, self.left_tag, self.anchor = get_tag_ids(self.ns)

        _, self.tag_to_robot, self.anchor_to_robot = self.get_tags()

        print("Namespace:", self.ns)

        self.is_localized = False
        rospy.set_param(Jackal.localized_param_key, self.is_localized)

        self.loc = UKFUWBLocalization(p[0], p[1:7], accel_std=p[7], yaw_accel_std=p[8], alpha=p[9], beta=p[10],
                                      namespace=self.ns, right_tag=self.right_tag, left_tag=self.left_tag)

        self.d = np.linalg.norm(self.loc.tag_offset[self.right_tag] - self.loc.tag_offset[self.left_tag])

        print(self.d)

        rospy.set_param("left_id", self.left_tag)
        rospy.set_param("right_id", self.right_tag)
        rospy.set_param("anchor", self.anchor)

        anchors = '/gtec/toa/anchors'
        toa_ranging = '/gtec/toa/ranging'

        if self.ns is not None:
            odometry = self.ns + 'odometry/filtered'
        else:
            odometry = '/odometry/filtered'

        self.anchor_poses = dict()

        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)

        anchors_sub = rospy.Subscriber(anchors, MarkerArray, callback=self.add_anchors)

        odometry_sub = rospy.Subscriber(odometry, Odometry, callback=self.add_odometry)

        self.motion = JackalMotion(self.ns)

    def add_anchors(self, msg):
        # type: (MarkerArray) -> None

        for marker in msg.markers:
            if marker.id not in self.anchor_to_robot:
                self.anchor_poses[marker.id] = np.array(
                    [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])

    def add_ranging(self, msg):
        # type: (Ranging) -> None
        t = get_time()

        if self.tag_to_robot[msg.tagId] == self.ns:
            is_mobile = msg.anchorId in self.anchor_to_robot

            if is_mobile:
                robot = self.anchor_to_robot[msg.anchorId]

                localized = self.check_if_localized(robot)

                if localized:
                    pose = self.get_current_robot_pose(robot)
                else:
                    pose = None
            else:
                localized = True

                if msg.anchorId not in self.anchor_poses:
                    return

                pose = self.anchor_poses[msg.anchorId]

            self.ranging_data.append(
                {
                    "time": t,
                    "anchorID": msg.anchorId,
                    "tagID": msg.tagId,
                    "range": msg.range / 1000.,
                    'localized': localized,
                    'pose': pose
                }
            )

    def add_odometry(self, msg):
        # type: (Odometry) -> None

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

        # print(t, px, py, pz, v, theta, theta_yaw)
        # print(self.odometry_data.dtype)
        # print(len(self.odom_times), self.odom_times.dtype)

        if self.odometry_data is None:
            self.odometry_data = np.array((px, py, pz, v, theta, theta_yaw))
        else:
            self.odometry_data = np.vstack((self.odometry_data, (px, py, pz, v, theta, theta_yaw)))
        if self.odom_times is None:
            self.odom_times = np.array([t], dtype=np.uint64)
        else:
            self.odom_times = np.append(self.odom_times, t)

    def get_current_robot_pose(self, robot_name):
        path_name = robot_name + Jackal.jackal_publish_path

        msg = rospy.wait_for_message(path_name, Odometry)

        return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def explore_recorded_data(self):
        data = {
            "localized": {
                "data": [],
                'robots': []
            },
            "unlocalized": {
                'data': [],
                "robots": []
            }
        }

        for range_data in self.ranging_data:
            if range_data['localized']:
                container = data['localized']
            else:
                container = data['unlocalized']

            container['data'].append(range_data)
            container['robots'].append(range_data['anchorID'])

        return data

    def check_if_localized(self, robot_name):
        parameter_name = robot_name + Jackal.localized_param_key

        return rospy.has_param(parameter_name) and rospy.get_param(parameter_name)

    def spread_message(self, ns):
        pass

    def step(self):
        if self.is_localized:
            self.loc.step()
        else:
            recoreded_data = self.explore_recorded_data()

            total_knowns = len(set(recoreded_data['localized']['robots']))

            if total_knowns >= 3:
                total_data_points = len(recoreded_data['localized']['data'])

                print(total_data_points)
                if total_data_points > 50:
                    pose = self.trilaterate_position(recoreded_data['localized']['data'])
                    self.is_localized = True
                    self.loc.intialize(pose, np.identity(6))

                    print(pose)
            else:
                # Go thorugh all the neighbors and ask if they can localize
                #
                pass

        self.motion.step()
        rospy.set_param(Jackal.localized_param_key, self.is_localized)

    def find_closest_odometry(self, range_data):
        closest = np.zeros((len(range_data), 4))

        for i, datapoint in enumerate(range_data):
            t = datapoint['time']

            start, end = self.find_closest_sorted(t)
            interpol = self.odom_interpolation(start, end, t)

            # print start, end, t, self.odom_times[start], self.odom_times[end], interpol[[1, 2, 3, 5]]

            closest[i] = interpol[[0, 1, 2, 4]]
            # print start, end, interpol
        # print closest

        return closest

    def odom_interpolation(self, start, end, t):

        odom_initial = self.odometry_data[start]
        odom_final = self.odometry_data[end]

        dt = (self.odom_times[end] - self.odom_times[start])

        if dt != 0:
            percent = (t - self.odom_times[start]) / dt
        else:
            return odom_initial

        # print start, end, self.odom_times[end],  self.odom_times[start],percent

        blend = (1 - percent) * odom_initial + percent * odom_final

        angle_start = odom_initial[4] % (2 * np.pi)
        angle_end = odom_final[4] % (2 * np.pi)

        rotation = ((angle_start - angle_end) + np.pi) % (2 * np.pi) - np.pi
        blend[4] = (odom_initial[4] + rotation * percent) % (2 * np.pi)

        return blend

    def find_closest_sorted(self, t):
        # print times.shape

        idx = np.searchsorted(self.odom_times, t, side='left')

        if idx != 0 and (idx >= len(self.odom_times) - 1 or self.odom_times[idx] > t):
            if idx == len(self.odom_times):
                idx -= 1

            return idx - 1, idx
        return idx, idx + 1

    def trilaterate_position(self, range_data):
        if len(self.odometry_data) > len(range_data):
            odometry = self.find_closest_odometry(range_data)
        else:
            odometry = np.zeros((4, len(range_data)))

        res = least_squares(self.trilateration_function, [0, 0, 0], args=(range_data, odometry))

        res = res.x

        return np.array([res[0], res[1], 0, 0, res[2], 0])

    def trilateration_function(self, input_x, distances, odometry_data):
        # x[0] = x_start
        # x[1] = y_start
        # x[2] = theta

        _, _, theta = input_x

        xy = input_x[:2]

        residuals = [
            # (x1 - x2) ** 2 + (y1 - y2) ** 2 - self.d ** 2,
        ]

        for i, distance in enumerate(distances):
            anchor = distance['pose']
            tagID = distance['tagID']
            distance = distance['range']

            odometry = odometry_data[i]
            # 0 = x, 1 = y, 2 = z, 3 = theta
            xy_odom = odometry[:2]
            theta_odom = odometry[3]

            z = self.loc.tag_offset[tagID][2] + odometry[2]

            xy_tag = self.loc.tag_offset[tagID][:2]

            xy_world = self.rotate(xy_odom, theta) + xy
            xy_tag = self.rotate(xy_tag, theta + theta_odom) + xy_world

            x, y = xy_tag

            residuals.append((x - anchor[0]) ** 2 + (y - anchor[1]) ** 2 + (z - anchor[2]) ** 2 - distance ** 2)

        return residuals

    def rotate(self, xy, rot_angle):
        c = np.cos(rot_angle)
        s = np.sin(rot_angle)

        result = np.matmul(np.array([[c, -s],
                                     [s, c]]), xy)

        return result


if __name__ == "__main__":
    rospy.init_node("full_jackal", anonymous=True)

    jackal = Jackal()

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        jackal.step()

        rate.sleep()

    print("End")
