#! /usr/bin/env python

import rospy
from ukf_uwb_localization import UKFUWBLocalization, get_tag_ids, get_time
from jackal_motion import JackalMotion
from gtec_msgs.msg import Ranging
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import json
import rospkg
import os
import numpy as np
from scipy.optimize import least_squares

class Jackal():
    localized_param_key = "is_localized"
    jackal_publish_path = 'uwb/odom'

    def get_tags(self, tags_file="tag_ids.json"):
        rospack = rospkg.RosPack()
        package_location = rospack.get_path('uwb_localization')
        
        tags_file = os.path.join(package_location, 'src' ,tags_file)

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
        p = [1.0001, 11.0, 14.0001, 20.9001, 1.0001, 0.0001, 0.0001, 3.9001, 4.9001, 1.0, 0, 0.0001, 0.0001, 0.0001, 2.0001, 0.0001, 0.0001]

        self.ns = rospy.get_namespace()

        if self.ns == '/':
            self.ns = "/Jackal1/"
        self.ranging_data = []
        self.right_tag, self.left_tag, self.anchor = get_tag_ids(self.ns)

        _, self.tag_to_robot, self.anchor_to_robot = self.get_tags() 

        print("Namespace:", self.ns)

        self.is_localized = False
        rospy.set_param(Jackal.localized_param_key, self.is_localized)

        self.loc = UKFUWBLocalization(p[0], p[1:7], accel_std=p[7], yaw_accel_std=p[8], alpha=p[9], beta=p[10], namespace=self.ns, right_tag=self.right_tag, left_tag=self.left_tag)

        self.d = np.linalg.norm(self.loc.tag_offset[self.right_tag] - self.loc.tag_offset[self.left_tag])

        print(self.d)

        rospy.set_param("left_id", self.left_tag)
        rospy.set_param("right_id", self.right_tag)
        rospy.set_param("anchor", self.anchor)

        anchors = '/gtec/toa/anchors'
        toa_ranging = '/gtec/toa/ranging'

        self.anchor_poses = dict()

        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)

        anchors_sub = rospy.Subscriber(anchors, MarkerArray, callback=self.add_anchors)

        self.motion = JackalMotion(self.ns)

    def add_anchors(self, msg):
        # type: (MarkerArray) -> None

        for marker in msg.markers:
            if marker.id not in self.anchor_to_robot: 
                self.anchor_poses[marker.id] = np.array([marker.pose.position.x,marker.pose.position.y, marker.pose.position.z]) 

    def add_ranging(self, msg):
        # type: (Ranging) -> None

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
                    "time": get_time(),
                    "anchorID": msg.anchorId,
                    "tagID": msg.tagId,
                    "range": msg.range / 1000,
                    'localized' : localized,
                    'pose' : pose
                }
            )

    def get_current_robot_pose(self, robot_name):
        path_name = robot_name + Jackal.jackal_publish_path

        msg = rospy.wait_for_message(path_name, Odometry)

        return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])


    def explore_recorded_data(self):
        data = {
            "localized": {
                "data": [],
                'robots' : []
            },
            "unlocalized": {
                'data': [],
                "robots" :[]
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

        self.motion.step()
        rospy.set_param(Jackal.localized_param_key, self.is_localized)

    def trilaterate_position(self, range_data):
        res = least_squares(self.trilateration_function, [0,0,0,0], args=(range_data, ))

        left = res.x[0:2]
        right = res.x[2:4]
        
        center = (left + right) / 2
        v_ab = left - right
        theta = np.arccos(v_ab[1] / np.linalg.norm(v_ab))

        print(center, v_ab, theta, np.degrees(theta))

        return np.array([center[0], center[1], 0, 0, theta, 0 ])

    def trilateration_function(self, x, distances):
        # x[0] = left_x
        # x[1] = left_y
        # x[2] = right_x
        # x[3] = right_y

        x1, y1, x2, y2 = x

        residuals = [
            (x1 - x2) ** 2 + (y1 - y2) ** 2 - self.d ** 2,
        ]

        for distance in distances:
            anchor = distance['pose']
            tagID = distance['tagID']
            distance = distance['range']

            z = self.loc.tag_offset[tagID][2]

            if tagID == self.left_tag:
                x = x1
                y = y1
            else:
                x = x2
                y = y2

            residuals.append((x - anchor[0]) ** 2 + (y - anchor[1]) ** 2 + (z - anchor[2]) ** 2 - distance ** 2)

        return residuals

if __name__ == "__main__":
    rospy.init_node("full_jackal", anonymous=True)

    jackal = Jackal()

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        jackal.step()

        rate.sleep()


