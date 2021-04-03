#! /usr/bin/env python

import rospy
from ukf_uwb_localization import UKFUWBLocalization, get_tag_ids, get_time
from jackal_motion import JackalMotion
from gtec_msgs.msg import Ranging
import json
import rospkg
import os

class Jackal():
    localized_param_key = "is_localized"

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

        rospy.set_param("left_id", self.left_tag)
        rospy.set_param("right_id", self.right_tag)
        rospy.set_param("anchor", self.anchor)

        toa_ranging = '/gtec/toa/ranging'

        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)

        self.motion = JackalMotion(self.ns)

    def add_ranging(self, msg):
        # type: (Ranging) -> None

        if self.tag_to_robot[msg.tagId] == self.ns: 
            self.ranging_data.append(
                {
                    "time": get_time(),
                    "anchorID": msg.anchorId,
                    "tagID": msg.tagId,
                    "range": msg.range / 1000
                }
            )

    def get_current_robot_pose(self, robot_name):
        path_name = robot_name + Jackal.jackal_publish_path

        msg = rospy.wait_for_message(path_name, Odometry)

        return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])


    def explore_recorded_data(self):
        data = {
            "anchors": [],
            "localized": [],
            "unlocalized": []
        }

        for range_data in self.ranging_data:
            if range_data['anchorID'] in self.anchor_to_robot:
                robot_name_ns = self.anchor_to_robot[range_data['anchorID']]

                was_localized = self.check_if_localized(robot_name_ns)

                if was_localized:
                    data['localized'].append(range_data)
                else:
                    data['unlocalized'].append(range_data)
            else:
                data['anchors'].append(range_data)

        return data

    def check_if_localized(self, robot_name):
        parameter_name = robot_name + Jackal.localized_param_key
        
        return rospy.has_param(parameter_name) and rospy.get_param(parameter_name)

    def step(self):
        if self.is_localized:
            self.loc.step()
        else:
            recoreded_data = self.explore_recorded_data()

        
        self.motion.step()
        rospy.set_param(Jackal.localized_param_key, self.is_localized)

if __name__ == "__main__":
    rospy.init_node("full_jackal", anonymous=True)

    jackal = Jackal()

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        jackal.step()

        rate.sleep()


