#! /usr/bin/env python

import matplotlib
matplotlib.use('Qt5Agg')

import rospy
import matplotlib.pyplot as plt
from gtec_msgs.msg import Ranging
from live_plotter import LivePlotter

class RangePlotter:
    def __init__(self, tag_ids=("0", "1"), n=-1):
        toa_ranging = '/gtec/toa/ranging'

        self.live_plotter = LivePlotter()

        self.n = n

        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging)


    def add_ranging(self, msg): 
        # type: (Ranging) -> None


        anchor_id = msg.anchorId
        tag_id = msg.tagId

        label = "tag{}_anchor{}".format(tag_id, anchor_id)

        time = rospy.Time.now().to_sec()

        self.live_plotter.add_data_point(label, time, msg.range)

    def run(self):
        self.live_plotter.show()
        
if __name__ == "__main__":
    rospy.init_node("data_drawer_node")

    data_plotter = RangePlotter()
    data_plotter.run()