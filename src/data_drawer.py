#! /usr/bin/env python

import matplotlib
matplotlib.use('Qt5Agg')

import rospy
import matplotlib.pyplot as plt
from gtec_msgs.msg import Ranging


class DataPlotter:
    def __init__(self, tag_ids=("0", "1"), n=-1):
        self.data = dict()

        toa_ranging = '/gtec/toa/ranging'

        self.axes = plt.gca()
        self.axes.legend()
        self.colors = ['r', 'b', 'g']

        self.n = n

        ranging_sub = rospy.Subscriber(toa_ranging, Ranging, callback=self.add_ranging, queue_size=1)


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

        self.data[label][0].append(msg.range)
        self.data[label][1].append(time)

        self.data[label][2].set_xdata(self.data[label][1])
        self.data[label][2].set_ydata(self.data[label][0])

    def run(self):

        rate = rospy.Rate(50)


        while not rospy.is_shutdown():
            self.axes.legend()
            # print(self.data)
            self.axes.relim()      # make sure all the data fits
            self.axes.autoscale()  # auto-scale

            if self.n > 0 :
                x_max = self.axes.get_xlim()[-1]
                x_min = max(x_max - self.n, self.axes.get_xlim()[0])

                self.axes.set_xlim([x_min, x_max])

            plt.draw()
            plt.pause(0.0001)
            rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("data_drawer_node")

    every = matplotlib.rcsetup.all_backends

    for name in every:
        out = matplotlib.rcsetup.validate_backend(name)
        print out, name

    plt.ion()
    plt.show()

    data_plotter = DataPlotter()
    data_plotter.run()

    plt.show()