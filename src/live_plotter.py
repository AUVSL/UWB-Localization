# coding=utf-8
from __future__ import print_function

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class LivePlotter(object):
    def __init__(self, update_interval=1000, alpha=1.0):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.fig.tight_layout()

        self.update_interval = update_interval

        self.alpha = alpha
        self.data = dict()
        self.objects = dict()
        self.ani = None

    def add_data_point(self, object_name, x, y):
        if object_name not in self.data:
            self.data[object_name] = {
                "x": [],
                "y": []
            }

            line, = self.ax.plot([], [], label=object_name, alpha=self.alpha)

            self.objects[object_name] = line

        self.data[object_name]["x"].append(x)
        self.data[object_name]["y"].append(y)

        self.objects[object_name].set_xdata(self.data[object_name]['x'])
        self.objects[object_name].set_ydata(self.data[object_name]['y'])

    def func_animate(self, i):
        try:
            self.ax.legend()
            self.ax.relim()
            self.ax.autoscale_view()
        except ValueError:
            print("Error graphing")

        return self.objects.values()

    def show(self):
        self.ani = FuncAnimation(self.fig, self.func_animate, interval=self.update_interval)

        plt.show()


if __name__ == '__main__':
    plotter = LivePlotter()
    plotter.show()
