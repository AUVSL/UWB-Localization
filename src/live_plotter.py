# coding=utf-8
from __future__ import print_function

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class LivePlotter(object):
    def __init__(self, update_interval=1000, alpha=1.0, window_name=None, time_removal=None, lengend_loc='best', font_size=None):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.fig.tight_layout()
        if window_name is not None:
            self.fig.canvas.set_window_title(window_name)

        self.update_interval = update_interval

        self.alpha = alpha
        self.data = dict()
        self.objects = dict()
        self.ani = None
        self.time_removal = time_removal
        self.legend_loc = lengend_loc
        self.font_size = font_size

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
            if self.time_removal is None or i < self.time_removal:
                if self.time_removal is not None:

                self.ax.legend(loc=self.legend_loc, fontsize=self.font_size)
            else:
                self.ax.legend_ = None
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
