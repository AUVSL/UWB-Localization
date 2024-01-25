#! /usr/bin/env python
# coding=utf-8
import threading

import rospy
from geometry_msgs.msg import Twist
from rospy import Rate


class Robot:
    def __init__(self, class_name, id):
        self.pub = rospy.Publisher("/" + class_name + str(id) + "/cmd_vel", Twist, queue_size=1, latch=True)

        self.data = Twist()

    def set(self, x=0, y=0, z=0, ax=0, ay=0, az=0):
        self.data.linear.x = x
        self.data.linear.y = y
        self.data.linear.z = z
        self.data.angular.x = ax
        self.data.angular.y = ay
        self.data.angular.z = az

        self.pub.publish(self.data)


class Drone(Robot):
    def __init__(self, id):
        Robot.__init__(self, 'drone', id)


class Jackal(Robot):
    def __init__(self, id):
        Robot.__init__(self, 'Jackal', id)


class RobotGroup:
    def __init__(self, num, robot_type):
        self.drones = [robot_type(i + 1) for i in range(num)]

    def set(self, x=0, y=0, z=0, ax=0, ay=0, az=0):
        for drone in self.drones:
            drone.set(x, y, z, ax, ay, az)


class TaskTimer(threading.Thread):
    """
    Convenience class for calling a callback at a specified rate
    """

    def __init__(self, seconds, callback, rate=60):
        """
        Constructor.
        @param period: desired period between callbacks
        @type  period: rospy.Duration
        @param callback: callback to be called
        @type  callback: function taking rospy.TimerEvent
        @param oneshot: if True, fire only once, otherwise fire continuously until shutdown is called [default: False]
        @type  oneshot: bool
        @param reset: if True, timer is reset when rostime moved backward. [default: False]
        @type  reset: bool
        """
        super(TaskTimer, self).__init__()
        self.rate = rate
        self._period = seconds
        self._callback = callback
        self._oneshot = True
        self._reset = False
        self._shutdown = False
        self.daemon = True
        self.started = False
        self.max_counter = self.rate * self._period

    def start(self):
        super(TaskTimer, self).start()
        self.started = True

    def shutdown(self):
        """
        Stop firing callbacks.
        """
        self._shutdown = True

    def run(self):
        r = Rate(self.rate, reset=self._reset)
        print self.rate

        counter = 0

        while not rospy.core.is_shutdown() and not self._shutdown:
            if self._shutdown:
                break

            self._callback()

            counter += 1

            if counter >= self.max_counter:
                break
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                if rospy.core.is_shutdown():
                    break
                raise


class SystemTasks:
    def __init__(self, robot_groups):
        self.robot_groups = robot_groups

        self.tasks = {}
        for key in robot_groups.keys():
            self.tasks[key] = []

    def add_task(self, seconds, robot_group, **vel_command):
        self.tasks[robot_group].append(
            TaskTimer(seconds, lambda: self.robot_groups[robot_group].set(**vel_command)))

    def run(self):
        rate = Rate(1)

        while not rospy.is_shutdown() and len(self.tasks) > 0:
            for key, robot_tasks in self.tasks.items():
                current_task = robot_tasks[0]

                if current_task.started and not current_task.is_alive():
                    robot_tasks.pop(0)

                    if len(robot_tasks) == 0:
                        del self.tasks[key]
                elif not current_task.started:
                    current_task.start()

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("JackalMover", anonymous=True)

    drones = RobotGroup(3, Drone)
    jackals = RobotGroup(2, Jackal)

    system_tasks = SystemTasks({
        Drone: drones,
        Jackal: jackals
    })

    system_tasks.add_task(1, Drone, z=1)
    system_tasks.add_task(0, Drone, z=0)
    system_tasks.add_task(5, Jackal, x=2)

    system_tasks.run()
