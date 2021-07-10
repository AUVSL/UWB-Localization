# coding=utf-8
class DataType(object):
    LIDAR = 1
    ODOMETRY = 2
    UWB = 3
    IMU = 4
    GROUND_TRUTH = 0


class DataPoint(object):
    def __init__(self, data_type, measurement_data, timestamp, extra=None):
        self.extra = extra
        self.data_type = data_type
        self.measurement_data = measurement_data
        self.timestamp = timestamp

    def __repr__(self):
        return str(self.data_type) + ":" + str(self.timestamp) + ": " + str(self.measurement_data)
