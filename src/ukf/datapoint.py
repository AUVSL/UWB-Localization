class DataType:
    LIDAR = 1
    ODOMETRY = 2
    UWB = 3
    GROUND_TRUTH = 0


class DataPoint:
    def __init__(self, data_type, measurement_data, timestamp, extra=None):
        self.extra = extra
        self.data_type = data_type
        self.measurement_data = measurement_data
        self.timestamp = timestamp
