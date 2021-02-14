class DataType:
    LIDAR = 1
    ODOMETRY = 2
    UWB = 3


class DataPoint:
    def __init__(self, data_type, measurement_data, timestamp, extra=None) -> None:
        super().__init__()
        self.extra = extra
        self.data_type = data_type
        self.measurement_data = measurement_data
        self.timestamp = timestamp
