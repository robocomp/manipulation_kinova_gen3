import numpy


class JointsData:
    def __init__(self, timestamp, joints):
        self.timestamp = timestamp
        self.joints = joints
        pass

    def getJoints(self):
        return self.joints

    def getTimestamp(self):
        return self.timestamp

    def setJoints(self, joints):
        self.joints = joints
        pass

    def setTimestamp(self, timestamp):
        self.timestamp = timestamp
        pass
