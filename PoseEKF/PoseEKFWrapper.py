import os
current_path =os.path.abspath(__file__)
current_dir = os.path.dirname(current_path)
import sys
sys.path.append(current_dir)
from python_pose_ekf import python_pose_ekf


class PoseEKFWrapper(python_pose_ekf):
    def __init__(self, processNoise, measurementNoise, mahalanobisThresh):
        super().__init__(processNoise, measurementNoise, mahalanobisThresh)
        self.last_time = self.get_ms()/1000.0

    def update(self, pose):
        self.set(pose)
        self.last_time = self.get_ms()/1000.0

    def estimate(self):
        current_time = self.get_ms()/1000.0
        return self.get(self.last_time, current_time)