import apriltag

import cv2
from scipy.spatial.transform import Rotation as R
import numpy as np

class AprilTagWrapper:
    nofilter = False
    def __init__(self, tagIDs, frameRate, filterClass = None):
        '''
        :param tagIDs: a list of april tag names
        :param frameRate: camera FPS
        '''
        assert isinstance(tagIDs, list)

        self.tagIDs = tagIDs
        self.detector = apriltag.Detector(
                                 searchpath=apriltag._get_demo_searchpath())
        if filterClass is None:
            self.nofilter = True
        else:
            self.ekf = [filterClass(FPS=frameRate) for _ in range(len(tagIDs))]



    def detect(self, frame):
        '''
        :param frame: BGR cv2 frame needs to be converted to gray scale image
        :return: detection results
        '''
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.detections, dimg = self.detector.detect(gray, return_image=True)



        return self.detections

    def update_filter(self, args, coordType):

        if self.nofilter:
            return

        elif coordType is 'ImageCoord':
            # iterate over all the detection tags
            for detect in args:
                family_id = detect.tag_id
                if family_id in self.tagIDs:
                    index = self.tagIDs.index(family_id)
                    # update specific ekf
                    self.ekf[index](detect.center[0], detect.center[1])

        elif coordType is 'WorldCoord':
            for tag in args:
                for tagId, value in tag.items():
                    indx = self.tagIDs.index(tagId)
                    self.ekf[indx](value['pose'])

    def __call__(self, tagID):
        '''
        :param tagID: string tag name
        :return: center point of the tag
        '''
        if self.nofilter: return
        assert tagID in self.tagIDs

        index = self.tagIDs.index(tagID)
        tagState = next(self.ekf[index])

        if tagState is None:
            return

        # convert it tuple for the sake of cv point
        # tagPoint = tuple(tagState.astype('int')[:2].tolist())

        return tagState

    def get_world_coords(self, frame, K):
        '''
        frmae: rgb frame
        :return: world coordinates
        '''
        return  self.world_coordinates(self.detect(frame), K)

    def world_coordinates(self, detections, K):
        '''
        :param detections: M is a 4x4 transformation matrix with 3x3 rotation matrix
        and 3x1 translation matrix
        :return: tag poses in euler domain
        '''
        for detect in detections:
            M, init_error, final_error = self.detector.detection_pose(detect, K)
            pose = self.transformation_mat_to_pose(M)
            yield {detect.tag_id: {"pose": pose, 'init_error': init_error, 'final_error': final_error}}

    @staticmethod
    def transformation_mat_to_pose(M):
        '''
        :param M: 4 x 4 transformation matrix with 3x3 rotation matrix
        and 3x1 trannslation vector
        :return: 6D pose list (3 position and 3 euler angles)
        '''
        rotation = M[:3, :3]
        translation = M[:3, 3]
        euler = R.from_matrix(rotation).as_euler('xyz', degrees=False)  # (roll, pitch, yaw)
        pose = np.hstack((translation, euler))
        return pose
