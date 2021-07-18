import apriltag

import cv2

class AprilTagWrapper:

    def __init__(self, tagIDs, frameRate, filterClass):
        '''
        :param tagIDs: a list of april tag names
        :param frameRate: camera FPS
        '''
        assert isinstance(tagIDs, list)

        self.tagIDs = tagIDs
        self.detector = apriltag.Detector(
                                 searchpath=apriltag._get_demo_searchpath())
        self.ekf = [filterClass(FPS=frameRate) for _ in range(len(tagIDs))]



    def detect(self, frame):
        '''
        :param frame: BGR cv2 frame needs to be converted to gray scale image
        :return: detection results
        '''
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections, dimg = self.detector.detect(gray, return_image=True)



        return detections

    def update_filter(self, args, coordType):
        if coordType is 'ImageCoord':
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
        assert tagID in self.tagIDs

        index = self.tagIDs.index(tagID)
        tagState = next(self.ekf[index])

        if tagState is None:
            return

        # convert it tuple for the sake of cv point
        # tagPoint = tuple(tagState.astype('int')[:2].tolist())

        return tagState