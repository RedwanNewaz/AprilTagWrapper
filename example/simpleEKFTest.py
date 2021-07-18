from AprilTagWrapper import camera, ATW
from AprilTagWrapper.tracker import EKFImageCoord
import numpy as np

if __name__ == '__main__':
    tagIDs= [20, 21, 22, 23]
    atw = ATW(tagIDs, frameRate=30, filterClass=EKFImageCoord)
    cam = camera(camParam=2, FRAME_WIDTH=1920, FRAME_HEIGHT=1080) # camParam = webcam int index | default webcam index is 0

    for frame in cam.run():
        detections = atw.detect(frame)
        atw.update_filter(detections, coordType="ImageCoord")

        centers = {tagName:atw(tagName) for tagName in tagIDs}
        initialized = all([atw.ekf[index].initialization for index in range(len(tagIDs))])
        if not initialized:
            continue
        centers = {key:np.squeeze(value).astype('int')[:2] for key, value in centers.items()}
        cam.overlay_frame(centers.values(), detections)
