#!/home/redwan/anaconda3/envs/aprilTag/bin/python

from AprilTagWrapper import camera, ATW
from flask import Flask, jsonify
from threading import Thread
import re
from scipy.spatial.transform import Rotation as R
import numpy as np
'''
To use this example install Flask and scipy using 
pip3 install flask 
pip3 install scipy
'''
centers = {}


def read_camera_parameters(file):
    '''
    :param file: a file that contains fx, fy, cx, cy parameters in order
    :return: list (fx, fy, cx, cy)
    '''
    with open(file) as File:
        data = File.read()

    data = re.findall("\d+\.\d+", data)
    return list(map(float, data))

def world_coordinates(detections, atw, K):
    '''
    :param detections: M is a 4x4 transformation matrix with 3x3 rotation matrix
    and 3x1 translation matrix
    :return: tag poses in euler domain
    '''
    for detect in detections:
        M, init_error, final_error = atw.detector.detection_pose(detect, K)
        rotation = M[:3, :3]
        translation = M[:3, 3]
        euler = R.from_matrix(rotation).as_quat() # quaternion (roll, pitch, yaw)
        pose = np.hstack((translation, euler)).tolist()
        yield {detect.tag_id: {"pose": pose, 'init_error': init_error, 'final_error': final_error}}


def apriltag_wrapper_thread():
    global centers
    webcam2 = '../utility/results/K_webcam2.param'
    K = read_camera_parameters(webcam2)
    tagIDs = [20, 21, 22, 23]
    atw = ATW(tagIDs, frameRate=30)
    cam = camera(camParam=2, FRAME_WIDTH=1920, FRAME_HEIGHT=1080,
                 display=False)  # camParam = webcam int index | default webcam index is 0

    for frame in cam.run():
        detections = atw.detect(frame)
        if not all(centers.values()):
            continue
        coords = list(world_coordinates(detections, atw, K))
        if len(coords):
            for item in coords:
                for key, value in item.items():
                    centers[key] = value
            # print('[Pose]: ', centers)

        # cam.overlay_frame(centers.values(), detections)

app = Flask(__name__)

@app.route('/')
def index():
    return jsonify(centers)

if __name__ == '__main__':
    atwt = Thread(target=apriltag_wrapper_thread)
    atwt.start()
    app.run(host="0.0.0.0", port=5001, debug = False)
    atwt.join()