from AprilTagWrapper import camera, ATW
from AprilTagWrapper.tracker import EKFCameraWorldCoord
import re
import numpy as np
from scipy.spatial.transform import Rotation as R
from flask import Flask, jsonify
from threading import Thread
import logging
import time
import sys
sys.path.append("PoseEKF")
from PoseEKF import PoseEKFWrapper

# logging.basicConfig(level=logging.DEBUG)

'''
To use this example install Flask and scipy using 
pip3 install flask 
pip3 install scipy
for visualization use matplotlib
pip3 install matplotlib
'''
show_animation = True
if show_animation:
    import matplotlib.pyplot as plt
    from utility.viz_world_coordinates import CoordAxis
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')


def show_coordinate(poses):
    ax.cla()
    ax.set_xlim3d(-15, 15)
    ax.set_ylim3d(-15, 15)
    ax.set_zlim3d(0, 35)

    for i, value in enumerate(poses.values()):
        # xs, ys, zs = value[:3]
        # ax.scatter(xs, ys, zs)
        ref = CoordAxis(ax, length=3)
        ref.transform(value)
        ref.plot()

    plt.pause(0.0001)

def transformation_mat_to_pose(M):
    rotation = M[:3, :3]
    translation = M[:3, 3]
    euler = R.from_matrix(rotation).as_euler('xyz', degrees=False)  # (roll, pitch, yaw)
    pose = np.hstack((translation, euler))
    return pose

def inertial_frame_transformation(pose):
    position, orientation = pose[:3], pose[3:]
    # print(position, orientation)
    body_frame = np.eye(4)
    rotation_matrix = R.from_euler('xyz', orientation, degrees=False).as_matrix()
    body_frame[:3, :3] = rotation_matrix
    body_frame[:3, 3] = position


    I_euler = [-1.57, 0, 0]
    I_rotation_matrix = R.from_euler('xyz', I_euler, degrees=False).as_matrix()

    body_frame[:3, :3] = I_rotation_matrix @ body_frame[:3, :3]
    position = body_frame[:3, :3] @ position
    body_frame[:3, 3] = position
    # print(position)


    return transformation_mat_to_pose(body_frame)
    # return  pose

    # world_frame = Inertial_frame * body_frame
    # print(world_frame)
    # return transformation_mat_to_pose(world_frame)



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
        pose = transformation_mat_to_pose(M)
        pose = inertial_frame_transformation(pose)
        yield {detect.tag_id: {"pose": pose, 'init_error': init_error, 'final_error': final_error}}

states = {}
def apriltag_wrapper_thread():
    global states
    tagIDs= [20, 21, 22, 23]
    atw = ATW(tagIDs, frameRate=30, filterClass=EKFCameraWorldCoord)
    cam = camera(camParam=2, FRAME_WIDTH=1920, FRAME_HEIGHT=1080, display=True) # camParam = webcam int index | default webcam index is 0

    webcam2 = '../utility/results/K_webcam2.param'
    K = read_camera_parameters(webcam2)
    pose_ekf = PoseEKFWrapper(0.5, 1e-9, 5)
    for frame in cam.run():
        detections = atw.detect(frame)

        coords = list(world_coordinates(detections, atw, K))
        if len(coords):
            for tag in coords:
                for tagId, value in tag.items():
                    if tagId == 20:
                        pose_ekf.update(value['pose'].tolist())
            atw.update_filter(coords, coordType="WorldCoord")
            # print(coords)
        initialized = all([atw.ekf[index].initialization for index in range(len(tagIDs))])
        if not initialized:
            continue
        states = {tagName: np.squeeze(atw(tagName)).tolist()[:6] for tagName in tagIDs}
        logging.debug("{}".format(states))
        print(pose_ekf.estimate())

app = Flask(__name__)

@app.route('/')
def index():
    return jsonify(states)

if __name__ == '__main__':
    atwt = Thread(target=apriltag_wrapper_thread)
    atwt.start()
    if show_animation:
        while True:
            if len(states):
                show_coordinate(states)
    else:
        app.run(host="0.0.0.0", port=5001, debug = False)
        atwt.join()


