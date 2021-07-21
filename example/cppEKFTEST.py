import sys
import re
from AprilTagWrapper import camera, ATW
sys.path.append("PoseEKF")
from PoseEKF import PoseEKFWrapper
from scipy.spatial.transform import Rotation as R

def read_camera_parameters(file):
    '''
    :param file: a file that contains fx, fy, cx, cy parameters in order
    :return: list (fx, fy, cx, cy)
    '''
    with open(file) as File:
        data = File.read()

    data = re.findall("\d+\.\d+", data)
    return list(map(float, data))

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


def main():
    tagIDs= [20, 21, 22, 23]
    atw = ATW(tagIDs, frameRate=30, filterClass=None)
    cam = camera(camParam=2, FRAME_WIDTH=1920, FRAME_HEIGHT=1080, display=True) # camParam = webcam int index | default webcam index is 0

    webcam2 = '../utility/results/K_webcam2.param'
    K = read_camera_parameters(webcam2)
    filters = {tag:PoseEKFWrapper(0.5, 1e-7, 5) for tag in tagIDs}

    initialize = False
    for frame in cam.run():
        coords = atw.get_world_coords(frame, K)
        update  = False
        for cand in coords:
            for tag, value in cand.items():
                pose = value['pose']
                if not pose is None:
                    ekf = filters.get(tag)
                    ekf.update(pose.tolist())
                    initialize = True
                    update = True

        if update:
            cam.draw_bounding_box(frame, atw.detections)
        # update animation
        if initialize:
            viz_data = {}
            for tag in tagIDs:
                ekf = filters.get(tag)
                viz_data[tag] = ekf.estimate()[:6]
            show_coordinate(viz_data)


if __name__ == '__main__':
    main()