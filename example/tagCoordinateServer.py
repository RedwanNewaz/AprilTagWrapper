import re
from AprilTagWrapper import camera, ATW
from flask import Flask, jsonify
from threading import Thread
from argparse import ArgumentParser

payloads = {}

def read_camera_parameters(file):
    '''
    :param file: a file that contains fx, fy, cx, cy parameters in order
    :return: list (fx, fy, cx, cy)
    '''
    with open(file) as File:
        data = File.read()

    data = re.findall("\d+\.\d+", data)
    return list(map(float, data))


def publish_coordinates(args):
    '''
    april tag entry points
    :return: update payload dictionary
    '''
    global payloads
    tagIDs= [20, 21, 22, 23]
    # select appropriate index for your camera. O is the default camera index
    camera_index = args.camera
    atw = ATW(tagIDs, frameRate=30, filterClass=None)
    cam = camera(camParam=camera_index, FRAME_WIDTH=1920, FRAME_HEIGHT=1080, display=False) # camParam = webcam int index | default webcam index is 0

    webcam2 = args.K
    K = read_camera_parameters(webcam2)

    for frame in cam.run():
        coords = atw.get_world_coords(frame, K)
        for cand in coords:
            for tag, value in cand.items():
                pose = value['pose'].tolist()
                payloads[tag] = pose

app = Flask(__name__)

@app.route('/')
def index():
    '''
    flask server entry point
    :return: send payloads on request
    '''
    return jsonify(payloads)

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--camera', type=int, default=2, help='cv camera index')
    parser.add_argument('--K', type=str, default='../utility/results/K_webcam2.param',  help="camera intrinsic parameter file")

    args = parser.parse_args()
    atwt = Thread(target=publish_coordinates, args=(args,))
    atwt.start()
    app.run(host="0.0.0.0", port=5000)
    atwt.join()