from AprilTagWrapper import camera, ATW
from flask import Flask, jsonify
from threading import Thread
'''
To use this example install Flask using 
pip3 install flask 
'''
centers = {}

def apriltag_wrapper_thread():
    global centers
    tagIDs= [20, 21, 22, 23]
    atw = ATW(tagIDs, frameRate=30)
    cam = camera(camParam=2, FRAME_WIDTH=1920, FRAME_HEIGHT=1080, display=False) # camParam = webcam int index | default webcam index is 0

    for frame in cam.run():
        detections = atw.detect(frame)
        centers = {tagName:atw(tagName) for tagName in tagIDs}
        if not all(centers.values()):
            continue
        print(centers)

app = Flask(__name__)

@app.route('/')
def index():
    return jsonify(centers)

if __name__ == '__main__':
    atwt = Thread(target=apriltag_wrapper_thread)
    atwt.start()
    app.run(host="0.0.0.0", port=5000)
    atwt.join()