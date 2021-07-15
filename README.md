# AprilTagWrapper


April Tag detection and tracking with Extended Kalman Filter.
The basic April Tag functionality has been improved with Opencv interface and Extended Kalman Filter.
     

## How to install 
To install this library using pip 
```bash 
pip3 install git+https://github.com/RedwanNewaz/AprilTagWrapper.git
```

## Quick Start 


```python 
from AprilTagWrapper import camera, ATW

if __name__ == '__main__':
    tagNames = ["tag36h11"] # make sure tag names are in lower case
    atw = ATW(tagNames, frameRate=30)
    cam = camera(camParam=0, FRAME_WIDTH=1920, FRAME_HEIGHT=1080) # camParam = webcam int index | default webcam index is 0

    for frame in cam.run():
        detections = atw.detect(frame)
        centers = {tagName:atw(tagName) for tagName in tagNames}
        if not all(centers.values()):
            continue
        cam.overlay_frame(centers.values(), detections)
```
