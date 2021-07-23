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
    tagIDs= [20, 21, 22, 23]
    atw = ATW(tagIDs, frameRate=30)
    cam = camera(camParam=2, FRAME_WIDTH=1920, FRAME_HEIGHT=1080) # camParam = webcam int index | default webcam index is 0

    for frame in cam.run():
        detections = atw.detect(frame)
        centers = {tagName:atw(tagName) for tagName in tagIDs}
        if not all(centers.values()):
            continue
        cam.overlay_frame(centers.values(), detections)
```

## Docker integration 

You need a camaera.param file which contains camera calibration parameters.
```txt
  fx = 1317.1810580277324
  fy = 1311.5926572264152
  cx = 959.4999999687046
  cy = 539.5000001152366
```
The text are not imporant just need to define **fx, fy, cx, cy** in four lines. 
Once you have it try the following configuration 


```yaml
version: "3.0"

services:
  apriltag:
    build: "."
    container_name: apriltag
    ports:
    - 5000:500
    privileged: true
    volumes:
      - /dev/video0:/dev/video0 # share the camera index
      - ./camera.param:/app/camera.param # share the camera parameter
    restart: unless-stopped
 ```
 
 To share an external camera change this line 
 **- /dev/video0:/dev/video0** to **- /dev/video1:/dev/video0**
 assuming that your external camera index is 1 
