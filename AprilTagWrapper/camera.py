import cv2

class camera(object):
    # fontScale for text
    fontScale = 1
    # Blue color in BGR
    color = (0, 255, 0)
    # Line thickness of 2 px
    thickness = 2
    # Radius of center circle
    radius = 20
    # Red color in BGR
    center_color = (0, 0, 255)
    # set display resolution
    DISPLAY_WIDTH = 1280
    # display height
    DISPLAY_HEIGHT = 720
    def __init__(self, camParam, webcam = True, display = True,
                 FRAME_WIDTH = 1920, FRAME_HEIGHT=1080,
                 ):
        if webcam:
            assert isinstance(camParam, int)
            self.cap = cv2.VideoCapture(camParam)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            print(self.set_res(FRAME_WIDTH, FRAME_HEIGHT))
        else:
            self.cap = cv2.VideoCapture(camParam)

        self.display = display

        if display:
            self.windowName = "AprilTagWrapper"
            cv2.namedWindow(self.windowName, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.windowName, self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT)


    def set_res(self, x, y):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(x))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(y))
        return str(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), str(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


    def run(self):

        while True:
            success, self.frame = self.cap.read()
            k = cv2.waitKey(1)
            if not success or k == 27:
                break
            yield self.frame

            if self.display:
                cv2.imshow(self.windowName, self.frame)

    @staticmethod
    def corners2cvbox(corners):
        '''
        :param corners: apriltag detection.corners
        :return: top left and botom right points
        '''
        # represents the top left corner of rectangle
        start_point = (int(corners[0][0]), int(corners[0][1]))
        # represents the bottom right corner of rectangle
        end_point = (int(corners[2][0]), int(corners[2][1]))

        return start_point, end_point


    def show_coord(self, image, detections, centers):
        font = cv2.FONT_HERSHEY_SIMPLEX
        # get center position

        # org

        for detection, x in zip(detections, centers):
            org = (int(x[0]), int(x[1]))
            start_point, end_point = self.corners2cvbox(corners=detection.corners)

            # Using cv2.putText() method
            image = cv2.putText(image, '[{}]'.format(org), org, font,
                                self.fontScale, self.color, self.thickness, cv2.LINE_AA)
            image = cv2.rectangle(image, start_point, end_point, self.color, self.thickness)

        return image

    def overlay_frame(self, centers, detection):

        for center in centers:
           self.frame = cv2.circle(self.frame, center, self.radius, self.center_color, self.thickness)
        self.frame = self.show_coord(self.frame, detection, centers)

    def draw_bounding_box(self, image, detections):
        for detection in detections:
            start_point, end_point = self.corners2cvbox(corners=detection.corners)
            # Using cv2.putText() method
            image = cv2.rectangle(image, start_point, end_point, self.color, self.thickness)

        return image




