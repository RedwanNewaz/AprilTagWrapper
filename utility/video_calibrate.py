from AprilTagWrapper import camera
import cv2


def save_webcame_image(patternsize):
    '''
    :param patternsize: (row, col)
    :return: save images
        chess.png : rgb image from camera
        result.png: detection results overlayed on chess board
    '''
    cam = camera(camParam=2)
    for frame in cam.run():
        fresh = frame.copy()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        retval, corners = cv2.findChessboardCorners(frame, patternsize)
        cv2.drawChessboardCorners(frame, patternsize, corners, retval)
        if retval:
            print(retval, corners)
            cv2.imwrite('results/chess2.png', fresh)
            cv2.imwrite('results/result2.png', frame)
            break



if __name__ == '__main__':

    patternsize = (7, 5) # row, col
    save_webcame_image(patternsize)



