from picamera.array import PiRGBArray
from picamera import PiCamera
from PiMotor import Motor, Arrow
import time
import cv2

class Ultrasonic:

    def __init__(self, trig, echo):
        pass

drive = None
steer = None
cam   = None
cap   = None
front = None
left  = None
right = None
back  = None

def main():
    # Initializing all the variables
    drive = Motor("MOTOR1",2)
    steer = Motor("MOTOR2",1)

    # Camera setup
    cam   = PiCamera()
    cam.resolution = (640,480)
    cam.framerate  = 32
    cap   = PiRGBArray(cam)

    # Camera warm up
    time.sleep(.1)

    for frame in camera.capture_continuous(cap, format='bgr', use_video_port=True):
        img = frame.array

        cv2.imshow("Frame", img)
        key = cv2.waitKey(1) & 0xFF

        cap.truncate(0)

        if key == ord('q'):
            break


if __name__ == "__main__":
    main()
