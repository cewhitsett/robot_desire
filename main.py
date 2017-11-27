from picamera.array import PiRGBArray
from picamera import PiCamera
from PiMotor import Motor, Arrow
import time
import numpy
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

	# Loop to generate images using the picamera
    for frame in cam.capture_continuous(cap, format='bgr', use_video_port=True):
		# getting the image from frame
		img = frame.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = cv2.CascadeClassifier("files/haarcascade_frontalface_default.xml").detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30,30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        for (x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.imshow("Face Tracking", img)
        key = cv2.waitKey(1) & 0xFF

        cap.truncate(0)

        if key == ord('q'):
            break


if __name__ == "__main__":
    main()
