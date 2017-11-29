from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import numpy
import cv2

class Ultrasonic:

    def __init__(self, trig, echo):
        pass

cam   = None
cap   = None

def main():

    # Camera setup
    cam   = PiCamera()
    cam.framerate  = 32
    cap   = PiRGBArray(cam)

    # Camera warm up
    time.sleep(.1)


    GPIO.setmode(GPIO.BCM)

    GPIO.setup(18, GPIO.OUT) # Lifting servo
    GPIO.setup( 2, GPIO.OUT) # camera servo
    lift = GPIO.PWM(18, 100)
    head = GPIO.PWM( 2, 100)

    curr = 0

    def update_servo(servo, angle):
        duty = float(angle) / 10.0 + 2.5
        if servo == 1: # Bottom
            lift.ChangeDutyCycle(duty)
        else:
            head.ChangeDutyCycle(duty)

    for frame in cam.capture_continuous(cap, format='bgr', use_video_port=True):
        img = frame.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = cv2.CascadeClassifier("/files/haarcascade_frontalface_default.xml").detectMultiscale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30,30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )

        if curr % 2:
            print("At 0!")
            update_servo(1, 0)
            time.sleep(5)
        else:
            print("At 90!")
            update_servo(0, 90)
            time.sleep(5)

        
        # for (x,y,w,h) in faces:
        #     cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)
        #
        # cv2.imshow("Face Tracking", img)

        key = cv2.waitKey(1) & 0xFF

        cap.truncate(0)

        if key == ord('q'):
            break


if __name__ == "__main__":
    main()
