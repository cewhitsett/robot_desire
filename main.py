# To convert picamera image to opencv
from picamera.array import PiRGBArray
from picamera import PiCamera
# Code provided by motor shield
import PiMotor
import RPi.GPIO as GPIO
import time
import numpy
import cv2
import random

def main():
    # motor for tail
    tail  = PiMotor.Motor("MOTOR1", 1)
    cam   = PiCamera()
    cam.resolution = (640,480)
    cam.framerate  = 32
    cap   = PiRGBArray(cam)
    # Classifier for front of face. Must be absolute path 
    # for autoboot
    classifier = cv2.CascadeClassifier("/home/pi/robot_desire/files/haarcascade_frontalface_default.xml")
    # Camera warm up
    time.sleep(.1)
    close = -1000

    # Setting up servo
    GPIO.setup( 3, GPIO.OUT)

    head = GPIO.PWM( 3, 100)
    head_ang = 100
    last_seen = time.time()

    head.start(7.5)
            
    # Moves head to desired angle
    def swervo():
        duty = float(head_ang) / 10.0 + 2.5
        head.ChangeDutyCycle(duty)

    # Shaking head to make sure user knows
    # everything is ready
    print("waking up")
    for i in range(5):
        if i % 2:
            head_ang = 50
        else:
            head_ang = 0
        swervo()
        time.sleep(2)

    # Loop to generate images using the picamera
    for frame in cam.capture_continuous(cap, format='bgr', use_video_port=True):
        # getting the image from frame
        img = frame.array

        # opencv algs are in grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 

        # List of faces
        faces = classifier.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30,30),
                flags=cv2.CASCADE_SCALE_IMAGE
                )

        if len(faces) != 0:
            close=faces[0]
        else:
            close=-1000

        for (x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)

        avg_y = 0
        # No need to show image unless you have
        # some form of gui. Jessie lite does not
        # cv2.imshow("Image", img)
        closest = 1000000
        for face in faces:
            x,y,w,h = face
            avg_y += y
            dist = (x-close[0])**2 + (y-close[1])**2
            if dist < closest:
                close = x, y, w, h
                closest = dist
        if len(faces):
            avg_y = float(avg_y)/len(faces)

        if close != -1000:
            print("seen")
            last_seen = time.time()
            
            # Moving came to center faces
            if avg_y > 270 and head_ang >=  10:
                head_ang -= 10
                print("you're too low")
            elif avg_y < 210 and head_ang <= 90:
                head_ang += 10
                print("you're too high")
            # move tail if facing someone
            if len(faces):
                tail.forward(100)
        else:
            # Move head if board, haha
            if time.time() - last_seen > 10:
                head_ang = random.randint(50,100)
                last_seen = time.time()
                print("moving head")
            tail.stop()

        swervo()
        # This is to stop program, however, only
        # works when GUI is enabled
        # key = cv2.waitKey(1) & 0xFF
        #
        # cap.truncate(0)
        #
        # if key == ord('q'):
        #     break


if __name__ == "__main__":
    main()
