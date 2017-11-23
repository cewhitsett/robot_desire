import picamera
import time
# from PiMotor import Motor, Arrow


def main():
    # drive = PiMotor("MOTOR1",2)
    # steer = PiMotor("MOTOR2",1)

    cam   = picamera.PiCamera()

    cam.start_preview()

    time.sleep(10)

    cam.stop_preview()


if __name__ == "__main__":
    main()
