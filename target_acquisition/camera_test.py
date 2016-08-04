#!/usr/bin/env python
from time import sleep

from termcolor import colored

from picamera import PiCamera, PiCameraError


def main():
    """Script to test the Raspberry Pi Camera."""
    # Camera initialisation.
    try:
        camera = PiCamera()
        sleep(0.1)  # Sleep to give camera time to start up.
    except PiCameraError:
        print colored('FAILED - ', 'red'),
        print "Camera count not be initialised."

    # Take image and write to file.
    camera.capture('./camera_test.jpg')
    print colored('PASSED', 'green')


if __name__ == '__main__':
    main()
