#!/usr/bin/env python

from time import sleep

import rospy
from cv_bridge import CvBridge, CvBridgeError
from picamera import PiCamera, PiCameraError
from picamera.array import PiRGBArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ImageCapture:
    """
    Class containing a ros publisher and subscirber.

    Subscriber listens to the shutter topic which triggers the callback function.
    The callback function takes an image using the Raspberry Pi Camera and publishes
    it to the images topic.
    """

    def __init__(self):
        """Initialise ros publisher and subscriber."""
        # Camera initialisation.
        try:
            self.camera = PiCamera()
            self.rawCapture = PiRGBArray(self.camera)
            sleep(0.1)  # Sleep to give camera time to start up.
        except PiCameraError:
            rospy.loginfo("Cannot Initialise picamera.")

        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber('shutter', Bool, self.callback, queue_size=1)
        self.publisher = rospy.Publisher('images', Image, queue_size=5)

    def callback(self, ros_data):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Takes an image using the Raspberry Pi Camera and publishes it.
        """
        rospy.loginfo("Capturing image.")
        self.camera.capture(self.rawCapture, format='bgr')
        cv2_image = self.rawCapture.array

        rospy.loginfo("Publishing image.")
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv2_image, 'bgr8')
            self.publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)


def main():
    """Initialize and cleanup ros node."""
    image_capture = ImageCapture()
    rospy.init_node('image_capture', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS ImageCapture module."


if __name__ == '__main__':
    main()
