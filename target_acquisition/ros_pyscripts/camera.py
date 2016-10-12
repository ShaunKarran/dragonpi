from time import sleep

import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from picamera import PiCamera, PiCameraError
from picamera.array import PiRGBArray
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty


class Camera:
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
        self.subscriber = rospy.Subscriber('take_image', Empty, self.callback, queue_size=1)
        # self.publisher = rospy.Publisher('images', Image, queue_size=1)
        self.publisher = rospy.Publisher('images', CompressedImage, queue_size=1)

    def callback(self, req):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Takes an image using the Raspberry Pi Camera and publishes it.
        """
        rospy.loginfo("Capturing image.")
        self.rawCapture.truncate(0)
        self.camera.capture(self.rawCapture, format='bgr')
        cv_image = self.rawCapture.array

        rospy.loginfo("Publishing image.")
        try:
            ros_image = CompressedImage()
            ros_image.header.stamp = rospy.Time.now()
            ros_image.format = 'jpeg'
            ros_image.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            # ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)


def main():
    """Initialize and cleanup ros node."""
    camera = Camera()
    rospy.init_node('camera', anonymous=False)
    try:
        rospy.loginfo("Camera node running.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Camera module."


if __name__ == '__main__':
    main()
