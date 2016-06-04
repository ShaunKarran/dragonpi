#!/usr/bin/env python

# Python libs
import argparse
import imutils
import sys
import time

# OpenCV
import cv2

# Ros libs
import rospy

# Ros messages
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

# Raspberry Pi Camera
from picamera import PiCamera, PiCameraError
from picamera.array import PiRGBArray


class Image_Processor:
    """
    Class containing a ros publisher and subscirber.

    Subscriber listens to the shutter topic which triggers the callback function.
    The callback function takes an image using the Raspberry Pi Camera and publishes
    it to the images topic.
    """

    def __init__(self, cascades, resize_width):
        """Initialise ros publisher and subscriber."""
        self.resize_width = resize_width
        self.targets = {}
        for cascade in cascades:
            self.targets[cascade] = cv2.CascadeClassifier(cascade)

        # Do this to initialise the camera.
        try:
            self.camera = PiCamera()
            self.rawCapture = PiRGBArray(self.camera)
            time.sleep(0.1)
            self.camera_initialised = True
        except PiCameraError:
            rospy.loginfo('Cannot initialise picamera.')
            self.camera_initialised = False

        self.publisher = rospy.Publisher("images", Image, queue_size=5)

        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("shutter", Bool, self.callback, queue_size=1)

    def callback(self, ros_data):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Takes an image using the Raspberry Pi Camera and publishes it.
        """
        start_time = time.time()

        if self.camera_initialised:
            rospy.loginfo('{:.2f}s: Taking image.'.format(time.time() - start_time))
            self.camera.capture(self.rawCapture, format="bgr")
            cv2_image = self.rawCapture.array
        else:
            rospy.loginfo('{:.2f}s: Loading image.'.format(time.time() - start_time))
            # cv2_image = cv2.imread('acid_2.jpg')
            cv2_image = cv2.imread('../../images/acid_2.jpg')

        if args.resize_width:
            rospy.loginfo('{:.2f}s: Resizing image.'.format(time.time() - start_time))
            cv2_image = imutils.resize(cv2_image, width=self.resize_width)

        targets_detected = {}
        for target, cascade in self.targets.iteritems():
            rospy.loginfo('{:.2f}s: Running algorithm for target {}'.format(time.time() - start_time, target))
            targets_detected[target] = cascade.detectMultiScale(cv2_image)

        rospy.loginfo('{:.2f}s: Highlighting targets in image.'.format(time.time() - start_time))
        for target, results in targets_detected.iteritems():
            for (x, y, w, h) in results:
                cv2_image_highlighted = cv2.rectangle(cv2_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        rospy.loginfo('{:.2f}s: Publishing highlighted image.'.format(time.time() - start_time))
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv2_image_highlighted, "bgr8")
            self.publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)

        rospy.loginfo('{:.2f}s: Finished.'.format(time.time() - start_time))


def main(args):
    """Initialize and cleanup ros node."""
    image_processor = Image_Processor(args.cascades, args.resize_width)
    rospy.init_node('image_processor', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image processor module."
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-c', '--cascades', nargs='*', required=True)
    parser.add_argument('-r', '--resize-width', type=int)

    args = parser.parse_args(sys.argv[1:])
    main(args)
