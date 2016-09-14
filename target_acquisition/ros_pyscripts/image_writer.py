#!/usr/bin/env python

# Python libs
import argparse
import os
import sys
import time

# OpenCV
import cv2
# Ros libs
import rospy
from cv_bridge import CvBridge, CvBridgeError
# Ros messages
from sensor_msgs.msg import Image


class ImageWriter:
    """Class to convert images from the 'images' topic to openCV format and write them to file."""

    def __init__(self, topic_name, file_name, format, output_path):
        """Initialise member variables and create ros subscriber."""
        self.file_name = file_name
        self.format = '.' + format
        self.output_path = output_path
        self.number_written = 0

        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber(topic_name, Image, self.callback, queue_size=5)

    def callback(self, ros_data):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Converts the image to openCV format using cv_brige and writes to file.
        """
        start_time = time.time()

        rospy.loginfo('{:.2f}s: Converting from ros image to opencv image.'.format(time.time() - start_time))
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        image_name = os.path.join(self.output_path,
                                  ("{}_{}.{}".format(self.file_name, str(self.number_written), self.format)))
        rospy.loginfo('{:.2f}s: Writing image {}.'.format(time.time() - start_time, image_name))
        cv2.imwrite(image_name, cv_image)
        self.number_written += 1

        rospy.loginfo('{:.2f}s: Finished.'.format(time.time() - start_time))


def main(args):
    """Initialize and cleanup ros node."""
    image_writer = ImageWriter(args.topic_name, args.file_name, args.format, args.output_path)
    rospy.init_node('image_writer', anonymous=True)
    try:
        rospy.loginfo("Image Writer node running.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image processor module."
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-t', '--topic-name', default='images')
    parser.add_argument('-n', '--file-name', default='image')
    parser.add_argument('-f', '--format', default='jpg')
    parser.add_argument('-o', '--output-path', default='./')

    args = parser.parse_args(sys.argv[1:])
    main(args)
