import argparse
import sys

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage


class ImageDisplayer:
    """Class to convert images from the 'images' topic to openCV format and write them to file."""

    def __init__(self, topic_name):
        """Initialise member variables and create ros subscriber."""
        self.topic_name = topic_name
        self.subscriber = rospy.Subscriber(topic_name, CompressedImage, self.callback, queue_size=1)

    def callback(self, ros_data):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Converts the image to openCV format using cv_brige and writes to file.
        """
        rospy.loginfo('Converting from ros image to opencv image.')
        np_arr = np.fromstring(ros_data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # cv2.destroyAllWindows()
        cv2.imshow(self.topic_name, cv_image)


def main(args):
    """Initialize and cleanup ros node."""
    image_writer = ImageDisplayer(args.topic_name)
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

    args = parser.parse_args(sys.argv[1:])
    main(args)
