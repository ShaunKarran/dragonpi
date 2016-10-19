import argparse
import os
import sys

import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class ImageDisplayer:
    """Class to convert images from the 'images' topic to openCV format and write them to file."""

    def __init__(self, topic_name):
        """Initialise member variables and create ros subscriber."""
        self.topic_name = topic_name
        # self.bridge = CvBridge()

        # self.subscriber = rospy.Subscriber(topic_name, Image, self.callback, queue_size=1)
        self.subscriber = rospy.Subscriber(topic_name, CompressedImage, self.callback, queue_size=1)

    def callback(self, ros_data):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Converts the image to openCV format using cv_brige and writes to file.
        """
        rospy.loginfo('Converting from ros image to opencv image.')
        try:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # image_name = "{}_{}.{}".format(self.file_name, str(self.number_written), self.format)
        # rospy.loginfo('Writing {} to {}.'.format(image_name, self.output_path))
        # cv2.imwrite(os.path.join(self.output_path, image_name), cv_image)
        cv2.destroyAllWindows()
        cv2.imshow(self.topic_name, cv_image)
        # self.number_written += 1
        # rospy.loginfo('Image successfully written.')


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
    # parser.add_argument('-n', '--file-name', default='image')
    # parser.add_argument('-f', '--format', default='jpg')
    # parser.add_argument('-o', '--output-path', default='./')

    args = parser.parse_args(sys.argv[1:])
    main(args)
