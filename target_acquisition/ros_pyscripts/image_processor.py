#!/usr/bin/env python

import argparse
import sys

import cv2
import imutils
import rospy
from cv_bridge import CvBridge, CvBridgeError
# from my_msgs import ProcessorResults
from sensor_msgs.msg import Image


class ImageProcessor:
    """
    Class containing a ros publisher and subscirber.

    Subscriber listens to the images topic which triggers the callback function. It then publishes the results of the
    processing and an image with found targets to a highlighted_images topic.
    """

    def __init__(self, classifiers, resize_width):
        """Initialise ros publisher and subscriber and read the classifiers."""
        self.resize_width = resize_width
        self.cascades = {}
        for classifer in classifiers:
            rospy.loginfo("Loading {}".format(classifer))
            self.cascades[classifer] = cv2.CascadeClassifier(classifer)

        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber('images', Image, self.callback, queue_size=5)
        self.image_publisher = rospy.Publisher('images', Image, queue_size=5)
        # self.results_publisher = rospy.Publisher('processor_results', ProcessorResults, queue_size=5)

    def callback(self, ros_data):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Converts the image to an openCV image, runs the detection algorithm for each classifer on the image and then
        publishes the results and a new image with bounding boxes drawn around the targets.
        """
        rospy.loginfo("Converting from ros image to opencv image.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        if args.resize_width:
            rospy.loginfo("Resizing image to width {}".format(self.resize_width))
            cv_image = imutils.resize(cv_image, width=self.resize_width)

        targets_detected = {}
        for cascade_name, cascade in self.cascades.iteritems():
            rospy.loginfo("Running algorithm for target {}".format(cascade_name))
            targets_detected[cascade_name] = cascade.detectMultiScale(cv_image)

        # Draw bounding box around targets in image.
        for cascade_name, results in targets_detected.iteritems():
            for (x, y, w, h) in results:
                cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img=cv_image,
                            text=cascade_name,
                            org=(x, y),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 0, 255),
                            thickness=2)

        # rospy.loginfo("Publishing processor results.")
        # self.results_publisher.publish(results)

        rospy.loginfo("Publishing highlighted image.")
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)


def main(args):
    """Initialize and cleanup ros node."""
    image_processor = ImageProcessor(args.classifiers, args.resize_width)
    rospy.init_node('image_processor', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image processor module."
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-c', '--classifiers', nargs='*', required=True)
    parser.add_argument('-r', '--resize-width', type=int)

    args = parser.parse_args(sys.argv[1:])
    main(args)
