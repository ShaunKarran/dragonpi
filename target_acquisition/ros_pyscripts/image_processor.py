import argparse
import sys

import cv2
import numpy as np

import imutils
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class ImageProcessor:
    """
    Class containing a ros publisher and subscirber.

    Subscriber listens to the images topic which triggers the callback function. The callback publishes
    the results of the processing and an image with found targets to a highlighted_images topic.
    """

    def __init__(self, classifiers, resize_width):
        """Initialise ros publisher and subscriber and read the classifiers."""
        self.resize_width = resize_width
        self.cascades = {}
        self.classifier_publishers = {}
        self.target_seach_complete = {}

        for classifier_name in classifiers:
            trimmed_name = classifier_name[:-4]
            self.cascades[trimmed_name] = cv2.CascadeClassifier(classifier_name)
            self.classifier_publishers[trimmed_name] = rospy.Publisher(trimmed_name, Bool, queue_size=1)
            self.target_seach_complete[trimmed_name] = False

        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber('images', CompressedImage, self.callback, queue_size=1)
        self.image_publisher = rospy.Publisher('processed_images', CompressedImage, queue_size=1)
        self.complete_publisher = rospy.Publisher('processing_complete', Bool, queue_size=1)

    def callback(self, ros_data):
        """
        Callback function that is called when a message is recieved by the subscriber.

        Converts the image to an openCV image, runs the detection algorithm for each classifer on the image
        and then publishes the results and a new image with bounding boxes drawn around the targets.
        """
        rospy.loginfo("Converting from ros image to opencv image.")
        try:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        if args.resize_width:
            rospy.loginfo("Resizing image to width: {}px".format(self.resize_width))
            cv_image = imutils.resize(cv_image, width=self.resize_width)

        targets_detected = {}
        for classifier_name, cascade in self.cascades.iteritems():
            if self.target_seach_complete[classifier_name]:
                rospy.loginfo("Running detection for {}".format(classifier_name))
                targets_detected[classifier_name] = cascade.detectMultiScale(cv_image)
            else:
                rospy.loginfo("Skipping detection for {}. Already found.".format(classifier_name))

        # Draw bounding box around targets in image.
        for classifier_name, results in targets_detected.iteritems():
            for (x, y, w, h) in results:
                cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img=cv_image,
                            text=classifier_name,
                            org=(x, y),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 0, 255),
                            thickness=2)

        rospy.loginfo("Processing complete.")
        self.complete_publisher.publish(False)

        rospy.loginfo("Publishing results.")
        for classifier_name, publisher in self.classifier_publishers.iteritems():
            if len(targets_detected[classifier_name]) > 0:
                publisher.publish(True)
            else:
                publisher.publish(False)

        rospy.loginfo("Publishing highlighted image.")
        try:
            ros_image = CompressedImage()
            ros_image.header.stamp = rospy.Time.now()
            ros_image.format = 'jpeg'
            ros_image.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            # ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)


def main(args):
    """Initialize and cleanup ros node."""
    image_processor = ImageProcessor(args.classifiers, args.resize_width)
    rospy.init_node('image_processor', anonymous=False)
    try:
        rospy.loginfo("Image Processor node running.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image processor module."
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-c', '--classifiers', nargs='*', required=True)
    parser.add_argument('-r', '--resize-width', type=int, required=True)

    args = parser.parse_args(sys.argv[1:])
    main(args)
