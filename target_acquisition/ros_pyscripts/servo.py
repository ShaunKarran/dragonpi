import rospy
from servosix import ServoSix
from std_msgs.msg import Bool


class Servo:
    """
    """

    def __init__(self):
        """Initialise ros publisher and subscriber."""
        self.ss = ServoSix()
        self.vertical_servo = 2  # Servo channel for vertical pivot.

        # Set initial position.
        self.ss.set_servo(1, 88)  # Horizontal pivot is slightly off, 88 points straight ahead.
        self.ss.set_servo(2, 0)

        self.subscriber = rospy.Subscriber('position_servo', Bool, self.callback, queue_size=1)

    def callback(self, ros_data):
        """
        """
        if (ros_data.data):
            angle = 95  # 95 points straight ahead.
        else:
            angle = 0

        self.ss.set_servo(self.vertical_servo, angle)


def main():
    """Initialize and cleanup ros node."""
    servo = Servo()
    rospy.init_node('servo', anonymous=False)
    try:
        rospy.loginfo("Servo node running.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Servo module."
        servo.ss.cleanup()


if __name__ == '__main__':
    main()
