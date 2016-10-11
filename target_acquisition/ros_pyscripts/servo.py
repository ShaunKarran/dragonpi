import rospy
from servosix import ServoSix
from std_msgs.msg import Bool


class Servo:
    """
    """

    def __init__(self):
        """Initialise ros publisher and subscriber."""
        self.ss = ServoSix()
        self.ss.servo_min = 1000  # 1ms pulse = -90
        self.ss.servo_max = 2000  # 2ms pulse = +90
        self.servo = 1  # Servo channel

        self.subscriber = rospy.Subscriber('position_servo', Bool, self.callback, queue_size=1)

    def callback(self, ros_data):
        """
        """
        if (ros_data.data):
            angle = 90
        else:
            angle = 0

        self.ss.set_servo(self.servo, angle)


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
