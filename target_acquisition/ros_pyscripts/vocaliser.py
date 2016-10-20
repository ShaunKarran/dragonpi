import pyttsx
import rospy
from std_msgs.msg import Bool, Empty


class Vocaliser:

    def __init__(self):
        self.engine = pyttsx.init()
        # Callback to fire once when node start. Used to start pyttsx engine looping.
        self.sub_once = rospy.Subscriber('start_vocaliser', Empty, self.start_loop)

        self.acid_subscriber = rospy.Subscriber('classifiers/acid', Bool, self.acid_callback, queue_size=1)
        self.flammable_subscriber = rospy.Subscriber('classifiers/flammable', Bool,
                                                     self.flammable_callback, queue_size=1)
        self.misc_subscriber = rospy.Subscriber('classifiers/misc', Bool, self.misc_callback, queue_size=1)

    def acid_callback(self, ros_data):
        if ros_data.data:
            self.engine.say("Yellow target found. Hovering for 10 seconds.")

    def flammable_callback(self, ros_data):
        if ros_data.data:
            self.engine.say("Red target found. Hovering for 10 seconds.")

    def misc_callback(self, ros_data):
        if ros_data.data:
            self.engine.say("Black target found. Hovering for 10 seconds.")

    def start_loop(self, ros_data):
        self.engine.startLoop()
        self.sub_once.unregister()


def main():
    """Initialize and cleanup ros node."""
    vocaliser = Vocaliser()
    rospy.init_node('vocaliser', anonymous=False)
    try:
        rospy.loginfo("Vocaliser node running.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Vocaliser module."


if __name__ == '__main__':
    main()
