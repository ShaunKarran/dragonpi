import rospy
from std_msgs.msg import Empty


# def take_image():
#     rospy.loginfo("Taking image.")
#     rospy.wait_for_service('take_image')
#     try:
#         take_image = rospy.ServiceProxy('take_image', Empty)
#         take_image()
#     except rospy.ServiceException, e:
#         print "Service call failed: {}".format(e)
#
#
# if __name__ == '__main__':
#     take_image()


def take_image():
    rospy.loginfo("Taking image.")
    publisher = rospy.Publisher('take_image', Empty, queue_size=1)
    rospy.init_node('image_taker', anonymous=False)
    publisher.publish()

if __name__ == '__main__':
    try:
        take_image()
    except rospy.ROSInterruptException:
        pass
