import rospy
from std_msgs.msg import Empty


# def take_photo():
#     rospy.loginfo("Taking photo.")
#     rospy.wait_for_service('take_photo')
#     try:
#         take_photo = rospy.ServiceProxy('take_photo', Empty)
#         take_photo()
#     except rospy.ServiceException, e:
#         print "Service call failed: {}".format(e)
#
#
# if __name__ == '__main__':
#     take_photo()


def take_photo():
    rospy.loginfo("Taking photo.")
    publisher = rospy.Publisher('take_photo', Empty, queue_size=1)
    rospy.init_node('photo_taker', anonymous=False)
    publisher.publish()

if __name__ == '__main__':
    try:
        take_photo()
    except rospy.ROSInterruptException:
        pass
