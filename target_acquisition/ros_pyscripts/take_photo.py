import rospy
from std_srvs.srv import Empty


def take_photo():
    rospy.loginfo("Taking photo.")
    rospy.wait_for_service('take_photo')
    try:
        take_photo = rospy.ServiceProxy('take_photo', Empty)
        take_photo()
    except rospy.ServiceException, e:
        print "Service call failed: {}".format(e)


if __name__ == '__main__':
    take_photo()
