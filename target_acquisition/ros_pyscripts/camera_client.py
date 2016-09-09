import rospy


# from beginner_tutorials.srv import *


def camera_client():
    print "Taking Photo."
    rospy.wait_for_service('take_photo')
    try:
        take_photo = rospy.ServiceProxy('take_photo', None)
        take_photo()
    except rospy.ServiceException, e:
        print "Service call failed: {}".format(e)


if __name__ == '__main__':
    camera_client()
