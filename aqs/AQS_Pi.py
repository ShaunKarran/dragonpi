import rospy
import serial
import time
from std_msgs.msg import String

def AQS_Pi():
    pub = rospy.Publisher('AQS', String, queue_size=10)
    rospy.init_node('AQS_Pi', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    #Need to find serial port name on Pi
    arduino = serial.Serial('/dev/ttyACM0', 9600)

    while not rospy.is_shutdown():

        arduino.reset_input_buffer()
        while True:
            response = ''
            response += arduino.read()
            if "z" in response:
                break

        AQS_str = ""
        for _ in range (3):
            AQS_str += arduino.readline()
            AQS_str = AQS_str[:-2]
            AQS_str += " "
            time.sleep(0.05)


        rospy.loginfo(AQS_str)
        pub.publish(AQS_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        AQS_Pi()
    except rospy.ROSInterruptException:
        pass
