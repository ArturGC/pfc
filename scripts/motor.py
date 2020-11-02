#! /usr/bin/env python3
import os
import rospy
import time
from std_msgs.msg import Float32


if __name__ == "__main__":
    rospy.init_node('motorPub', anonymous=True)
    motorPub = rospy.Publisher('motor', Float32, queue_size=10)
    dt = 2
    rate = rospy.Rate(1/dt)
    sinal = rospy.get_param('/sinalMotor')

    count = 0
    while (count < len(sinal)) or (not rospy.is_shutdown()):
        motorPub.publish(sinal[count])
        rospy.loginfo(sinal[count])

        count += 1
        rate.sleep()

    rospy.spin()