#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class Twist_pub:
    def __init__(self):
        self.msg=Twist()
        self.pub= rospy.Publisher("cmd_vel",Twist,queue_size=10)

    def send_vel(self,x ,z):
        self.msg.linear.x = x
        self.msg.angular.z = z
        self.pub.publish(self.msg)
        return 'velocity published'

if __name__ == '__main__':
    rospy.init_node("cmd_publisher",anonymous=True)
    Twist_pub().send_vel(0.0, 0.0)