#!/usr/bin/env python3
import rospy, tf, actionlib
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Navigation:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.pose_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.client_act = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.initpose_msg = PoseWithCovarianceStamped()
        self.goal = MoveBaseGoal()
        self.dmin = 0.2
        rospy.sleep(1)

    def tf_listener(self):
            try:
                (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                return trans[0], trans[1], rot[2], rot[3]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return 0.0, 0.0, 0.0, 0.0
    
    def estimate_pose(self, positions):
        self.initpose_msg.header.frame_id = "map"
        self.initpose_msg.pose.pose.position.x = positions[0]
        self.initpose_msg.pose.pose.position.y = positions[1]
        self.initpose_msg.pose.pose.orientation.z = positions[2]
        self.initpose_msg.pose.pose.orientation.w = positions[3]
        self.pose_publisher.publish(self.initpose_msg)
    
    def send_goal(self, position):
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]        
        self.goal.target_pose.pose.orientation.z = position[2]
        self.goal.target_pose.pose.orientation.w = position[3]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.client_act.send_goal(self.goal)
    
    def goal_status(self, goal_pose):
        d = math.sqrt((goal_pose[0] - self.tf_listener()[0])**2 + (goal_pose[1] - self.tf_listener()[1])**2)
        if d <= self.dmin:
            return True
        return False