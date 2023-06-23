#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('goal_publisher', anonymous=True)
    rate = rospy.Rate(0.33) # publish every 3 seconds
    while not rospy.is_shutdown():
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = -5
        goal.pose.position.y = -4
        goal.pose.position.z = 0

        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0.13
        goal.pose.orientation.w = 1
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
