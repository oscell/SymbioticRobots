#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from unity_robotics_demo_msgs.msg import PosRot

# Initialize the publisher outside of callback
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

def callback(data):
    goal = PoseStamped()
    goal.header.frame_id = "map"  # adjust this if needed
    goal.pose.position.x = data.pos_x
    goal.pose.position.y = data.pos_y
    goal.pose.position.z = data.pos_z
    goal.pose.orientation.x = data.rot_x
    goal.pose.orientation.y = data.rot_y
    goal.pose.orientation.z = data.rot_z
    goal.pose.orientation.w = data.rot_w

    rospy.loginfo("Received Message: %s", data)
    print("Received Message: ", data)
    rospy.loginfo("Publishing Received Message")
    pub.publish(goal)

def listener():
    rospy.init_node('goal_publisher', anonymous=True)
    rospy.Subscriber("/pos_rot", PosRot, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
