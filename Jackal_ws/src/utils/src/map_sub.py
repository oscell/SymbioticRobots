#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

def callback(data):
    rospy.loginfo("Received map data.")
    rospy.loginfo("Map data type: %s", type(data))
    # rospy.loginfo("Map data: %s", data)

def listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
