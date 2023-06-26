#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

rospy.init_node('posestamps_plotter', anonymous=True)

def callback(data):
    # Extract the pose stamps
    pose_stamps = [pose.pose for pose in data.poses]

    # Split the pose stamps into their x and y components
    x = [pose.position.x for pose in pose_stamps]
    y = [pose.position.y for pose in pose_stamps]

    # Plot the pose stamps
    plt.figure()
    plt.plot(x, y, 'o-')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Pose Stamps')
    plt.show()

rospy.Subscriber("/move_base/NavfnROS/plan", Path, callback)

rospy.spin()
