#!/usr/bin/env python
import rospy
import os
import time
from PIL import Image



def map_saver():
    rospy.init_node('map_saver_node', anonymous=True)
    rate = rospy.Rate(1) # Set the rate to 1Hz
    while not rospy.is_shutdown():
        os.system('rosrun map_server map_saver -f mymap1')  # Runs the command
        # Open the .pgm file
        img = Image.open('mymap1.pgm')

        # Convert and save as .jpg
        img.save('mymap1.png')
        rate.sleep()  # Sleeps to maintain the rate

if __name__ == '__main__':
    try:
        map_saver()
    except rospy.ROSInterruptException:
        pass
