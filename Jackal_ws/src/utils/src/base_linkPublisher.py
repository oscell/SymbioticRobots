#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('base_link_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)  # 10Hz
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rospy.loginfo("Translation: %s" % str(trans))
            rospy.loginfo("Rotation: %s" % str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            continue

        rate.sleep()
