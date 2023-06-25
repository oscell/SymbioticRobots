#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf.transformations
import math
import threading

class OdomInfoNode:
    def __init__(self):
        rospy.init_node('odom_info_node', anonymous=True)
        rospy.Subscriber("/odometry/filtered", Odometry, self.callback)
        self.mutex = threading.Lock()
        self.latest_data = None
        self.should_exit = False

        t = threading.Thread(target=self.print_loop)
        t.start()

    def callback(self, data):
        with self.mutex:
            self.latest_data = data

    def print_loop(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown() and not self.should_exit:
            with self.mutex:
                data = self.latest_data
                self.latest_data = None

            if data is not None:
                self.process_data(data)
            
            rate.sleep()

    def process_data(self, data):
        position = data.pose.pose.position
        orientation_q = data.pose.pose.orientation

        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        # Convert rotation from radians to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        rospy.loginfo("Position: x=%f, y=%f, z=%f", position.x, position.y, position.z)
        rospy.loginfo("Euler Orientation (Degrees): roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw)

def main():
    node = OdomInfoNode()
    rospy.spin()
    node.should_exit = True

if __name__ == '__main__':
    main()
