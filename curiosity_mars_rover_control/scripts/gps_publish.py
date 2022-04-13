#!/usr/bin/env python3
import threading
import rospy

import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
from tf.transformations import *

gps = None
br = tf2_ros.TransformBroadcaster()


def get_fake_gps(msg):
    global gps
    gps = msg.pose.pose


def publish_fake_gps():
    rate = rospy.Rate(60)  # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        global gps
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        if gps:
            t.transform.translation.x = gps.position.x
            t.transform.translation.y = gps.position.y
            t.transform.translation.z = gps.position.z

            # Three.js transformations for curiosity_mars_rover_viz
            # Commented out as the conversion is done on the other end

            # t.transform.translation.x = msg.pose.pose.position.x
            # t.transform.translation.y = msg.pose.pose.position.z
            # t.transform.translation.z = -msg.pose.pose.position.y

            q_orig = [gps.orientation.x, gps.orientation.y,
                      gps.orientation.z, gps.orientation.w]

            # q_rot = quaternion_from_euler(0, 0, 0)
            # q_rot = quaternion_from_euler(-1.57, 0, 0)
            # q_new = quaternion_multiply(q_rot, q_orig)

            t.transform.rotation.x = q_orig[0]
            t.transform.rotation.y = q_orig[1]
            t.transform.rotation.z = q_orig[2]
            t.transform.rotation.w = q_orig[3]

            br.sendTransform(t)
        else:
            rospy.loginfo("Waiting for odometry data...")
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('curiosity_mars_rover_fake_gps_node')

    rospy.Subscriber('/curiosity_mars_rover/odom',
                     nav_msgs.msg.Odometry,
                     get_fake_gps)
    worker = threading.Thread(target=publish_fake_gps)
    worker.start()

    rospy.spin()
