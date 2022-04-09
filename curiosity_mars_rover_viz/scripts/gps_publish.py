#!/usr/bin/env python3
import rospy

import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
from tf.transformations import *

def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = turtlename

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.z
    t.transform.translation.z = -msg.pose.pose.position.y
    q_orig =[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    q_rot = quaternion_from_euler(-1.57, 0, 0)
    q_new = quaternion_multiply(q_rot, q_orig)
    t.transform.rotation.x = q_new[0]
    t.transform.rotation.y = q_new[1]
    t.transform.rotation.z = q_new[2]
    t.transform.rotation.w = q_new[3]
    
    #if t.header.stamp > lastStamp:
    br.sendTransform(t)
        #lastStamp = t.header.stamp

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = 'base_link'
    rospy.Subscriber('/curiosity_mars_rover/odom',
                     nav_msgs.msg.Odometry,
                     handle_turtle_pose,
                     turtlename,)
    rospy.spin()
