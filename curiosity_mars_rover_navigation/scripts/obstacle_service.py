#!/usr/bin/env python3
import re
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from curiosity_mars_rover_navigation.srv import Teleop, TeleopResponse

class ObstacleService(object):
    def __init__(self):
        self.cmd_vel_service = rospy.Service("/curiosity_mars_rover/cmd_vel_obstacle", Teleop, self.teleop_obstacle)
        self.publisher = rospy.Publisher('/curiosity_mars_rover/ackermann_drive_controller/cmd_vel', Twist, queue_size = 1)
        # Same topics are used for fake depth cameras and normal stereo cameras
        self.front = rospy.Subscriber("/curiosity_mars_rover/camera_fronthazcam/scan", LaserScan, self.frontScan)
        self.back = rospy.Subscriber("/curiosity_mars_rover/camera_backhazcam/scan", LaserScan, self.backScan)
        self.frontBlocked = False
        self.backBlocked = False
        self.wait_publishers_to_be_ready()

    def teleop_obstacle(self, req):
        response = TeleopResponse()
        response.feedback = ""
        if req.twist.linear.x > 0 and self.frontBlocked:
            response.feedback = "Obstacle in front"
        elif req.twist.linear.x < 0 and self.backBlocked: 
            response.feedback = "Obstacle in rear"
        else:
            # Safe to publish movement
            self.publisher.publish(req.twist)
        return response
    
    def wait_publishers_to_be_ready(self):
        rate_wait = rospy.Rate(2)
        publisher_ready = False
        while not publisher_ready:
            rospy.loginfo("Connecting to drive controller...")
            pub_num = self.publisher.get_num_connections()
            publisher_ready = (pub_num > 0)
            rate_wait.sleep()
        rospy.loginfo("Ready to send teleoperation messages!")
    
    def frontScan(self, msg):
        if any(t < 2.5 for t in msg.ranges):
            self.frontBlocked = True
        elif self.frontBlocked == True:
            self.frontBlocked = False

    def backScan(self, msg):
        if any(t < 2.5 for t in msg.ranges):
            self.backBlocked = True
        elif self.backBlocked == True:
            self.backBlocked = False
    
if __name__ == "__main__":
    rospy.init_node("CuriosityRoverObstacleNode", log_level=rospy.INFO)
    curiosity_mars_rover_suspension_object = ObstacleService()
    rospy.spin()
