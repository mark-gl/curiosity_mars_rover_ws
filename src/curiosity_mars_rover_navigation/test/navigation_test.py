#!/usr/bin/env python3

# Navigation tester that sends random goals, waits for a result message and
# compares the final rover position with the goal position originally sent.

import asyncio
import random
import unittest
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import *

# Decorator for asynchronous testing, borrowed from:
# https://stackoverflow.com/questions/23033939/how-to-test-python-3-4-asyncio-code


def async_test(coro):
    def wrapper(*args, **kwargs):
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro(*args, **kwargs))
        finally:
            loop.close()
    return wrapper


class NavigationPublishAndWait:
    def __init__(self):
        self.pub = rospy.Publisher('/move_base_simple/goal',
                                   PoseStamped, queue_size=10)
        rospy.Subscriber('/move_base/result',
                         MoveBaseActionResult, self.callback_result)
        rospy.init_node('test_nav_node', anonymous=True)
        rospy.sleep(1)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)
        self.result = False
        # Initialise recorded positions as unlikely results (1km from origin)
        self.final_x = 1000
        self.final_y = 1000
        self.final_rot = 10

    def send_goal(self, x, y, rot):
        # Send a simple navigation goal to the action server
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        # Convert random rotation in radians to quaternion
        quaternion = quaternion_from_euler(0, 0, rot)
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.pub.publish(pose)

    async def wait_for_result(self):
        # Wait for callback_result to be called
        while not self.result and not rospy.is_shutdown():
            self.rate.sleep()
        # Set the final recorded position of the rover after action server sends result
        self.final_pos, self.final_rot = self.listener.lookupTransform(
            'odom', 'base_link', rospy.Time(0))
        self.final_rot = euler_from_quaternion(self.final_rot)
        return

    def callback_result(self, data):
        self.result = True


class NavigationUnitTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(NavigationUnitTest, self).__init__(*args, **kwargs)
        self.tester = NavigationPublishAndWait()

    @async_test
    async def test_move_random(self):
        random_x = round(random.uniform(-10, 10), 2)
        random_y = round(random.uniform(-10, 10), 2)
        random_rot = round(random.uniform(-3.14, 3.14), 2)
        self.tester.send_goal(random_x, random_y, random_rot)
        await self.tester.wait_for_result()
        # Ensure that the final positions are within 0.5 of the random values generated
        # 0.5 is used as it is the goal tolerance specified in base_local_params.yaml
        self.assertTrue(random_x - 0.5 <= self.tester.final_pos[0] <=
                        random_x + 0.5, "Rover X position ({0}) is > 0.5m from goal ({1})".format(self.tester.final_pos[0], random_x))
        self.assertTrue(random_y - 0.5 <= self.tester.final_pos[1] <=
                        random_y + 0.5, "Rover Y position ({0}) is > 0.5m from goal ({1})".format(self.tester.final_pos[1], random_y))
        self.assertTrue(abs(random_rot) - 0.8 <= abs(self.tester.final_rot[2]) <=
                        abs(random_rot) + 0.8, "Rover rotation ({0}) is > 0.8 radians from goal ({1})".format(abs(self.tester.final_rot[2]), abs(random_rot)))

    @async_test
    async def test_move_home(self):
        self.tester.send_goal(0, 0, 0)
        await self.tester.wait_for_result()
        self.assertTrue(-0.5 <= self.tester.final_pos[0] <=
                        0.5, "Rover X position ({0}) is > 0.5m from goal (0.0)".format(self.tester.final_pos[0]))
        self.assertTrue(-0.5 <= self.tester.final_pos[1] <=
                        0.5, "Rover Y position ({0}) is > 0.5m from goal (0.0)".format(self.tester.final_pos[1]))
        self.assertTrue(-0.8 <= self.tester.final_rot[2] <=
                        0.8, "Rover rotation ({0}) is > 0.8 radians from goal (0.0)".format(self.tester.final_rot[2]))


if __name__ == '__main__':
    unittest.main()
