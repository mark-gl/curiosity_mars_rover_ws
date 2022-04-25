import unittest
import asyncio
import random
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
        self.final_x = 1000
        self.final_y = 1000
        self.final_rot = 10

    def send_goal(self, x, y, rot):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        quaternion = quaternion_from_euler(0, 0, rot)
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.pub.publish(pose)

    async def wait_for_result(self):
        while not self.result and not rospy.is_shutdown():
            self.rate.sleep()
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
        self.assertTrue(random_x - 0.5 <= self.tester.final_pos[0] <=
                        random_x + 0.5, "Rover X position ({0}) is > 0.5m from goal ({1})".format(random_x, self.tester.final_pos[0]))
        self.assertTrue(random_y - 0.5 <= self.tester.final_pos[1] <=
                        random_y + 0.5, "Rover Y position ({0}) is > 0.5m from goal ({1})".format(random_y, self.tester.final_pos[1]))
        self.assertTrue(random_rot - 0.5 <= self.tester.final_rot[2] <=
                        random_rot + 0.5, "Rover rotation ({0}) is > 0.5 radians from goal ({1})".format(random_rot, self.tester.final_rot[2]))

    @async_test
    async def test_move_home(self):
        self.tester.send_goal(0, 0, 0)
        await self.tester.wait_for_result()
        self.assertTrue(-0.5 <= self.tester.final_pos[0] <=
                        0.5, "Rover X position ({0}) is > 0.5m from goal (0.0)".format(self.tester.final_pos[0]))
        self.assertTrue(-0.5 <= self.tester.final_pos[1] <=
                        0.5, "Rover Y position ({0}) is > 0.5m from goal (0.0)".format(self.tester.final_pos[1]))
        self.assertTrue(-0.5 <= self.tester.final_rot[2] <=
                        0.5, "Rover rotation ({0}) is > 0.5 radians from goal (0.0)".format(self.tester.final_rot[2]))


if __name__ == '__main__':
    unittest.main()
