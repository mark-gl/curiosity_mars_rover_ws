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


class RoverMoveBaseAction:
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

    def send_and_wait(self, x, y, rot):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        self.pub.publish(pose)

    async def wait_for_result(self):
        while not self.result and not rospy.is_shutdown():
            self.rate.sleep()
        return 

    def callback_result(self, data):
        self.result = True
        trans, rot = self.listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        print(trans)
        print(rot)


async def main():
    tester = RoverMoveBaseAction()
    random_x = round(random.uniform(-10, 10),2)
    random_y = round(random.uniform(-10, 10),2)
    random_rot = quaternion_from_euler(0, 0, round(random.uniform(-3.14, 3.14),2))
    tester.send_and_wait(random_x,random_y,random_rot)
    await tester.wait_for_result()


asyncio.run(main())
