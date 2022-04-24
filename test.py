import unittest
import asyncio
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

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
        rospy.init_node('test_nav_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.result = False

    async def send_and_wait(self):
        pub = rospy.Publisher('/move_base_simple/goal',
                              PoseStamped, queue_size=10)
        rospy.Subscriber('/move_base/result',
                               MoveBaseActionResult, self.callback)
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = -1
        pose.pose.position.y = -1
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pub.publish(pose)

    async def wait_for_result(self):
        while not self.result and not rospy.is_shutdown():
            self.rate.sleep()

    def callback(self, data):
        self.result = True


async def main():
    print('hello')
    tester = RoverMoveBaseAction()
    print('hello3')
    await tester.send_and_wait()
    print("acutally publihse!")
    await tester.wait_for_result()
    print('world')

asyncio.run(main())
