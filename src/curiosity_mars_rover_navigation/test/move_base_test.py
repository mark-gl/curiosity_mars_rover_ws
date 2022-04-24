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
    async def __init__(self):
        self.result = False

        rospy.init_node('test_nav_node', anonymous=True)
        self.pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber(
            '/move_base/result', MoveBaseActionResult, self.callback)

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

        self.pub.publish(pose)
        await self.wait_for_result()
        return 

    def callback(self, data):
        self.result = True

    async def wait_for_result(self):
        while not self.result:
            pass
        return


class RoverMoveBaseTest(unittest.TestCase):

    def test_move(self):

        await
        self.assertEqual('foo'.upper(), 'FOO')


if __name__ == '__main__':
    unittest.main()
