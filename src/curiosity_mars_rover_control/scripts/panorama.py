#! /usr/bin/env python3
import rospy
import actionlib
from curiosity_mars_rover_control.msg import PanoramaAction, PanoramaFeedback, PanoramaResult
from curiosity_mars_rover_control.srv import Mast, MastRequest
from std_srvs.srv import Empty, EmptyRequest


class PanoramaActionServer():
    _feedback = PanoramaFeedback()
    _result = PanoramaResult()

    def __init__(self, name):
        self._action_name = name
        self._ms = rospy.ServiceProxy(
            '/curiosity_mars_rover/mast_service', Mast)
        self._ps = rospy.ServiceProxy(
            '/hugin_panorama/image_saver/save', Empty)
        self._rs = rospy.ServiceProxy('/hugin_panorama/reset', Empty)
        self._ss = rospy.ServiceProxy('/hugin_panorama/stitch', Empty)
        self._as = actionlib.SimpleActionServer(
            self._action_name, PanoramaAction, execute_cb=self.execute_cb, auto_start=False)
        rospy.loginfo("Ready!")
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo('New goal recieved.')
        reset = EmptyRequest()
        result = self._rs(reset)
        r = rospy.Rate(1)

        success = True
        self._feedback.state = "0%"
        orientation = 1.07

        mast = MastRequest()
        mast.mode = "set"
        mast.pos_mast_02 = orientation
        result = self._ms(mast)

        if result.status_message == "Fail! Mast Mode: Lowered":
            rospy.loginfo('%s: Mast not raised, cancelling.' %
                          self._action_name)
            self._as.set_preempted()
            success = False
        else:
            # Wait for the mast to move into place
            rospy.sleep(5)
            for i in range(0, 31):
                photo = EmptyRequest()
                result = self._ps(photo)
                orientation -= 0.1
                mast = MastRequest()
                mast.mode = "set"
                mast.pos_mast_02 = orientation
                result = self._ms(mast)
                if result.status_message == "Fail! Mast Mode: Lowered":
                    rospy.loginfo('%s: Mast not raised, cancelling.' %
                                  self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Cancelled by request.' %
                                  self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.state = "{:.2f}%".format(i/30 * 100)
                rospy.loginfo(self._feedback.state)
                self._as.publish_feedback(self._feedback)
                r.sleep()
            # Reset mast position to 'raised'
            mast = MastRequest()
            mast.mode = "set"
            mast.pos_mast_02 = -0.5
            result = self._ms(mast)
        if success:
            self._feedback.state = "Stitching"
            self._result.success = True
            self._as.set_succeeded(self._result)
            self._as.publish_feedback(self._feedback)
            rospy.loginfo(self._feedback.state)
            stitch = EmptyRequest()
            result = self._ss(stitch)
            self._feedback.state = "Stitched!"
            rospy.loginfo(self._feedback.state)
            self._as.publish_feedback(self._feedback)


if __name__ == '__main__':
    rospy.init_node('curiosity_mars_rover_panorama_server_node')
    rospy.loginfo("Waiting for mast service...")
    rospy.wait_for_service('/curiosity_mars_rover/mast_service')
    rospy.loginfo("Waiting for Hugin...")
    rospy.wait_for_service('/hugin_panorama/image_saver/save')
    rospy.wait_for_service('/hugin_panorama/reset')
    rospy.wait_for_service('/hugin_panorama/stitch')
    server = PanoramaActionServer(rospy.get_name())
    rospy.spin()
