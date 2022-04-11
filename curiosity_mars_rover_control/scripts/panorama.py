#! /usr/bin/env python3

import rospy
import actionlib
from curiosity_mars_rover_control.msg import PanoramaAction, PanoramaFeedback, PanoramaResult
from curiosity_mars_rover_control.srv import Mast, MastRequest
from std_srvs.srv import Empty, EmptyRequest

class PanoramaActionServer():
    # create messages that are used to publish feedback/result
    _feedback = PanoramaFeedback()
    _result = PanoramaResult()

    def __init__(self, name):
        self._action_name = name
        self._ms = rospy.ServiceProxy('/curiosity_mars_rover/mast_service', Mast)
        self._ps = rospy.ServiceProxy('/hugin_panorama/image_saver/save', Empty)
        self._rs = rospy.ServiceProxy('/hugin_panorama/reset', Empty)
        self._ss = rospy.ServiceProxy('/hugin_panorama/stitch', Empty)
        self._as = actionlib.SimpleActionServer(self._action_name, PanoramaAction, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo("Ready!")
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo('New goal recieved.')

        reset = EmptyRequest()
        result = self._rs(reset)
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.state = "Photographing 0%"
        
        orientation = 1.07
        mast = MastRequest()
        # Now send the request through the connection
        mast.mode = "set"
        mast.pos_mast_02 = orientation
        result = self._ms(mast)
        if result.status_message == "Fail! Mast Mode: Lowered":
            rospy.loginfo('%s: Mast not raised, cancelling.' % self._action_name)
            self._as.set_preempted()
            success = False
        else:
            # start executing the action
            rospy.sleep(5)
            for i in range(0, 31):
                photo = EmptyRequest()
                result = self._ps(photo)
                orientation -= 0.1
                mast = MastRequest()
                # Now send the request through the connection
                mast.mode = "set"
                mast.pos_mast_02 = orientation
                result = self._ms(mast)
                if result.status_message == "Fail! Mast Mode: Lowered":
                    rospy.loginfo('%s: Mast not raised, cancelling.' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Cancelled by request.' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.state = "Photographing {0}%".format(i/30 * 100)
                rospy.loginfo(self._feedback.state)
                self._as.publish_feedback(self._feedback)
                r.sleep()
            mast = MastRequest()
            # Reset
            mast.mode = "set"
            mast.pos_mast_02 = -0.5
            result = self._ms(mast)
        if success:
            rospy.loginfo('%s: Succeeded, now stitching.' % self._action_name)
            self._feedback.state = "Stitching"
            rospy.loginfo(self._feedback.state)
            self._as.publish_feedback(self._feedback)

            stitch = EmptyRequest()
            result = self._ss(stitch)
            self._feedback.state = "Finished!"
            rospy.loginfo(self._feedback.state)
            self._as.publish_feedback(self._feedback)

            self._result.success = True
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('panorama_server_node')
    rospy.loginfo("Waiting for mast service...")
    rospy.wait_for_service('/curiosity_mars_rover/mast_service')
    rospy.loginfo("Waiting for Hugin...")
    rospy.wait_for_service('/hugin_panorama/image_saver/save')
    rospy.wait_for_service('/hugin_panorama/reset')
    rospy.wait_for_service('/hugin_panorama/stitch')
    server = PanoramaActionServer(rospy.get_name())
    rospy.spin()
