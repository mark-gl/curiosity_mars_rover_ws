#! /usr/bin/env python3

import rospy
import actionlib
from curiosity_mars_rover_control.msg import PanoramaAction, PanoramaFeedback, PanoramaResult
from curiosity_mars_rover_control.srv import Mast, MastRequest

class PanoramaActionServer():
    # create messages that are used to publish feedback/result
    _feedback = PanoramaFeedback()
    _result = PanoramaResult()

    def __init__(self, name):
        self._action_name = name
        self._ms = rospy.ServiceProxy('/curiosity_mars_rover/mast_service', Mast)
        self._as = actionlib.SimpleActionServer(self._action_name, PanoramaAction, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo("Ready!")
        self._as.start()

      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.percent_complete = 0
        
        # publish info to the console for the user
        rospy.loginfo(goal.destination_file)
        
        mast = MastRequest()
        # Now send the request through the connection
        mast.mode = "set"
        mast.pos_mast_p = 0.0
        mast.pos_mast_02 = -0.5
        mast.pos_mast_cameras = 0.0
        result = self._ms(mast)
        if result.status_message == "Fail! Mast Mode: Lowered":
            rospy.loginfo('%s: Mast not raised, cancelling.' % self._action_name)
            self._as.set_preempted()
            success = False
        else:
            # start executing the action
            for i in range(1, 5):
                mast = MastRequest()
                # Now send the request through the connection
                mast.mode = "set"
                result = self._ms(mast)
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Cancelled by request.' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.percent_complete = 10
                rospy.loginfo("Test")
                # publish the feedback
                self._as.publish_feedback(self._feedback)
                # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
                r.sleep()
          
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('panorama_server_node')
    rospy.loginfo("Waiting for mast service...")
    rospy.wait_for_service('/curiosity_mars_rover/mast_service')
    server = PanoramaActionServer(rospy.get_name())
    rospy.spin()
