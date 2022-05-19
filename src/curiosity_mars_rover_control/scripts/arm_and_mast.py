#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from curiosity_mars_rover_control.srv import Mast, Arm, MastResponse, ArmResponse

# This code has been modified from the original Curiosity ROS simulation.
class CuriosityMarsRoverArmAndMast(object):
    def __init__(self):
        rospy.loginfo("Curiosity Arm And Mast Initialising...")
        self.arm_state = 'Closed'
        self.mast_state = 'Raised'
        self.publishers_curiosity_d = {}
        self.controller_ns = "curiosity_mars_rover"
        self.controller_command = "command"
        self.controllers_list = ["mast_p_joint_position_controller",
                                 "mast_02_joint_position_controller",
                                 "mast_cameras_joint_position_controller",
                                 "arm_01_joint_position_controller",
                                 "arm_02_joint_position_controller",
                                 "arm_03_joint_position_controller",
                                 "arm_04_joint_position_controller",
                                 "arm_tools_joint_position_controller"
                                 ]
        for controller_name in self.controllers_list:
            topic_name = "/"+self.controller_ns+"/" + \
                controller_name+"/"+self.controller_command
            self.publishers_curiosity_d[controller_name] = rospy.Publisher(
                topic_name,
                Float64,
                queue_size=1)

        self.wait_publishers_to_be_ready()
        self.init_publisher_variables()

        arm_service_name = "/"+self.controller_ns+"/arm_service"
        self.arm_service = rospy.Service(
            arm_service_name, Arm, self.arm_service_cb)
        self.arm_pub = rospy.Publisher(
            "/" + self.controller_ns + "/arm_state", String, queue_size=10)
        mast_service_name = "/" + self.controller_ns + "/mast_service"
        self.mast_service = rospy.Service(
            mast_service_name, Mast, self.mast_service_cb)
        self.mast_pub = rospy.Publisher(
            "/" + self.controller_ns + "/mast_state", String, queue_size=10)
        rospy.loginfo("Curiosity Arm And Mast...READY")

    def arm_service_cb(self, req):
        response = ArmResponse()
        response.success = self.set_arm_pose(req)
        if response.success:
            response.status_message = "Done! Arm Mode: " + self.arm_state
            self.arm_pub.publish(response.status_message)
        else:
            response.status_message = "Fail! Arm Mode: " + self.arm_state
            self.arm_pub.publish(response.status_message)
        return response

    def mast_service_cb(self, req):
        response = MastResponse()
        response.success = self.set_mast_pose(req)
        if response.success:
            response.status_message = "Done! Mast Mode: " + self.mast_state
            self.mast_pub.publish(response.status_message)
        else:
            response.status_message = "Fail! Mast Mode: " + self.mast_state
            self.mast_pub.publish(response.status_message)
        return response

    def wait_publishers_to_be_ready(self):
        rate_wait = rospy.Rate(60)
        for controller_name, publisher_obj in self.publishers_curiosity_d.items():
            publisher_ready = False
            while not publisher_ready:
                rospy.loginfo("Checking Publisher for ==>" +
                              str(controller_name))
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = (pub_num > 0)
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")

    def init_publisher_variables(self):
        # Get the publishers
        self.mast_p = self.publishers_curiosity_d[self.controllers_list[0]]
        self.mast_02 = self.publishers_curiosity_d[self.controllers_list[1]]
        self.mast_cameras = self.publishers_curiosity_d[self.controllers_list[2]]
        self.arm_01 = self.publishers_curiosity_d[self.controllers_list[3]]
        self.arm_02 = self.publishers_curiosity_d[self.controllers_list[4]]
        self.arm_03 = self.publishers_curiosity_d[self.controllers_list[5]]
        self.arm_04 = self.publishers_curiosity_d[self.controllers_list[6]]
        self.arm_tools = self.publishers_curiosity_d[self.controllers_list[7]]

        # Init messages
        self.mast_p_pos_msg = Float64()
        self.mast_02_pos_msg = Float64()
        self.mast_cameras_pos_msg = Float64()
        self.arm_01_pos_msg = Float64()
        self.arm_02_pos_msg = Float64()
        self.arm_03_pos_msg = Float64()
        self.arm_04_pos_msg = Float64()
        self.arm_tools_pos_msg = Float64()

        self.mast_p_pos_msg.data = 0.0
        self.mast_02_pos_msg.data = -0.5
        self.mast_cameras_pos_msg.data = 0.0
        self.arm_01_pos_msg.data = -1.57
        self.arm_02_pos_msg.data = -0.4
        self.arm_03_pos_msg.data = -1.1
        self.arm_04_pos_msg.data = -1.57
        self.arm_tools_pos_msg.data = -1.57

        self.mast_p.publish(self.mast_p_pos_msg)
        self.mast_02.publish(self.mast_02_pos_msg)
        self.mast_cameras.publish(self.mast_cameras_pos_msg)
        self.arm_01.publish(self.arm_01_pos_msg)
        self.arm_02.publish(self.arm_02_pos_msg)
        self.arm_03.publish(self.arm_03_pos_msg)
        self.arm_04.publish(self.arm_04_pos_msg)
        self.arm_tools.publish(self.arm_tools_pos_msg)

    def set_arm_pose(self, req):
        if req.mode in ["close", "open", "toggle", "set"]:
            if req.mode == "close" or (req.mode == "toggle" and self.arm_state in ["Open", "User"]):
                self.arm_state = "Closed"
                self.arm_01_pos_msg.data = -1.57
                self.arm_02_pos_msg.data = -0.4
                self.arm_03_pos_msg.data = -1.1
                self.arm_04_pos_msg.data = -1.57
                self.arm_tools_pos_msg.data = -1.57
            elif req.mode == "open" or (req.mode == "toggle" and self.arm_state == "Closed"):
                self.arm_state = "Open"
                self.arm_01_pos_msg.data = 0.0
                self.arm_02_pos_msg.data = 0.0
                self.arm_03_pos_msg.data = 0.0
                self.arm_04_pos_msg.data = 0.0
                self.arm_tools_pos_msg.data = 0.0
            elif req.mode == "set":
                self.arm_state = "User"
                self.arm_01_pos_msg.data = req.pos_arm_01
                self.arm_02_pos_msg.data = req.pos_arm_02
                self.arm_03_pos_msg.data = req.pos_arm_03
                self.arm_04_pos_msg.data = req.pos_arm_04
                self.arm_tools_pos_msg.data = req.pos_arm_tools
            self.arm_01.publish(self.arm_01_pos_msg)
            self.arm_02.publish(self.arm_02_pos_msg)
            self.arm_03.publish(self.arm_03_pos_msg)
            self.arm_04.publish(self.arm_04_pos_msg)
            self.arm_tools.publish(self.arm_tools_pos_msg)
            return True
        else:
            return False

    def set_mast_pose(self, req):
        if req.mode in ["close", "open", "toggle", "set", "rotate"]:
            if (req.mode == "close" and not self.mast_state == "Panorama") or (req.mode == "toggle" and self.mast_state == "Raised"):
                self.mast_state = "Lowered"
                self.mast_p_pos_msg.data = 1.35  # not quite 90 degrees
                self.mast_02_pos_msg.data = 1.57
                self.mast_cameras_pos_msg.data = 0.0
            elif (req.mode == "open" and not self.mast_state == "Panorama") or (req.mode == "toggle" and self.mast_state == "Lowered"):
                self.mast_state = "Raised"
                self.mast_p_pos_msg.data = 0.0
                self.mast_02_pos_msg.data = -0.5
                self.mast_cameras_pos_msg.data = 0.0
            elif req.mode == "set":
                # This mode is only used for creating panoramas, so it blocks the others
                if req.pos_mast_02 <= -2 or req.pos_mast_02 == -0.5:
                    self.mast_state = "Raised"
                else:
                    self.mast_state = "Panorama"
                self.mast_p_pos_msg.data = req.pos_mast_p
                self.mast_02_pos_msg.data = req.pos_mast_02
                self.mast_cameras_pos_msg.data = req.pos_mast_cameras
            # Allow rotation messages only when not creating panorama
            elif req.mode == "rotate" and self.mast_state == "Raised":
                # Accepts changes eg +1 -1 rather than specific values
                if not req.rot_x == 0.0:
                    # Limit 90 degrees either direction
                    self.mast_cameras_pos_msg.data = max(-1.57, min(
                        self.mast_cameras_pos_msg.data + req.rot_x, 1.57))
                    self.mast_cameras.publish(self.mast_cameras_pos_msg)
                if not req.rot_y == 0.0:
                    # Limit 180 degrees either direction (starts at -0.5 radians, so one is pi-0.5 and the other is -pi-0.5)
                    self.mast_02_pos_msg.data = max(-3.64, min(
                        self.mast_02_pos_msg.data + req.rot_y, 2.64))
                    self.mast_02.publish(self.mast_02_pos_msg)
            self.mast_p.publish(self.mast_p_pos_msg)
            self.mast_02.publish(self.mast_02_pos_msg)
            self.mast_cameras.publish(self.mast_cameras_pos_msg)
            return True
        else:
            return False


if __name__ == "__main__":
    rospy.init_node("curiosity_mars_rover_arm_mast_node")
    curiosity_mars_rover_arm_mast_object = CuriosityMarsRoverArmAndMast()
    rospy.spin()
