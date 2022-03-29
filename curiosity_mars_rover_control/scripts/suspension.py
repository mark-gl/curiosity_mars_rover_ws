#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import DeleteModel, DeleteModelResponse

class CuriosityMarsRoverSuspension(object):
    def __init__(self):
        rospy.loginfo("Curiosity Suspension Initialising...")

        self.distance_axis = 0.3
        self.distance_front_center = 0.5
        self.distance_back_center = 0.5

        self.publishers_curiosity_d = {}
        self.controller_ns = "curiosity_mars_rover"
        self.controller_command = "command"
        self.controllers_list = [   "suspension_arm_B2_L_joint_position_controller",
                                    "suspension_arm_B2_R_joint_position_controller",
                                    "suspension_arm_B_L_joint_position_controller",
                                    "suspension_arm_B_R_joint_position_controller",
                                    "suspension_arm_F_L_joint_position_controller",
                                    "suspension_arm_F_R_joint_position_controller"
                                ]
        for controller_name in self.controllers_list:
            topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
            self.publishers_curiosity_d[controller_name] = rospy.Publisher(
                topic_name,
                Float64,
                queue_size=1)

        self.wait_publishers_to_be_ready()
        self.init_publisher_variables()
        self.init_state()

        suspension_service_name = "/"+self.controller_ns+"/suspension_service"
        self.suspension_service = rospy.Service(suspension_service_name, DeleteModel, self.suspension_service_cb)

        rospy.loginfo("Curiosity Suspension...READY")

    def suspension_service_cb(self, req):
        mode_name = req.model_name
        if mode_name == "standard":
            self.suspension_arm_B2_L_pos_msg.data = 1
            self.suspension_arm_B2_R_pos_msg.data = 1
            self.suspension_arm_B_L_pos_msg.data = -0.3
            self.suspension_arm_B_R_pos_msg.data = -0.3
            self.suspension_arm_F_L_pos_msg.data = 1
            self.suspension_arm_F_R_pos_msg.data = 1

            self.suspension_arm_B2_L.publish(self.suspension_arm_B2_L_pos_msg)
            self.suspension_arm_B2_R.publish(self.suspension_arm_B2_R_pos_msg)
            self.suspension_arm_B_L.publish(self.suspension_arm_B_L_pos_msg)
            self.suspension_arm_B_R.publish(self.suspension_arm_B_R_pos_msg)
            self.suspension_arm_F_L.publish(self.suspension_arm_F_L_pos_msg)
            self.suspension_arm_F_R.publish(self.suspension_arm_F_R_pos_msg)
        else:
            self.suspension_arm_B2_L_pos_msg.data = float(mode_name.split(' ')[0])
            self.suspension_arm_B2_R_pos_msg.data = float(mode_name.split(' ')[1])
            self.suspension_arm_B_L_pos_msg.data = float(mode_name.split(' ')[2])
            self.suspension_arm_B_R_pos_msg.data = float(mode_name.split(' ')[3])
            self.suspension_arm_F_L_pos_msg.data = float(mode_name.split(' ')[4])
            self.suspension_arm_F_R_pos_msg.data = float(mode_name.split(' ')[5])

            self.suspension_arm_B2_L.publish(self.suspension_arm_B2_L_pos_msg)
            self.suspension_arm_B2_R.publish(self.suspension_arm_B2_R_pos_msg)
            self.suspension_arm_B_L.publish(self.suspension_arm_B_L_pos_msg)
            self.suspension_arm_B_R.publish(self.suspension_arm_B_R_pos_msg)
            self.suspension_arm_F_L.publish(self.suspension_arm_F_L_pos_msg)
            self.suspension_arm_F_R.publish(self.suspension_arm_F_R_pos_msg)

        response = DeleteModelResponse()
        response.success = True
        response.status_message = "Executed Suspension Mode="+str(mode_name)
        return response


    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        for controller_name, publisher_obj in self.publishers_curiosity_d.items():
            publisher_ready = False
            while not publisher_ready:
                rospy.loginfo("Checking Publisher for ==>"+str(controller_name))
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = (pub_num > 0)
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")

    def init_publisher_variables(self):
        """
        We create variables for more pythonic access access to publishers
        and not need to access any more
        :return:
        """
        self.suspension_arm_B2_L = self.publishers_curiosity_d[self.controllers_list[0]]
        self.suspension_arm_B2_R = self.publishers_curiosity_d[self.controllers_list[1]]
        self.suspension_arm_B_L = self.publishers_curiosity_d[self.controllers_list[2]]
        self.suspension_arm_B_R = self.publishers_curiosity_d[self.controllers_list[3]]
        self.suspension_arm_F_L = self.publishers_curiosity_d[self.controllers_list[4]]
        self.suspension_arm_F_R = self.publishers_curiosity_d[self.controllers_list[5]]

        # Init Messages
        self.suspension_arm_B2_L_pos_msg = Float64()
        self.suspension_arm_B2_R_pos_msg = Float64()
        self.suspension_arm_B_L_pos_msg = Float64()
        self.suspension_arm_B_R_pos_msg = Float64()
        self.suspension_arm_F_L_pos_msg = Float64()
        self.suspension_arm_F_R_pos_msg = Float64()

    def init_state(self):
        self.set_suspension_mode("standard")

    def set_suspension_mode(self, mode_name):
        if mode_name == "standard":
            self.suspension_arm_B2_L_pos_msg.data = 1
            self.suspension_arm_B2_R_pos_msg.data = 1
            self.suspension_arm_B_L_pos_msg.data = -0.3
            self.suspension_arm_B_R_pos_msg.data = -0.3
            self.suspension_arm_F_L_pos_msg.data = 1
            self.suspension_arm_F_R_pos_msg.data = 1

            self.suspension_arm_B2_L.publish(self.suspension_arm_B2_L_pos_msg)
            self.suspension_arm_B2_R.publish(self.suspension_arm_B2_R_pos_msg)
            self.suspension_arm_B_L.publish(self.suspension_arm_B_L_pos_msg)
            self.suspension_arm_B_R.publish(self.suspension_arm_B_R_pos_msg)
            self.suspension_arm_F_L.publish(self.suspension_arm_F_L_pos_msg)
            self.suspension_arm_F_R.publish(self.suspension_arm_F_R_pos_msg)
        else:
            self.suspension_arm_B2_L_pos_msg.data = 0
            self.suspension_arm_B2_R_pos_msg.data = 0
            self.suspension_arm_B_L_pos_msg.data = 0
            self.suspension_arm_B_R_pos_msg.data = 0
            self.suspension_arm_F_L_pos_msg.data = 0
            self.suspension_arm_F_R_pos_msg.data = 0

            self.suspension_arm_B2_L.publish(self.suspension_arm_B2_L_pos_msg)
            self.suspension_arm_B2_R.publish(self.suspension_arm_B2_R_pos_msg)
            self.suspension_arm_B_L.publish(self.suspension_arm_B_L_pos_msg)
            self.suspension_arm_B_R.publish(self.suspension_arm_B_R_pos_msg)
            self.suspension_arm_F_L.publish(self.suspension_arm_F_L_pos_msg)
            self.suspension_arm_F_R.publish(self.suspension_arm_F_R_pos_msg)

if __name__ == "__main__":
    rospy.init_node("CuriosityRoverSuspension_node", log_level=rospy.INFO)
    curiosity_mars_rover_suspension_object = CuriosityMarsRoverSuspension()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()


