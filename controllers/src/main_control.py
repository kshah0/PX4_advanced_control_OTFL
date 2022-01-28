#!/usr/bin/env python3

import sys
import threading 

# ROS python API
import rospy

# Stamped Pose msgs
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped, AccelWithCovarianceStamped
from std_msgs.msg import Int16, String
from sensor_msgs.msg import BatteryState

# mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# custom classes
from base_cmd_line import CommandLine
from base_controller import OffboardControl
from advanced_cmd_line import CommandLineExt
from advanced_controller import AdvancedController
from gazebo_msgs.srv import SetLinkProperties, GetLinkProperties 


def main():

    rospy.init_node('cmd_line_control', anonymous=True)
    updateFreq = 100
    rate = rospy.Rate(updateFreq)

    # check whether advanced controller should be started
    bool_adv_ctrl = rospy.get_param('~adv_ctrl', False)
    bool_sim = rospy.get_param('~bool_sim', False)
    if bool_sim:
        hoverVal = 0.565  # Value used only when simulation in gazebo
    else:
        hoverVal = rospy.get_param('~hoverThrust', 0.44)

    # Set up controllers and subscribers
    if bool_adv_ctrl:
        offboard_controller = AdvancedController(hoverVal=hoverVal, updateTime=1.0/updateFreq,
                                           bool_sim=bool_sim)
        cmd_line = CommandLineExt(offboard_controller)

    else:
        offboard_controller = OffboardControl(hoverVal=hoverVal, updateTime=1.0/updateFreq)
        cmd_line = CommandLine(offboard_controller)

    # if bool_sim:
    rospy.Subscriber('mavros/local_position/pose_cov', PoseWithCovarianceStamped,
                         offboard_controller.pos_callback)
    # else:
    #     rospy.Subscriber('mavros/vision_pose/pose', PoseStamped,
    #                      offboard_controller.pos_callback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                     offboard_controller.estimate_callback)
    rospy.Subscriber('mavros/state', State,
                     offboard_controller.state_callback)
    rospy.Subscriber('mavros/setpoint_raw/target_attitude', AttitudeTarget,
                     offboard_controller.euler_callback)


    rospy.Subscriber('mavros/local_position/velocity_body_cov', TwistWithCovarianceStamped,offboard_controller.vel_callback)
    rospy.Subscriber('mavros/local_position/accel', AccelWithCovarianceStamped,offboard_controller.accel_callback)

    setLP_client = rospy.ServiceProxy("/gazebo/set_link_properties", SetLinkProperties)
    getLP_client = rospy.ServiceProxy("/gazebo/get_link_properties", GetLinkProperties)

    # Sleep some time for the system to be ready
    for _ in range(10):
        rate.sleep()

    # create input thread for command line inputs! Must be separate from main thread running ROS
    input_thread = threading.Thread(target=cmd_line.get_command_from_console,
                                    name='input command thread')
    input_thread.start()

    if bool_sim:
        rospy.loginfo(" --- READY FOR SIMULATION! --- ")
    else:
        rospy.loginfo(" --- READY FOR EXPERIMENT! --- ")

    rospy.sleep(10.)
    offboard_controller.setTakeoff()
    rospy.sleep(10.)
    
    c = 0
    # ool = True

    while not rospy.is_shutdown():
        c += 1
        offboard_controller.update()
        #offboard_controller.move(1,0,1.5)
        #rospy.loginfo(offboard_controller.curr_pose_cov)
        #rospy.sleep(20.0)
        # offboard_controller.move(-1,0,1.5)
        
        # rospy.loginfo(LP.status_message)
        if c > 7:
            LP = getLP_client("base_link")
            rospy.loginfo(LP.mass)
            rospy.loginfo("Changing Mass...")
            new_mass = 5+2*(c-10)
            new_ixx = .03+.03*(c-10)
            setLP_response = setLP_client("base_link", LP.com, LP.gravity_mode, new_mass, new_ixx, LP.ixy, LP.ixz, LP.iyy, LP.iyz, LP.izz)
            rospy.loginfo(setLP_response.success)
            rospy.loginfo(setLP_response.status_message)

            LP = getLP_client("base_link")
            rospy.loginfo(LP.mass)

            # ool = False

        rospy.sleep(1.5)
        rate.sleep()


if __name__ == "__main__":
    main()
