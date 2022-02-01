# PX4_advanced_control

Repo consists of a base controller and advanced controllers for PX4-based drones. The advanced controllers extend the base controller. To implement an advanced controller, use `advanced_controller` and `advanced_cmd_line`. 

The code is set up to switch between base controller and advanced controller during flight. Switching happens either manually or automatically. To manually switch, the user can use pre-defined commands in `base_cmd_line` and `advanced_cmd_line`. The code automatically switches out of the advanced controller if the drone becomes unstable (see `check_security` method).

## Setup

To install the required packages, follow the ROS_Gazebo_PX4_Setup guide. Some information may be outdated, so check the PX4 documentation for updates. The following package versions have been successfully tested:

* PX4 - 1.9.2 on Ubuntu 18.4 / 1.9.0dev on Hexacopter
* Mavros - 0.32.1 on Ubuntu 18.4 / 1.3.0 on Odroid
* Mavlink - origin/debian/kinetic/mavlink (we are running ROS melodic, but the kinetic branch is the right version for Mavlink) / 1.0 (!?) on Odroid

To see which versions you are running, `roscd <pkg>` and run `git branch -a`. If the git head is not pointing to the same version as the one above, use `git checkout tags/v<x.xx.x>` to checkout the right version.


## Run

Use `sitl_gazebo.launch` to  test the controller in Gazebo. A seperate terminal will launch, which we use to command the drone. The specified commands are found in the `base_cmd_line` and `advanced_cmd_line` files. 

For experiments, we first launch `offboard.launch` to initialize the communication with the vehicle and then launch `cmd_line.launch` in a seperate termianl. The launch files are seperated, because we want to use two different terminals when ssh'ing to the drone's onboard computer.

## Kush & Nick's OTFL Additions
**External Forces & Moments**
While the simulation is running, you can apply forces and moments to the vehicle via the command line. These disturbances are step signals rather than impulse signals, such that they can be interpreted as a change in simulation dynamics. Make sure you are in the `PX4_advanced_control_OTFL/controllers/src` directory and run the following command to impart step forces and moments:
```
./disturbance.sh Fx Fy Fz Mx My Mz
```
Replace Fx, Fy, Fz with force values in units of Newtons and Mx, My, Mz with moment values in units of Netwon-meters. If the permissions of the shell script are not properly set, try the following command: `chmod +x disturbance.sh`

**Changing Mass/Inertia Properties**
We have configured `main_control.py` to increment the mass/inertia of the drone through the course of the simulation. The following details the process through which we implemented this capability.
1. Add the following to your python script: `from gazebo_msgs.srv import SetLinkProperties, GetLinkProperties`
2. Create a ServiceProxy to set link properties: `setLP_client = rospy.ServiceProxy("/gazebo/set_link_properties", SetLinkProperties)`
3. Create another ServiceProxy to get the link properties: `getLP_client = rospy.ServiceProxy("/gazebo/get_link_properties", GetLinkProperties)`
4. Use the following command to change the properties of a specified link: 
```
setLP_response = setLP_client("link_name", center_of_mass, gravity_mode, mass, ixx, ixy, ixz, iyy, iyz, izz)
```
The inputs to the function are the parameters of the SetLinkProperties Service (see https://docs.ros.org/en/diamondback/api/gazebo/html/srv/SetLinkProperties.html)
5. Usually you do not change all the properties of a given link, so it is useful to get the current properties of the link and apply those to setLP_client. To get the current link properties:
```
LP = getLP_client("link_name")
```
**Navigation State**
We have added navigtion state information to `base_controller.py`. This can be used by the OFTL algorithm and `main_control.py`. In `main_control.py` it can be accessed using the following:
```
# True current position
OffboardControl.curr_position
# True current velocity
OffboardControl.curr_velocity
# True current acceleration
OffboardControl.curr_acceleration
# True current angular acceleration
OffboardControl.curr_angular_acceleration
# Current euler angle (rad) of the drone x<->roll, y<->pitch, z<->yaw
OffboardControl.curr_orientation
# True current body rates
OffboardControl.curr_body_rates
# True current euler rates
OffboardControl.curr_euler_rates
```