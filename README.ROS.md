contents:
README.md - this file
romipi - ROS source for the romipi node
Romi Operation.perspective - rqt perspective for the romipi robot

add to .bashrc
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src && ln -s ~/multi-robotics/robots/Romi-All/ROS/romipi romipi

Bringup Robot

RemotePC$ roscore
RomiPi$ roslaunch romipi_bringup romipi_robot.launch

Keyboard Teleoperation

RemotePC$ roslaunch romipi_teleop romipi_teleop_key.launch

Joystick Teleoperation

connect controller via usb 
LocalPC$ sudo ds4drv —hiraw
LocalPC$ roslaunch teleop_twist_joy teleop.launch

Joystick Teleoperation Installation Notes

PS4 Controller for RomiPi

Below is my experience with
Ubuntu 18.04 VM on my grey MacBook

Install Dual Shock 4 Driver
* sudo apt-get install python-pip
* sudo pip install ds4drv
* connect controller via usb
* sudo ds4drv —hiraw
** Open instructions here if you have trouble: https://github.com/chrippa/ds4drv

Test with GUI App (optional, but suggested)
* sudo apt-get install jstest-gtk
* sudo jstest-gtk

PS4 controller working with USB
RemotePC$ roslaunch romipi_teleop romipi_teleop_ds4_joy.launch
RemotePC$ rosrun romipi_astar  romipi_astar_node.py

Monitor Robot with rqt
 
RemotePC$ rqt
and open the perspective saved in romi-all/ROS

Monitor with RVIZ

RemotePC$ roslaunch romipi_bringup romipi_remote.launch
RemotePC$ rosrun rviz rviz -d `rospack find romipi_description`/rviz/model.rviz

ERROR: No transform from [wheel_left_link] to [base_footprint]

RaspiCam
adapted from http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/#installation 21.5.5.1
cd ~/catkin_ws/src
git clone https://github.com/UbiquityRobotics/raspicam_node.git
sudo apt-get install ros-kinetic-compressed-image-transport ros-kinetic-camera-info-manager
roslaunch raspicam_node camerav2_1280x960.launch
rqt_image_view
* roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

TODO
* What does Romi 32u4 processor need to publish?
** possibles: joint_states, tf, tf_static
Gazebo Simulation
* roslaunch romipi_gazebo romipi_empty_world.launch

TODO

