add to .bashrc
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src && ln -s ~/multi-robotics/robots/Romi-All/ROS/romipi romipi

Bringup Robot

RemotePC$ roscore
RomiPi$ roslaunch romipi_bringup romipi_robot.launch

Keyboard Teleoperation

RemotePC$ roslaunch romipi_teleop romipi_teleop_key.launch

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

