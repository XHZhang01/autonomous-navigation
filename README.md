#ROS

##安装depth-image-proc
sudo apt-get install ros-noetic-depth-image-proc


##安装octomap
sudo apt-get install ros-kinetic-octomap-ros 
sudo apt-get install ros-kinetic-octomap-msgs
sudo apt-get install ros-kinetic-octomap-server

##launch

roslaunch simulation simulation.launch

rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard
rosrun mbot_teleop mbot_teleop.py 


rosrun mbot_navigation move_test 40 0 0
