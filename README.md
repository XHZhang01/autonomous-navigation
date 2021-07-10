sudo apt-get install ros-noetic-depth-image-proc


#安装octomap
sudo apt-get install ros-kinetic-octomap-ros 
sudo apt-get install ros-kinetic-octomap-msgs
sudo apt-get install ros-kinetic-octomap-server


roslaunch simulation simulation.launch

rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard

rosrun mbot_navigation move_test.py 