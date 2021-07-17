# Group Project: Autonomous System




## Setup

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. Download the Unity executable files "ROS_with_SkyCamera.zip". 
    >Link:  https://syncandshare.lrz.de/getlink/fi4C8XwwgNP3gtPScV2b48pW/
    
3. Install [depth-image-proc](http://wiki.ros.org/depth_image_proc).
```shell script
sudo apt-get install ros-melodic-depth-image-proc  # exchange melodic for your ros distro if necessary
```


4. Install [Octomap](http://wiki.ros.org/octomap).
```shell script
sudo apt-get install ros-melodic-octomap-ros # exchange melodic for your ros distro if necessary
sudo apt-get install ros-melodic-octomap-msgs
sudo apt-get install ros-melodic-octomap-server
sudo apt-get install ros-melodic-octomap-rviz-plugins 
```

5. Create a workspace and clone the [repository](https://gitlab.lrz.de/ros_quadrotor/ros_quadrotor), then build the workspace. 
```shell script
# cd ~/workspace_path
git clone https://gitlab.lrz.de/ros_quadrotor/ros_quadrotor.git
catkin build
```
6. Unzip the Unity executable files to workspace
```shell script
# cd ~/Unity_executable_files_path
unzip -d ~/workspace_path/devel/lib/simulation ROS_with_SkyCamera_executable.zip
```

## Run
### Make the quadrotor autonomously navigate to a target goal.

**Step 1**. Open a terminal from your workspace, prepare to take off.
```shell script
source devel/setup.bash
roslaunch simulation simulation.launch
```
**Step 2**. Open the second terminal, open state machine.


**Step 3**. Open the third terminal, take off.
```shell script
source devel/setup.bash
roslaunch simulation mission_starter.launch
```

**Step 4**. Open another terminal, send the goal.

```shell script
source devel/setup.bash
rosrun mbot_navigation move_test 100 0 0
```

### Control the quadrotor with keyboard and map with Octomap.

**Step 1**. Open a terminal from your workspace, prepare to take off.
```shell script
source devel/setup.bash
roslaunch simulation simulation_octomap.launch
```
**Step 2**. Run keyboard control node.
```shell script
source devel/setup.bash
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard # holonomic control
```
or

```shell script
source devel/setup.bash
rosrun mbot_teleop mbot_teleop.py # Smoother movement trajectory
```


