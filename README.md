<!-- vscode-markdown-toc -->
* 1. [Team member](#Teammember)
* 2. [Setup](#Setup)
* 3. [Run](#Run)
	* 3.1. [Autonomous mode: make the quadrotor autonomously navigate to a target goal](#Autonomousmode:makethequadrotorautonomouslynavigatetoatargetgoal)
	* 3.2. [Keyboard mode: control the quadrotor with keyboard and map with Octomap](#Keyboardmode:controlthequadrotorwithkeyboardandmapwithOctomap)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc --># Group Project: Autonomous System

![git_gif](https://gitlab.lrz.de/ros_quadrotor/ros_quadrotor/-/raw/00d409793b373930e7510fc00c265fac5313a4da/quadrotor_test1.gif)


##  1. <a name='Teammember'></a>Team member
* Runxin Wang (runxin.wang@tum.de)
* Xiaolaing Li (xiaoliang.li@tum.de)

##  2. <a name='Setup'></a>Setup

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. Download the Unity executable files "ROS_with_SkyCamera.zip". 
    >Link:  https://syncandshare.lrz.de/getlink/fi4C8XwwgNP3gtPScV2b48pW/
    
3. Install [depth-image-proc](http://wiki.ros.org/depth_image_proc).
```shell
sudo apt-get install ros-melodic-depth-image-proc  # exchange melodic for your ros distro if necessary
```


4. Install [Octomap](http://wiki.ros.org/octomap).
```shell
sudo apt-get install ros-melodic-octomap-ros # exchange melodic for your ros distro if necessary
sudo apt-get install ros-melodic-octomap-msgs
sudo apt-get install ros-melodic-octomap-server
sudo apt-get install ros-melodic-octomap-rviz-plugins 
```

5. Create a workspace and clone the [repository](https://gitlab.lrz.de/ros_quadrotor/ros_quadrotor), then build the workspace. 
```shell
# cd ~/workspace_path
git clone https://gitlab.lrz.de/ros_quadrotor/ros_quadrotor.git
catkin build
```
6. Unzip the Unity executable files to **/devel/lib/simulation**
```shell
# cd ~/Unity_executable_files_path
unzip -d ~/workspace_path/devel/lib/simulation ROS_with_SkyCamera_executable.zip
```

##  3. <a name='Run'></a>Run
###  3.1. <a name='Autonomousmode:makethequadrotorautonomouslynavigatetoatargetgoal'></a>Autonomous mode: make the quadrotor autonomously navigate to a target goal

**Step 1**. Open a terminal from your workspace, prepare to take off.
```shell
source devel/setup.bash
roslaunch simulation simulation.launch
```
**Step 2**. Open the second terminal, open state machine.
```shell
source devel/setup.bash
rosrun controller_pkg state_machine 
```

**Step 3**. Open the third terminal, take off.
```shell
source devel/setup.bash
roslaunch simulation mission_starter.launch
```

**Step 4**. Open another terminal, send the goal.

```shell
source devel/setup.bash
rosrun mbot_navigation move_test 100 0 0
```

###  3.2. <a name='Keyboardmode:controlthequadrotorwithkeyboardandmapwithOctomap'></a>Keyboard mode: control the quadrotor with keyboard and map with Octomap

**Step 1**. Open a terminal from your workspace.
```shell
source devel/setup.bash
roslaunch simulation simulation.launch use_octomap:=true
```
**Step 2**. Run keyboard control node.
```shell
source devel/setup.bash
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard # holonomic control
```
or

```shell
source devel/setup.bash
rosrun mbot_teleop mbot_teleop.py # smoother movement trajectory
```
**Step 3**. Use the keyboard to control the quadrotor to complete the mapping

**Step 4**. In rviz, enable OccupancyGrid or OccupancyMap to view the map in 3D or 2D. For better display, you can switch to Orbit view. Save the **/projected_map** by running 
```shell
rosrun map_server map_saver map:=/projected_map -f PATH_TO_YOUR_FILE/mymap
```
![git_png](https://gitlab.lrz.de/ros_quadrotor/ros_quadrotor/-/raw/nav_2d/projected_map_octomap.png)

