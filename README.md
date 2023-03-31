# Project: Autonomous Navigation


![git_gif](https://github.com/XHZhang01/autonomous-navigation/blob/master/Report/quadrotor_test1.gif)

<!-- vscode-markdown-toc -->
* 1. [Author](#author)
* 2. [Setup](#setup)
* 3. [Run](#run)
    * 3.1. [Autonomous mode: autonomously navigate to a target goal](#autonomous-mode:-autonomously-navigate-to-a-target-goal)
    * 3.2. [Keyboard mode: control the quadrotor with keyboard and map with Octomap](#keyboard-mode:-control-the-quadrotor-with-keyboard-and-map-with-octomap)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->


##  1. <a name='author'></a>Author
* Xuhui Zhang (xuhui.zhang@tum.de)

##  2. <a name='setup'></a>Setup

**Step 1**. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

**Step 2**. Download the Unity executable files "ROS_with_SkyCamera.zip". 
    >Link:  https://syncandshare.lrz.de/getlink/fi4C8XwwgNP3gtPScV2b48pW/
    
**Step 3**. Install [depth-image-proc](http://wiki.ros.org/depth_image_proc).
```shell
sudo apt-get install ros-melodic-depth-image-proc  # exchange melodic for your ros distro if necessary
```


**Step 4**. Install [Octomap](http://wiki.ros.org/octomap).
```shell
sudo apt-get install ros-melodic-octomap-ros # exchange melodic for your ros distro if necessary
sudo apt-get install ros-melodic-octomap-msgs
sudo apt-get install ros-melodic-octomap-server
sudo apt-get install ros-melodic-octomap-rviz-plugins 
```

**Step 5**. Create a workspace and clone the [repository](git@github.com:XHZhang01/autonomous-navigation.git), then build the workspace. 
```shell
# cd ~/workspace_path
git clone git@github.com:XHZhang01/autonomous-navigation.git
catkin build
```
**Step 6**. Unzip the Unity executable files to **/devel/lib/simulation**
```shell
# cd ~/Unity_executable_files_path
unzip -d ~/workspace_path/devel/lib/simulation ROS_with_SkyCamera_executable.zip
```

##  3. <a name='run'></a>Run
###  3.1. <a name='autonomous-mode:-autonomously-navigate-to-a-target-goal'></a>Autonomous mode: autonomously navigate to a target goal

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
rosrun navigation send_goal 100 0 0
```

###  3.2. <a name='keyboard-mode:-control-the-quadrotor-with-keyboard-and-map-with-octomap'></a>Keyboard mode: control the quadrotor with keyboard and map with Octomap

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
rosrun teleop teleop.py # smoother movement trajectory
```
**Step 3**. Use the keyboard to control the quadrotor to complete the mapping

**Step 4**. In rviz, enable OccupancyGrid or OccupancyMap to view the map in 3D or 2D. For better display, you can switch to Orbit view. Save the **/projected_map** by running 
```shell
rosrun map_server map_saver map:=/projected_map -f PATH_TO_YOUR_FILE/mymap
```

<img src="https://github.com/XHZhang01/autonomous-navigation/blob/master/Report/projected_map_octomap.png" width = 40% height = 40% />
<img src="https://github.com/XHZhang01/autonomous-navigation/blob/master/Report/3D_Octomap.png" width = 35% height = 35% />

