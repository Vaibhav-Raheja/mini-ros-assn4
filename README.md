# ECE598JK Assigment 4 Instructions

For this assignment you will be working in the following files. 
- `~/ws_mini_assn4/src/mini_ros/scripts/measured_zmp.py`
- `~/ws_mini_assn4/src/mini_ros/scripts/approx_zmp_multi_mass.py`
- `~/ws_mini_assn4/src/mini_ros/scripts/approx_zmp_single_mass.py`
- `~/ws_mini_assn4/src/mini_ros/config/walkingParam.yaml`
- `~/ws_mini_assn4/src/mini_ros/scripts/ball_detection.py`

Implement the parts of the code marked TODO in each file and follow the instructions listed in the code and instruction pdf. The starter code setup and commands to run the simulation and hardware are outlined below.

# ZMP Walking Simulation Setup

Clone mini-ros-assn4 repo from Github

Create a new workspace for assignment 4 as the code structure has changed from the previous assignment.
```
mkdir -p ~/ws_mini_assn4/src && cd ~/ws_mini_assn4/src
git clone https://github.com/uiuckimlab/mini-ros-assn4.git
```

Make using catkin and source ros workspace env
```
cd ~/ws_mini_assn4 && catkin_make
```

Terminal A: launch simulator and ros controller nodes

```
cd ~/ws_mini_assn4 && source devel/setup.bash && roslaunch mini_ros sim_bringup.launch
```

Start the simulation by clicking the play button on the botton left of the Gazebo UI.

Terminal B: start `sensors.launch` that start the below nodes. Note that you will need to rerun this launch file whenever you make changes to the zmp and perception scripts.
- rviz node 
- com calc + viz node 
- odometry node (`mini_ros/scripts/odom.py`)
- approx zmp calc + viz node (`mini_ros/scripts/approx_zmp.py`)
- measured zmp calc + viz node (`mini_ros/scripts/measured_zmp.py`)
- ball_detection node (`mini_ros/scripts/ball_detection.py`)

```
cd ~/ws_mini_assn4 && source devel/setup.bash && roslaunch mini_ros sensors.launch
```

# ZMP Walking Simulation 

Set `use_sim` to `True` in `walkingParams.yaml`
## 1) Run zmp walking motion without IMU feedback
Set all stabilizer gains to 0 in `walkingParams.yaml`
```
cd ~/ws_mini_assn4 && source devel/setup.bash && roslaunch mini_ros walking.launch 
```
## 2) Walking motion with IMU feedback
Find and set optimal stabilizer gains params in `walkingParams.yaml`
```
cd ~/ws_mini_assn4 && source devel/setup.bash && roslaunch mini_ros walking.launch 
```
## 3) Walking motion with Camera + IMU feedback
Set `use_perception` to `True` in `walkingParams.yaml`
```
cd ~/ws_mini_assn4 && source devel/setup.bash && roslaunch mini_ros walking.launch 
```
# ZMP Walking Hardware Setup

## 1) SSH to orangepi and setup starter code in hardware
```
ssh mini@{ORANGEPI_IP_ADDRESS} 
mkdir -p ~/ws_mini_assn4/src && cd ~/ws_mini_assn4/src
git clone https://github.com/uiuckimlab/mini-ros-assn4.git
```

## 2) Add CATKIN_IGNORE
The ros workspace in orangepi cannot `catkin_make` with gazebo related dependencies. Make it ignore the `mini_ros_gazebo` repo by adding an empty CATKIN_IGNORE file.
```
touch ~/ws_mini_assn4/mini_ros_gazebo/CATKIN_IGNORE
```
```
cd ~/ws_mini_assn4 && catkin_make
```
## Change walkingParam.yaml
Set `use_sim` to `False` and `use_perception` to `True` in `walkingParams.yaml`
## 3) Bringup hardware control nodes
Terminal A: launch hw and ros controller nodes
```
cd ~/ws_mini_assn4 && source devel/setup.bash && roslaunch mini_ros hw_bringup.launch imu:=true camera:=true
```
## 4) Run walking motion with Camera + IMU feedback
```
cd ~/ws_mini_assn4 && source devel/setup.bash && roslaunch mini_ros walking.launch
```
