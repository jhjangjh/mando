# Mando Workspace
This is a workspace for 2023 Mando autonomous driving competition.

## Run
1. Clone repository
```
git clone https://github.com/jhjangjh/mando.git
```
2. Build
```
catkin_make
source devel/setup.bash
```
3. Launch nodes
- Launch mission_generator
```
roslaunch mission_generator mission_generator.launch
```
- Launch local_planning
```
roslaunch local_planning local_planning.launch
```
- Launch vehicle_control
```
roslaunch vehicle_control control.launch
```

## Prerequisites
```
cd mando/src/perception
git clone https://github.com/mats-robotics/detection_msgs.git
```