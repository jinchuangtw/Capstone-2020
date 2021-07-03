# Capstone-2020 README

### Prerequisites
1. PX4-Autopilot v1.10 Download ->
```git clone https://github.com/PX4/Firmware```
2. Mavros (binary installation), see references in https://docs.px4.io/master/en/ros/mavros_installation.html

### Build
1. Replace the files in `(your path to PX4)/Tools/sitl_gazebo/models` by the files in `(your path to Capstone-2020)/Capstone2020/models`
3. Then, simply `catkin build` or `catkin_make`

### Run
In terminal, 
```
roscore

no_sim=1 make px4_sitl_default gazebo

roslaunch capstone2020 iris_fpvD.launch

roslaunch capstone2020 px4_sitl.launch 

***In px4 commander: param set EKF2_AID_MASK 25

roslaunch capstone2020 aruco_detect_capstone2020.launch 

rosrun offboard_pkg takeoff.py
if the mode cannot switch to offboard, ctrl c this node and reopen.

chmod +x fiducial_tf.py
rosrun offboard_pkg fiducial_tf.py

After the aircraft reach a sufficient altitude
***In px4 commander: param set EKF2_AID_MASK 24
```
