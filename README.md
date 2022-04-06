# TrackDesign2022

---
##  Structure
### bag
rosbag files. 
### rviz
Rviz settings.
### src
main codes

    data_hub : Main codes for data. Declare subscriber/publisher instances.
    
    oculus_sub : Subscribe Oculus Quest2 data from ROS#. It includes VR pose, controller pose.
    
    visualization_demo : Display Oculus Quest2 data to Rviz.
    
    dynamixel_communicator : Communicate Serialy with Dynamixel. Read&write motor data.

    dynamixel_trajectory_calculator : Calculate trajectory

### msg
References for topic messages.
### srv
References for Dynamixel control.

---
## Tips

- 컨트롤러, 헤드 마운트 pose 보려면 rosrun TrackDesign2022 visualization_demo 하면 됩니다


- To Build this package successfully, you SHOULD download Dynamixel_SDK package and catkin_make it


- If you have some troubles with msg files, try [ source ~/catkin_ws/devel/setup.bash ] at EVERY terminal you're using.


- If you have some troubles with ttyUSB0 - permission, follow https://valueelectronic.tistory.com/202


- You should set minimum communication interval(1 [ms]).
  - echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  - Or,
  - setserial /dev/ttyUSB0 low_latency
