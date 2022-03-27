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
    
    oculus_sub : Subscribe Oculus Quest2 Data from ROS#.
                    It includes VR pose, Controller pose.
    
    study_rviz : Code for Rivz study 
    
    visualization_demo : Display Oculus Quest2 Data to Rviz.
    
    dynamixel_controller : Communicate with Dynamixel. Read&write motor data.

### msg
References for topic messages.
### srv
References for Dynamixel control.

---
## Tips

- 컨트롤러, 헤드 마운트 pose 보려면 rosrun TrackDesign2022 visualization_demo 하면 됩니다
- 