# 1 Overview #

This package provides a robot to human localisation node


# 2 Node #

### 2.1 Subscribed Topics ###


- vehicle_controller/odom (nav_msgs::Odometry)

- localisation/twist (romea_localisation_msgs::ObservationTwist2DStamped)

- localisation/leader_twist(romea_localisation_msgs::ObservationTwist2DStamped)
    
- localisation/pose (romea_localisation_msgs::ObservationPose2DStamped)

- localisation/range (romea_localisation_msgs::ObservationRangeStamped)
        
- joy(sensor_msgs::Joy)


### 2.2 Published Topics ###

- leader_pose (romea_common_msgs::Position2DStamped)

### 2.3 Parameters ###


- ~controller (string, default: fsm) 

    The node can be controlled in using joystick or romea finite state machine (fsm).   
    By default the node is controlled by finite state machine. When joystick is used  
    four buttons are availables :  
    &nbsp;&nbsp;&nbsp;&nbsp;- A : start localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;- B : reset localisation  

- ~angular_speed_source (string, default odometry)

    Name of angular source speed :
    &nbsp;&nbsp;&nbsp;&nbsp;- angular_speed : data is read from localisation/angular_speed topic
    &nbsp;&nbsp;&nbsp;&nbsp;- twist : kinematic data is read from localisation/twist topic

- ~leader (string)

    Name of leader to follow. When fsm controller is used, this parameter is   
    optional and can be set by fsm_service. 
     
- ~minimal_kinematic_rate(float, default: 10) 

    Miminal rate of kinematic data. Kinematic data can come from odometry  
    or joint_state messages 

- ~autostart (bool, default: false)

    Start or not localisation processing 
    
- ~display (bool, default: false)

    Enable or not rviz display 


### 2.5 Service server ###

- fsm_service (romea_lifecycle_msgs::FSMService)

    When control is done by fsm state machine. Some services can be called via   
    fsm_service topic (see romea_lifecycle_msgs for more information about this topic).   
    Available fsm_service resquests are:  
    &nbsp;&nbsp;&nbsp;&nbsp;-start : start localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;-stop : stop localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;-reset : reset localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;-setLeader : set leader to follow  
    &nbsp;&nbsp;&nbsp;&nbsp;-checkStatus : check value of localisation status   
