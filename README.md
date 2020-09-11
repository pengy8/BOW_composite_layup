# BOW_composite_layup
Code for BOW transporting a composite sheet


## Getting Started

1. In a Win10 computer, run the Azure Kinect RR Service from Kinect_Azure_RR_Interface; also, run the SpeechRecognition_RR_service.py for the voice command service


2. In a Ubuntu 14.04 + ROS Indigo computer, run the jointcontroller_host and peripherals_host from baxter_rr_bridge with port 1111 and 2222
```
rosrun baxter_rr_bridge jointcontroller_host.py --port 1111
rosrun baxter_rr_bridge peripherals_host.py --port 2222
```

3. Setup the ROS network for the Ridgeback mobile base:
```
export ROS_MASTER_URI=http://ridgeback_ip:11311 # In our project case, ridgeback_ip = 192.168.131.1
export ROS_IP=local_pc_ip # In our project case, local_pc_ip = 192.168.131.51
```

4. To run the code, under the script folder, run
```
python Main_BOW_Layup.py
```
or
```
python ridgeback_QP_ForceKinect_kinectAzure_RR3_decoupled_20200313.py
```

## Prerequisites

### Hardware:
Baxter from Rethink
Ridgeback from ClearPath
Kinect Azure from Microsoft

### Software
Ridgeback and baxter ROS Indigo installation are required.


Kinect_Azure_RR_Interface 
```
https://github.com/wlawler45/Kinect_Azure_RR_Interface
```
baxter_rr_bridge 
```
https://github.com/rpiRobotics/baxter_rr_bridge
```


