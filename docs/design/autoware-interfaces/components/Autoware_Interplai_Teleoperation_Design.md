​																			**Autoware Interplai Teleoperation Design**



**Remote Monitoring - Architecture:**



![image-20230404160609015](/home/dark/snap/typora/78/.config/Typora/typora-user-images/image-20230404160609015.png)



**Phase – 1 (Remote monitoring):**

**On Vehicle Side:**

- ROS2 node for starting/ launching the robot script
for ex: WebRTCDataTransferNode
- Gstream pipeline for gathering video data from 4 cameras and audio from 1  microphone
- Data channel for transmitting/ receiving control commands 
- Registering the vehicle ID on signaling server (communication medium)

- Integrate ROS2 node (WebRTCDataTransferNode) inside launch file of Autoware Universe

- Communicate with the signaling server, hosted on (ex: AWS, GCP) where Vehicle ID can get registered



**Dash board:**



![image-20230404160644152](/home/dark/snap/typora/78/.config/Typora/typora-user-images/image-20230404160644152.png)





**On Remote Side:**

- Web login setup for remote user for monitoring the vehicle

- Option to choose the remote monitoring on web page

- ID mapping between vehicle and remote operator happens on signaling server , to allow only authenticated access

- Disconnect button for closing the remote monitoring



**Phase – 2 (Tele-operation):**



In addition to phase – 1,

- We have option to send GPS coordinates from vehicle to remote web page, to monitor the vehicle state

- We can send control commands via data channel from operator web page using joystick to vehicle
	- Control commands: Throttle, Steering

- We have ROS2 topic “operator_control_command”, which accepts the incoming control commands from operator

Application: Tele-operation for parking lot



Vehicle to Infrastructure communication
	-5G Modem(simcom,quectel)

- Auto-ware platform’s “emergency stop”  can be reused for safe teleoperation as redudant safety system

