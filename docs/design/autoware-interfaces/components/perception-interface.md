# Perception

![Node diagram](images/Perception-Bus-ODD-Architecture.drawio.svg)

## Inputs

### PointCloud

PointCloud data published by Lidar. 

- [sensor_msgs/msg/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)

### Image

Image frame captured by camera. 

- [sensor_msgs/msg/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)
  
### Vehicle kinematic state

current position of ego, used in traffic signals recognization. See Iutputs of Planning.

### Lanelet2 Map

map of the environment. See Iutputs of Planning.

## Output

### 3D Object Predictions

3D Object detected, tracked and predicted by sensor fusing. See Iutputs of Planning.

### Traffic Light Response

traffic light signals recognized by object detection model. See Iutputs of Planning.


