# Tutorials

This page explains how to try out Autoware through simulations.

There are some methods and types of simulations.
Read the following sections and choose a simulation method/type you want

See the left sidebar for how to run each simulation.

!!! warning

    The namings of each simulation are under discussion and would be changed in the near future.

## Simulation methods

### Ad hoc simulation

Ad hoc simulation is a simulation method that you run simulations flexibly on your local machine.
It is recommended for your first trial.

### Scenario simulation

Scenario simulation is a simulation method that a scenario runner runs simulations based on scenarios you defined.
It is often run automatically on a CI system, but you can also run this simulation on your local machine.

## Simulation types

Autoware supports three types of simulations.  
Note that it is just a rough classification, and that there are several minor derivative versions.

### Planning simulation

Planning simulation is a simulation type that uses simple dummy data and tests mainly planning/control components.
It brings the car to the destination while avoiding pedestrians and surrounding cars.

#### How the planning simulation is achieved

- Generates a path to the goal destination
- Controls the car along the generated path
- Detect humans and surrounding cars for the safe operation

In the real-world operation, we need to detect traffic signs and signals to follow the rules, but in this simulation, we only do path generation, path following, and obstacle avoidance.

### Rosbag replay simulation

Rosbag replay simulation is a simulation type that uses Rosbag data and tests mainly perception modules, such as localization and object recognition.
Sometimes it is used for endurance tests by repeatedly playing back the data.

#### Localization

Rosbag simulation performs localization, which is the process of figuring out the vehicle pose on the map or on a reference coordinate system.
In other words, localization is the process of determining the vehicle pose on a local area map (e.g., map of the Tokyo area) or on the earth, or on a specific coordinate system (e.g., the tunnel entrance or the initial pose).
Usually, the word "localization" means estimating the position of an object, but in autonomous driving, this word also includes estimating the pose of a vehicle.

##### Why we need the localization

- To know the current location in the path to the destination
- To facilitate decisions and control in the operation
  - Exploit preset locations of traffic lights and signs on the map to improve recognition accuracy
  - Exploit preset locations of stop lines and driving lanes for smooth driving and stopping
  - Determine if a pedestrian is standing in front of a crosswalk and give way
  - Determine if another vehicle is waiting in front of a stop line and use this information to control the vehicle
- To estimate the reliability of GNSS
  - One can check if the car is in the GNSS-available environment or expect multi path errors caused by the reflection in an urban environment

##### How localization is achieved

The current localization system in Autoware is accomplished by GNSS and NDT.
GNSS is a satellite based positioning system. NDT (Normal Distribution Transform) is an algorithm that estimates the vehicle pose by matching LiDAR sensor data to a point cloud map.

### Digital twin simulation

Digital twin simulation is a simulation type that produces realistic data and simulate almost the entire system.
