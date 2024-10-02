# Simulation tutorials

Simulations provide a way of verifying Autoware's functionality before field testing with an actual vehicle.  
There are three main types of simulation that can be run ad hoc or via a scenario runner.

## Simulation methods

### Ad hoc simulation

Ad hoc simulation is a flexible method for running basic simulations on your local machine, and is the recommended method for anyone new to Autoware.

### Scenario simulation

Scenario simulation uses a scenario runner to run more complex simulations based on predefined scenarios.
It is often run automatically for continuous integration purposes, but can also be run on a local machine.

## Simulation types

### Planning simulation

Planning simulation uses simple dummy data to test the Planning and Control components - specifically path generation, path following and obstacle avoidance. It verifies that a vehicle can reach a goal destination while avoiding pedestrians and surrounding cars, and is another method for verifying the validity of Lanelet2 maps. It also allows for testing of traffic light handling.

#### How does planning simulation work?

1. Generate a path to the goal destination
2. Control the car along the generated path
3. Detect and avoid any humans or other vehicles on the way to the goal destination

### Rosbag replay simulation

Rosbag replay simulation uses prerecorded rosbag data to test the following aspects of the Localization and Perception components:

- Localization: Estimation of the vehicle's location on the map by matching sensor and vehicle feedback data to the map.
- Perception: Using sensor data to detect, track and predict dynamic objects such as surrounding cars, pedestrians, and other objects

By repeatedly playing back the data, this simulation type can also be used for endurance testing.

### Digital twin simulation

Digital twin simulation is a simulation type that is able to produce realistic data and simulate almost the entire system. It is also commonly referred to as end-to-end simulation.


# Evaluation Tutorials

## Components Evaluation
Components evaluation tutorials provide a way to evaluate the performance of Autoware's components in a controlled environment.

### Localization Evaluation

The Localization Evaluation tutorial provides a way to evaluate the performance of the Localization component in urban environment. It uses the [Istanbul Open Dataset](https://autowarefoundation.github.io/autoware-documentation/main/datasets/#istanbul-open-dataset) for testing.



