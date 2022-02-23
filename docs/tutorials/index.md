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

### Rosbag replay simulation

Rosbag replay simulation is a simulation type that uses Rosbag data and tests mainly perception modules.
Sometimes it is used for endurance tests by repeatedly playing back the data.

### Digital twin simulation

Digital twin simulation is a simulation type that produces realistic data and simulate almost the entire system.
