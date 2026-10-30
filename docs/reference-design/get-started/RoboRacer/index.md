<!-- cspell: ignore roboracer -->

# RoboRacer

## Overview

This document describes the hardware and software configurations of RoboRacer (formerly F1TENTH), a 1/10 scale autonomous racing car that can run Autoware. It serves as a guideline to select hardware and software components to build autonomous vehicles. This document is not a step-by-step instruction or a Q&A book. Where such material exists, this document links to it.

## Design overview

The chart compares RoboRacer with the LeoDrive bus of the Bus ODD project. The [metrics page](../../metrics.md) defines most of the axes. It has no definition for ODD.

{{ diagram("images/roboracer-meta-info-chart.svg", "RoboRacer and the LeoDrive bus score from 0 to 5 on each of the nine meta-info metrics.") }}

## Who should read

Read this page if you want to build an autonomous racing car or a low-cost platform to learn Autoware, and look for a place to start.

## References and resources

- RoboRacer website: [roboracer.ai](https://roboracer.ai/)
- RoboRacer build documentation: [RoboRacer - Build Documentation](https://f1tenth.readthedocs.io/en/main/)
- RoboRacer course material: [RoboRacer course kit](https://f1tenth-coursekit.readthedocs.io/en/latest/)
- Autoware on RoboRacer: [Autoware Installation on the RoboRacer](https://f1tenth.readthedocs.io/en/main/autoware/intro.html)

## Contributors

The [RoboRacer website](https://roboracer.ai/) has the complete list.

### RoboRacer core developers

- Hongrui (Billy) Zheng
- Johannes Betz

### Contributors

- Matthew O'Kelly
- Kim Luong
- Haoru Xue
- Joshua Whitley
- RoboRacer Lehigh and Rosa Zheng
- RoboRacer UCSD and Jack Silberman
- Jim Benson

### Community outreach

- Rahul Mangharam
- RoboRacer Clemson and Venkat Krovi
- Jaroslav Klapálek

## Targeted use cases

| Use Cases: |             Education             |              Racing               |                 PoC                 |             Open AD Kit             |   Commercial Services/Production    |
| :--------: | :-------------------------------: | :-------------------------------: | :---------------------------------: | :---------------------------------: | :---------------------------------: |
| RoboRacer  | <input type="checkbox" checked /> | <input type="checkbox" checked /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## ODD

|    ODD    |                Cargo                |                Bus                 |              RoboTaxi               |     Low Speed (Utility) Vehicle     |           Delivery Robot            |
| :-------: | :---------------------------------: | :--------------------------------: | :---------------------------------: | :---------------------------------: | :---------------------------------: |
| RoboRacer | <input type="checkbox" disabled  /> | <input type="checkbox" disabled /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## Automation level

| Automation Level |                 0                  |                 1                  |                 2                 |                  3                  |                  4                  |                  5                  |
| :--------------: | :--------------------------------: | :--------------------------------: | :-------------------------------: | :---------------------------------: | :---------------------------------: | :---------------------------------: |
|    RoboRacer     | <input type="checkbox" checked  /> | <input type="checkbox" checked  /> | <input type="checkbox" checked /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## System architecture

You can use RoboRacer for racing and for education. The simplest use case is to drive on a racing track without a collision with obstacles. You can also develop a SLAM algorithm that builds the map while the car drives, and then finish the loop in a short time.

![RoboRacer NX](https://raw.githubusercontent.com/f1tenth/f1tenth_doc/94efb99a2df0fdf29954501f02c8de6c50ae20f9/img/f1tenth_NX.png)

## Hardware configuration

### Chassis

- Traxxas Slash 4x4 Premium: [link to purchase](https://www.amainhobbies.com/traxxas-slash-4x4-ultimate-rtr-4wd-short-course-truck-orange-tra68077-4-orng/p858530)
- VESC speed controller: VESC 6 MK III

### Sensors

- LiDAR: choose one
  - Hokuyo UTM-30LX
  - Hokuyo 10LX
- Camera (optional):
  - ZED 2
  - Intel RealSense

### Computing platforms

- NVIDIA Jetson Xavier NX
- USB Hub (4 ports)
- (DEPRECATED) NVIDIA Jetson TX2

![RoboRacer](https://raw.githubusercontent.com/f1tenth/f1tenth_doc/94efb99a2df0fdf29954501f02c8de6c50ae20f9/getting_started/build_car/img/together/together_NX_00.JPG)

## Software configuration

The [Autoware installation guide for RoboRacer](https://f1tenth.readthedocs.io/en/main/autoware/intro.html) uses this software. It also has a record and replay demo: you build a map, record a trajectory while you drive the car by hand, and let Autoware follow the trajectory.

- Software on the car:
  - Operating system: Ubuntu 20.04
  - ROS: [ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
  - GPU driver: NVIDIA JetPack 5.1.1 (rev. 1)
  - Autoware: the [`galactic` branch](https://github.com/autowarefoundation/autoware/tree/galactic) of `autowarefoundation/autoware`, with the [`f1tenth_galactic` branch](https://github.com/autowarefoundation/autoware_universe/tree/f1tenth_galactic) of Autoware Universe
  - SLAM Toolbox to create and edit map: ros-galactic-slam-toolbox
- Tools:
  - Simulator: [RoboRacer simulator](https://github.com/f1tenth/f1tenth_gym)
  - Motor control: [VESC Tool](https://vesc-project.com/vesc_tool)
  - Version control tool: [vcstool](https://github.com/dirk-thomas/vcstool)

!!! note

    This integration is older than Autoware 2.0.0, and ROS 2 Galactic reached its end of life in 2022. Autoware 2.0.0 supports ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu 24.04. The RoboRacer driver stack now uses ROS 2 Humble.
