<!-- cspell: ignore gokart purepursuit Adafruit -->

# Go-Kart

## Overview

This document describes the hardware and software configurations of the autonomous Go-Kart, which uses Autoware for autonomous driving. It serves as a guideline to select hardware and software components to build autonomous vehicles. This document is not a step-by-step instruction or a Q&A book. Where such material exists, this document links to it.

## Design overview

The charts give a high-level overview of the design. The [metrics page](../../metrics.md) defines the values.

{{ diagram("images/gokart-meta-info-chart.svg", "The Go-Kart design scores from 0 to 5 on each of the nine meta-info metrics.") }}

{{ diagram("images/gokart-tech-info-chart.svg", "The Go-Kart design scores from 0 to 5 on each of the eleven tech-info metrics.") }}

## Who should read

Read this page if you want to build a small autonomous vehicle and look for a place to start.

## References and resources

- Complete Go-Kart with Autoware documents: [Go-Kart documentation](https://go-kart-upenn.readthedocs.io/en/latest/)
- ROS 2 Foxy setup and autonomous code: [`ros2_foxy_purepursuit` branch](https://github.com/mlab-upenn/gokart-sensor/tree/ros2_foxy_purepursuit)
- ROS 2 Humble setup and autonomous code: [`ros2_humble_purepursuit` branch](https://github.com/mlab-upenn/gokart-sensor/tree/ros2_humble_purepursuit)
- STM32 Nucleo controller code: [`STM32 Control` folder](https://github.com/mlab-upenn/gokart-mechatronics/tree/main/STM32%20Control)

!!! note

    The Go-Kart code uses ROS 2 Foxy or ROS 2 Humble. Autoware 2.0.0 supports ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu 24.04. To build Autoware for the Go-Kart, see [Source installation](../../../installation/autoware/source-installation.md).

## Contributors

- Autoware Center of Excellence (under Prof. Rahul Mangharam)

## Targeted use cases

| Use Cases: |             Education             |              Racing               |                PoC                 |             Open AD Kit             |   Commercial Services/Production    |
| :--------: | :-------------------------------: | :-------------------------------: | :--------------------------------: | :---------------------------------: | :---------------------------------: |
|  Go-Kart   | <input type="checkbox" checked /> | <input type="checkbox" checked /> | <input type="checkbox" checked  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## ODD

|   ODD   |                Cargo                |                Bus                 |              RoboTaxi               |     Low Speed (Utility) Vehicle     |           Delivery Robot            |
| :-----: | :---------------------------------: | :--------------------------------: | :---------------------------------: | :---------------------------------: | :---------------------------------: |
| Go-Kart | <input type="checkbox" disabled  /> | <input type="checkbox" disabled /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## Automation level

| Automation Level |                 0                  |                 1                  |                 2                 |                 3                  |                  4                  |                  5                  |
| :--------------: | :--------------------------------: | :--------------------------------: | :-------------------------------: | :--------------------------------: | :---------------------------------: | :---------------------------------: |
|     Go-Kart      | <input type="checkbox" checked  /> | <input type="checkbox" checked  /> | <input type="checkbox" checked /> | <input type="checkbox" checked  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## System architecture

The autonomous Go-Kart is a one-third scale autonomous vehicle for development and prototyping. You can use it to develop and test software components of Autoware. The photo shows the drive-by-wire parts: the drive motor with throttle-by-wire, the upper and lower steer-by-wire, the brake-by-wire, and the back shelf with the main controller.

![The Go-Kart with its drive-by-wire parts](https://raw.githubusercontent.com/mlab-upenn/Go-Kart/45e3f067209b3805c7d544e4d88beb27d8ee5934/docs/source/avev_gokart.png)

## Hardware configuration

### Chassis

- TopKart: [bill of materials and purchase links](https://go-kart-upenn.readthedocs.io/en/latest/Bom.html)

### Sensors

- LiDAR: Ouster OS1 and Velodyne Velarray M1600
- Camera: OAK-D camera
- GNSS: RTK-GNSS (Septentrio receiver)
- IMU: Adafruit 9-DoF IMU

### Computing platforms

- x86 laptop with NVIDIA GPU
- STM32 Nucleo microcontroller boards (Nucleo L432KC and Nucleo F439ZI)

## Software configuration

The diagram shows how the sensors and the microcontrollers connect to the computer.

{{ diagram("images/gokart-compute-sensing.svg", "The Go-Kart sensors and microcontrollers connect to the computer over Ethernet, USB, and USB-TTL.") }}
