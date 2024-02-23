# Metrics for Meta-info and Tech-info Charts

Meta-Info and Tech-Info charts are provided for each project to provide a high-level overview of the design and technologies used in each project. The metrics for the chart are defined on this page.

## Meta-Info Charts

- ### **Openness**

the adaptation of using open-source software.

| Value | Definition                                                                               |
| :---: | ---------------------------------------------------------------------------------------- |
|   0   | Complete proprietary software                                                            |
|   1   | Mostly proprietary software ( >80%) and few open source software ( <20%)                 |
|   2   | Majority are proprietary software ( >60%) and the others are open source ( <40%)         |
|   3   | Less than half are proprietary software ( >40%) and the majority are open source ( <60%) |
|   4   | Few proprietary software ( <20%) and mostly open source software (<80%)                  |
|   5   | Complete (100%) open source software.                                                    |

- ### **Vehicle Size**

the length of the size.

| Value | Definition        |
| :---: | ----------------- |
|   0   | Shorter than 1m   |
|   1   | Between 1m and 3m |
|   2   | Between 3m and 5m |
|   3   | Between 5m and 7m |
|   4   | Between 7m and 9m |
|   5   | Longer than 9m    |

- **Hardware Requirement**: the complexity of required computation hardware.

| Value | Definition                                                     |
| :---: | -------------------------------------------------------------- |
|   0   | Single Core CPU Only                                           |
|   1   | Multi-core CPU                                                 |
|   2   | Multi-core CPU and Customized ECU                              |
|   3   | Multi-core CPU and Single GPU                                  |
|   4   | Multi-core CPU and Multi GPU                                   |
|   5   | Multi-core CPU, Multi GPU, and Customized (Safety) Processors. |

- **Software Requirement**: the complexity of required software.

| Value | Definition                                                         |
| :---: | ------------------------------------------------------------------ |
|   0   | Embedded OS / Proprietary OS                                       |
|   1   | Linux (including ROS 1, Autoware.ai)                               |
|   2   | Linux + ROS 2 (including Autoware.auto)                            |
|   3   | Linux + ROS 2 + Autoware.core/universe                             |
|   4   | Linux + ROS 2 + Autoware.core/universe + Containerization          |
|   5   | Linux + ROS 2 + Autoware.core/universe + Containerization + SOAFEE |

- ### **Automation Level**

compatibility to the level of automation.

| Value | Definition |
| :---: | ---------- |
|   0   | Level 0    |
|   1   | Level 1    |
|   2   | Level 2    |
|   3   | Level 3    |
|   4   | Level 4    |
|   5   | Level 5    |

- **Sensors**: configuration of required sensors (unordered).

| Value | Definition                                     |
| :---: | ---------------------------------------------- |
|   0   | RGB Camera                                     |
|   1   | RGB Camera + GPS SNSS                          |
|   2   | RGB Camera + GPS SNSS + IMU                    |
|   3   | RGB Camera + GPS SNSS + IMU + 2D Lidar         |
|   4   | RGB Camera + GPS SNSS + IMU + 3D Lidar         |
|   5   | RGB Camera + GPS SNSS + IMU + Radar + 3D Lidar |

- ### **Maturity of the systems**

  the maturity of the complete system

| Value | Definition                                                                           |
| :---: | ------------------------------------------------------------------------------------ |
|   0   | Tested in closed/pre-defined region without other mobile objects                     |
|   1   | Tested by third parties in closed/pre-defined region without other mobile objects    |
|   2   | Tested in closed/pre-defined region along with other mobile objects                  |
|   3   | Tested by third parties in closed/pre-defined region along with other mobile objects |
|   4   | Tested in open regions, including other vehicles                                     |
|   5   | Tested by third parties in open regions, including other vehicles                    |

- ### **Availability**

  the availability to be acquired.

| Value | Definition                                          |
| :---: | --------------------------------------------------- |
|   0   | No, only one set.                                   |
|   1   | Multiple customized vehicles.                       |
|   2   | Off-the-shelf parts but build-by-yourself.          |
|   3   | Retrofit existing vehicles using off-the-self parts |
|   4   | Available for purchase but limited supply.          |
|   5   | Available for purchase but limited supply.          |

## Tech-Info Charts

- ### **Payload**

Design Payload.

| Value | Definition                |
| :---: | ------------------------- |
|   0   | < 100 KG                  |
|   1   | Between 100 kg and 500 Kg |
|   2   |                           |
|   3   |                           |
|   4   |                           |
|   5   |                           |

- **Cost**: Building cost excluding vehicle itself.

| Value | Definition              |
| :---: | ----------------------- |
|   0   | < 1K USD                |
|   1   | Between 1K and 5K USD   |
|   2   | Between 5K and 100K USD |
|   3   | Between 100K and 1M USD |
|   4   | Between 1M and 10M USD  |
|   5   | > 10M USD               |

- ### **Communicaiton**

Communication Capability.

| Value | Definition                                                   |
| :---: | ------------------------------------------------------------ |
|   0   | CAN                                                          |
|   1   | CAN + On-Vehicle Ethernet                                    |
|   2   | CAN + On-Vehicle Ethernet + DSRC/C-V2X Receiving             |
|   3   | CAN + On-Vehicle Ethernet + DSRC/C-V2X Receiving + V2I       |
|   4   | CAN + On-Vehicle Ethernet + DSRC/C-V2X Receiving + V2I + V2V |
|   5   | (Undefined)                                                  |

- ### **Vehicle Speed**

Tested Vehicle Speed.

| Value | Definition              |
| :---: | ----------------------- |
|   0   | < 5 KMH                 |
|   1   | Between 5 and 30 KMH    |
|   2   | Between 30 and 60 KMH   |
|   3   | Between 50 and 100 KMH  |
|   4   | Between 100 and 300 KMH |
|   5   | > 300KMH                |

- ### **Distance between Disengagement**

The average distance between disengagement.

| Value | Definition               |
| :---: | ------------------------ |
|   0   | < 100m                   |
|   1   | Between 100m and 1KM     |
|   2   | Between 1KM and 10KM     |
|   3   | Between 10KM and 100KM   |
|   4   | Between 100KM and 1000KM |
|   5   | > 1000KM                 |

- ### **Power Consumption**

Required power consumption for ECU, Sensors, and Actuators.

| Value | Definition              |
| :---: | ----------------------- |
|   0   | < 100W                  |
|   1   | Between 100W and 500W   |
|   2   | Between 500W and 1000W  |
|   3   | Between 1000W and 3000W |
|   4   | Between 3000W and 6000W |
|   5   | > 6000W                 |
