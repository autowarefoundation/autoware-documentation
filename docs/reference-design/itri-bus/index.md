# ITRI Bus reference design

## Overview

This document describes the bus of the Industrial Technology Research Institute (ITRI) in Taiwan, which uses Autoware for autonomous driving. ITRI is a member of the Autoware Foundation. This document serves as a guideline to select hardware and software components to build autonomous vehicles. This document is not a step-by-step instruction or a Q&A book. Where such material exists, this document links to it.

## Design overview

The charts give a high-level overview of the design. The [metrics page](../metrics.md) defines the values.

{{ diagram("images/itri-bus-meta-info-chart.svg", "The ITRI bus design scores from 0 to 5 on each of the eight meta-info metrics.") }}

{{ diagram("images/itri-bus-tech-info-chart.svg", "The ITRI bus design scores from 0 to 5 on each of the six tech-info metrics.") }}

## The Bus ODD project

The ITRI bus is one of the two vehicles of the Bus ODD project of the Autoware Foundation. The project chose the Autoware Universe architecture for its implementation in 2022. It worked on two fronts:

- ITRI worked toward a bus service demonstration on public roads in Taiwan.
- Leo Drive worked toward a service on a university campus.

In 2023, ITRI ran the final integration tests on its public road bus. Then it recorded the Bus ODD demo video in Taiwan. The Autoware simulation work also tested the scenario simulator and AWSIM on ITRI scenes, with Lanelet2-based scenarios that the ODD working group prepared.

## References and resources

- [Past, present and the future of Autoware](https://autoware.org/past-present-and-the-future-of-autoware/): the plan of the Bus ODD project
- [Autoware Technical Steering Committee, September 2023](https://autoware.org/monthly-technical-steering-committee-update-september-2023/): the integration tests of the ITRI bus
- [Autoware Technical Steering Committee, October 2023](https://autoware.org/autoware-technical-steering-committee-october-2023/): the Bus ODD demo recording in Taiwan
- [Autoware Technical Steering Committee, November 2023](https://autoware.org/autoware-technical-steering-committee-update-november-2023-autoware-foundation/): scenario tests on ITRI scenes
- [Vehicle interface](../../tutorials/integrating-autoware/creating-vehicle-interface-package/vehicle-interface.md): a tutorial that explains the inputs and outputs of a vehicle interface
- [MORAI Sim: Drive](../../demos/digital-twin-demos/MORAI_Sim-tutorial.md): a simulator page that links to a tutorial for the ITRI bus ODD

## Targeted use cases

| Use Cases: |             Education             |               Racing               |                 PoC                 |             Open AD Kit             |   Commercial Services/Production    |
| :--------: | :-------------------------------: | :--------------------------------: | :---------------------------------: | :---------------------------------: | :---------------------------------: |
|  ITRI Bus  | <input type="checkbox" checked /> | <input type="checkbox" disabled /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## ODD

|   ODD    |                Cargo                |                Bus                |              RoboTaxi               |     Low Speed (Utility) Vehicle     |           Delivery Robot            |
| :------: | :---------------------------------: | :-------------------------------: | :---------------------------------: | :---------------------------------: | :---------------------------------: |
| ITRI Bus | <input type="checkbox" disabled  /> | <input type="checkbox" checked /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## Automation level

| Automation Level |                 0                  |                 1                  |                 2                 |                  3                  |                  4                  |                  5                  |
| :--------------: | :--------------------------------: | :--------------------------------: | :-------------------------------: | :---------------------------------: | :---------------------------------: | :---------------------------------: |
|     ITRI Bus     | <input type="checkbox" checked  /> | <input type="checkbox" checked  /> | <input type="checkbox" checked /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> | <input type="checkbox" disabled  /> |

## System architecture

The Bus ODD project uses the Autoware Universe architecture. For the components of this architecture, see [Architecture 1.0](../../design/autoware-architecture-v1/index.md).

## Hardware and software configuration

ITRI has not published the chassis, the sensors, the computing platform, or the software configuration of the bus in this documentation. For the sensors of the other Bus ODD vehicle, see the [Leo Drive - ISUZU sensor data](../../datasets/index.md#leo-drive-isuzu-sensor-data) dataset. For hardware that the community tested with Autoware, see the [reference hardware](../reference-hw/index.md) list.
