# Reference design

Autoware has been developed and deployed in several autonomous vehicle projects. These projects range from prototypes, research platforms, education platforms, racing vehicles, commercial vehicles and utility vehicles. The pages in this section elaborate their designs, including hardware and software, for these projects.

A Reference Design combines a Deployment Profile with a Reference Software Stack. The Deployment Profile describes the vehicle, the operating conditions and the use case. The Reference Software Stack lists the modules, the model versions and the parameters. The result is a documented system that others can reproduce.

The [Autoware Index](../installation/autoware/autoware-index.md) at [index.autoware.org](https://index.autoware.org) marks the entries that are a Reference Design. [Vision Pilot](https://github.com/autowarefoundation/vision_pilot) is one of them.

The get started pages provide a decision tree to find the reference design guideline that meets your needs. You select the guideline by technology maturity, environment, location, and vehicle size. To start, see [Find your reference design](get-started/find-your-reference-design.md).

Below are the lists of the design guidelines.

## Main documentation

<div class="grid cards" markdown>

- **Privately Owned Vehicles** Reference Design Documentation

  ***

  A modular design guideline for building a TRL-6 privately-owned vehicle using Autoware. This document defines how to choose sensors, ECUs, actuators and software stack, how to define the Operational Design Domain (ODD), and how to conduct evaluation and testing. But it does _not_ mandate a specific vehicle chassis. Covers a phased roadmap: from single-camera “Vision Pilot” (SAE Level 2+/2++) through multi-sensor configurations up to “Vision Drive” targeting Level 4 autonomy.

  [:octicons-arrow-right-24: Link to the docs](https://autowarefoundation.github.io/autoware.pov-reference-design-docs/main/)

- **Low Speed Autonomy** Reference Design Documentation

  ***

  A design guideline for building TRL-6 low-speed autonomous platforms (e.g. parking-lot robots, shuttles, small vehicles) using Autoware. Supports both indoor and outdoor use-cases, with flexible chassis and hardware configurations, adaptable ECUs, sensors and actuators. Includes guidance on software stack setup, middleware choices, optional custom workflows, and evaluation/testing procedures. Provides a solid starting point for small-scale autonomous mobility or robotics projects.

  [:octicons-arrow-right-24: Link to the docs](https://autowarefoundation.github.io/LSA-reference-design-docs/main/)

- **Off-Road Autonomy** Reference Design Documentation

  ***

  A design guideline for building TRL-6 off-road vehicles using Autoware. The guideline uses an end-to-end foundation model. It covers the ODD definition, the sensors and actuators, the software deployment on ECUs, and the evaluation and testing. It does not define a reference chassis. Many sections of this guideline are not written yet.

  [:octicons-arrow-right-24: Link to the docs](https://autowarefoundation.github.io/autoware.orv-reference-design-docs/main/)

</div>

## Racing robots

<div class="grid cards" markdown>

- **RoboRacer** Reference Design Documentation

  ***

  A design guideline for building TRL-6 low-speed autonomous racing platforms using Autoware. You can use RoboRacer for racing and education. The simplest use case is to drive on a racing track without a collision with obstacles. You can also develop a SLAM algorithm that builds the map while the car drives, and then finish the loop in a short time.

  [:octicons-arrow-right-24: Link to the docs](https://autowarefoundation.github.io/LSA-reference-design-docs/main/get-started/RoboRacer/)

- **GoKart** Reference Design Documentation

  ***

  A design guideline for building TRL-6 low-speed autonomous racing platforms using Autoware. You can use the GoKart for racing and education. The simplest use case is to drive on a racing track without a collision with obstacles. You can also develop a SLAM algorithm that builds the map while the kart drives, and then finish the loop in a short time.

  [:octicons-arrow-right-24: Link to the docs](https://autowarefoundation.github.io/LSA-reference-design-docs/main/get-started/GoKart/)

</div>

## Other example builds

- [ITRI Bus](itri-bus/index.md): a bus for public roads from the Bus ODD project
- [AutoSDV](get-started/AutoSDV/index.md): an affordable 1/10 scale platform for education and research
- [Autoware delivery vehicle](get-started/Delivery/index.md): a concept for a delivery vehicle on a private campus

## Reference hardware

[:fa-cl-s fa-circle-arrow-right: Reference Hardware Section](reference-hw/index.md){ .md-button }
