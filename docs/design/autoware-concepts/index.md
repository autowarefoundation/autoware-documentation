# Autoware concepts

The concept of Autoware revolves around providing an open and flexible platform to accelerate the development and deployment of autonomous driving systems. Below is an extended explanation of its key principles.

## 1. Open Source Software

Autoware is an open source software framework for autonomous driving, licensed under the Apache 2.0 license. This permissive licensing model allows users to freely use, modify, and distribute the software, even for commercial purposes, while ensuring proper attribution. This provides the following benefits to the users and developers:

- **Transparency & Validation:** Open access to the source code allows researchers, developers, and industry experts to inspect, validate, and improve the software, ensuring robustness and reliability.
- **Collaboration & Innovation:** A shared development model encourages contributions from a global community, accelerating advancements in autonomous technology.
- **Interoperability & Standardization:** Autoware adheres to common standards and widely adopted environments, ensuring seamless integration with diverse platforms, sensors, and vehicle architectures.
- **Cost & Accessibility:** By eliminating proprietary software costs, Autoware lowers the barriers to entry for startups, researchers, and commercial developers, enabling broader adoption and experimentation.

## 2. Comprehensive Functionality

Autoware offers a complete suite of capabilities for autonomous driving system development including:

- **Key Components for AD Systems:** Autoware provides various algorithms for localization, obstacle detection, path planning, and vehicle control for users to choose depending on their needs.
- **Hardware Integration:** Autoware provides reference implementation and instructions for integrating with different sensors and vehicles.
- **Simulation Support:** Autoware works with simulation platforms to validate algorithms and scenarios before real-world deployment.
- **Tools:** Provides useful tools for development including sensor calibration, mapping, data creation, diagnostic, and scenario tests.

## 3. Microautonomy Architecture

Autoware is designed with a modular and flexible architecture, enabling seamless adaptation to a wide range of requirements and use cases. Its well-defined interface design facilitates efficient inter-component communication, allowing for the smooth integration of new features and functionalities.

For instance, users can replace the default object detection module with a custom neural network tailored for specific tasks, such as identifying construction cones in a work zone. This ability to swap or enhance individual modules without disrupting the rest of the system underscores Autoware’s flexibility and robustness.

At the core of the microautonomy architecture is its interface design, which is categorized into internal and external interfaces. Internal [**Component Interfaces**](../autoware-interfaces/components/index.md) connect components across different modules within Autoware, facilitating coordinated behavior. External interfaces called [**Autonomous Driving (AD) API**](../autoware-interfaces/ad-api/index.md) expose Autoware’s capabilities to external applications, such as infotainment systems and cloud services.

This architecture is made possible through collaboration with AWF partners, who both contribute to and benefit from the clearly defined and standardized interfaces.

![Overview](../autoware-architecture/image/autoware-architecture-overview.drawio.svg)

## 4. Core & Universe Package Management

Autoware consists of packages managed by the Autoware Foundation as well as open-source contributions outside the foundation.

The packages maintained by the Autoware Foundation (AWF) are referred to as **Autoware Core**, and maintained under the AWF GitHub organization. These packages are developed and released according to quality standards set by AWF, which include unit tests, integration tests, and real-world testing. The users are expected to use the Core packages as the base platform for their autonomous driving systems.

In contrast, **Autoware Universe** is a collection of OSS packages that are independently developed and maintained by external individuals, companies, and research institutions. These packages are owned and managed by their original authors, who follow their own criteria for code quality and functionality. Authors have the option to either directly merge their contributions into the Autoware Universe repository hosted by the Autoware Foundation or register their own repositories as part of the Universe repository list. Autoware Universe packages may be ported to Autoware Core if their functionality and maturity are recognized by the Autoware Foundation.

With this two-tier package management, the Autoware Foundation ensures a quality-assured base platform by Autoware Core, while also fostering a collaborative ecosystem where third-party developers can easily share their contributions with the community under Autoware Universe.

<!--
We comment out the following section for now, as it is not relevant to the new page outline.
We can add it back in system design page when we create it.

## Challenges

A grand challenge of the microautonomy architecture is to achieve real-time capability, which guarantees all the technology components activated in the system to predictably meet timing constraints (given deadlines). In general, it is difficult, if not impossible, to tightly estimate the worst-case execution times (WCETs) of components.

In addition, it is also difficult, if not impossible, to tightly estimate the end-to-end latency of components connected by a DAG. Autonomous driving systems based on the microautonomy architecture, therefore, must be designed to be fail-safe but not never-fail. We accept that the timing constraints may be violated (the given deadlines may be missed) as far as the overrun is taken into account. The overrun handlers are two-fold: (i) platform-defined and (ii) user-defined. The platform-defined handler is implemented as part of the platform by default, while the user-defined handler can overwrite it or add a new handler to the system. This is what we call “fail-safe” on a timely basis.

## Requirements and roadmap

Goals:

- All open-source
- Use case driven
- Real-time (predictable) framework with overrun handling
- Code quality -->
