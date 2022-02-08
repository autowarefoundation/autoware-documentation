# Introduction

Autoware is the world’s first open-source software for autonomous driving systems. The value of Autoware is two-fold. The technology developers of autonomous driving systems can create new components based on Autoware. The service operators of autonomous driving systems, on the other hand, can select appropriate technology components with Autoware. This is enabled by the microautonomy architecture that modularizes its software stack into the core and universe subsystems (modules).

Note that a module and a component are orthogonal to each other. In general, a module is a unit of functions that can be loaded and unloaded, while a component is a unit of functions that is a part of the system. So the module can be developed using multiple components and the component can be developed using multiple modules.

The core module contains basic runtimes and technology components that satisfy the basic functionality and capability of sensing, computing, and actuation required for autonomous driving systems. The universe modules are extensions to the core module that can be provided by the technology developers to enhance the functionality and capability of sensing, computing, and actuation. In general, AWF develops and maintains the core module with their architects and leading members through their working groups. Note that AWF may also provide their own universe module through their projects. A key feature of the microautonomy architecture is that the universe modules can be contributed to by any organization and individual. That is, you can even create your universe and make it available for the Autoware community and ecosystem. AWF is responsible for quality control of the universe modules through their development process. As a result, there are multiple types of the universe modules - some are verified and validated by AWF and others are not. It is up to the users of Autoware which universe modules are selected and integrated to build their end applications.

The interface design is the most essential piece of the microautonomy architecture, which is classified into internal and external interfaces. The Component Interface is designed for the technology components in a universe module to communicate with those in other modules, including the core module, within Autoware internally. The AD API, on the other hand, is designed for the applications of Autoware to access the technology components in the core and universe modules of Autoware externally. Designing solid interfaces, the microautonomy architecture is made possible with our partners, and at the same time is made feasible for our partners.

A grand challenge of the microautonomy architecture is to achieve real-time capability, which guarantees all the technology components activated in the system to predictably meet timing constraints (given deadlines). In general, it is difficult, if not impossible, to tightly estimate the worst-case execution times (WCETs) of components. In addition, it is also difficult, if not impossible, to tightly estimate the end-to-end latency of components connected by a DAG. Autonomous driving systems based on the microautonomy architecture, therefore, must be designed to be fail-safe but not never-fail. We accept that the timing constraints may be violated (the given deadlines may be missed) as far as the overrun is taken into account. The overrun handlers are two-fold: (i) platform-defined and (ii) user-defined. The platform-defined handler is implemented as part of the platform by default, while the user-defined handler can overwrite it or add a new handler to the system. This is what we call “fail-safe” on a timely basis.

Requirements and Roadmap

Goals:
- All open-source.
- Use case driven.
- Real-time (predictable) framework with overrun handling.
- Code quality.

Non-goals:
- Accuracy of components

# Architecture

Core and Universe.

Autoware provides the runtimes and technology components by open-source software. The runtimes are based on the Robot Operating System (ROS). The technology components are provided by contributors, which include, but are not limited to:
 - Sensing
   - Camera Component
   - LiDAR Component
   - RADAR Component
   - GNSS Component
 - Computing
   - Localization Component
   - Perception Component
   - Planning Component
   - Control Component
   - Logging Component
   - System Monitoring Component
 - Actuation
   - DBW Component
 - Tools
   - Simulator Component
   - Mapping Component
   - Remote Component
   - ML Component
   - Annotation Component
   - Calibration Component


# Concern, Assumption, and Limitation

The downside of the microautonomy architecture is that the computational performance of end applications is sacrificed due to its data path overhead attributed to functional modularity. In other words, the trade-off characteristic of the microautonomy architecture exists between computational performance and functional modularity. This trade-off problem can be solved technically by introducing real-time capability. This is because autonomous driving systems are not really designed to be real-fast, that is, low-latency computing is nice-to-have but not must-have. The must-have feature for autonomous driving systems is that the latency of computing is predictable, that is, the systems are real-time. As a whole, we can compromise computational performance to an extent that is predictable enough to meet the given timing constraints of autonomous driving systems, often referred to as deadlines of computation.

# Design

!!! warning

    Under Construction

## Autoware concepts

## Component interfaces

## AD API

## Configuration management

# Conclusion

