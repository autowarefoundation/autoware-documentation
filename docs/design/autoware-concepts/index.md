# Autoware concepts

Autoware is the world’s first open-source software for autonomous driving systems. Autoware provides value for both technology developers and service operators. The technology developers of autonomous driving systems can create new components based on Autoware. The service operators of autonomous driving systems, on the other hand, can select appropriate technology components with Autoware. This is enabled by the microautonomy architecture that modularizes its software stack into the core and universe subsystems (modules).

## Microautonomy architecture

Autoware uses a [pipeline architecture](http://www.cs.sjsu.edu/~pearce/modules/patterns/distArch/pipeline.htm) to enable the development of autonomous driving systems. The pipeline architecture used in Autoware consists of components similar to [three-layer-architecture](http://www.flownet.com/gat/papers/tla.pdf). And they run in parallel. There are 2 main modules: the Core and the Universe. The components in these modules are designed to be extensible and reusable. And we call it microautonomy architecture.

![core-and-universe.svg](core-and-universe.svg)

### The Core module

The Core module contains basic runtimes and technology components that satisfy the basic functionality and capability of sensing, computing, and actuation required for autonomous driving systems. AWF develops and maintains the Core module with their architects and leading members through their working groups. Anyone can contribute to the Core but the PR(Pull Request) acceptance criteria is more strict compared to the Universe.

### The Universe module

The Universe modules are extensions to the Core module that can be provided by the technology developers to enhance the functionality and capability of sensing, computing, and actuation. AWF provides the base Universe module to extend from. A key feature of the microautonomy architecture is that the Universe modules can be contributed to by any organization and individual. That is, you can even create your Universe and make it available for the Autoware community and ecosystem. AWF is responsible for quality control of the Universe modules through their development process. As a result, there are multiple types of the Universe modules - some are verified and validated by AWF and others are not. It is up to the users of Autoware which Universe modules are selected and integrated to build their end applications.

## Interface design

The interface design is the most essential piece of the microautonomy architecture, which is classified into internal and external interfaces. The component interface is designed for the components in a Universe module to communicate with those in other modules, including the Core module, within Autoware internally. The AD(Autonomous Driving) API, on the other hand, is designed for the applications of Autoware to access the technology components in the Core and Universe modules of Autoware externally. Designing solid interfaces, the microautonomy architecture is made possible with AWF's partners, and at the same time is made feasible for the partners.

## Challenges

A grand challenge of the microautonomy architecture is to achieve real-time capability, which guarantees all the technology components activated in the system to predictably meet timing constraints (given deadlines). In general, it is difficult, if not impossible, to tightly estimate the worst-case execution times (WCETs) of components.

In addition, it is also difficult, if not impossible, to tightly estimate the end-to-end latency of components connected by a DAG. Autonomous driving systems based on the microautonomy architecture, therefore, must be designed to be fail-safe but not never-fail. We accept that the timing constraints may be violated (the given deadlines may be missed) as far as the overrun is taken into account. The overrun handlers are two-fold: (i) platform-defined and (ii) user-defined. The platform-defined handler is implemented as part of the platform by default, while the user-defined handler can overwrite it or add a new handler to the system. This is what we call “fail-safe” on a timely basis.

## Requirements and roadmap

Goals:

- All open-source
- Use case driven
- Real-time (predictable) framework with overrun handling
- Code quality
