# Architecture Overview

## Introduction

This page describes the architecture of Autoware. The current Autoware is defined to be a layered architecture that clarifies each module's role and simplifies the interface between them. By doing so:

- Autoware's internal processing becomes more transparent
- Collaborative development is made easier because of the reduced interdependency between modules
- Users can easily replace an existing module (e.g. localization) with their own software component by simply wrapping their software to fit in with Autoware's interface

Note that the initial focus of this architecture design was solely on driving capability, and so the following features were left as future work:

- Fail safe
- HMI
- Real-time processing
- Redundant system
- State monitoring system

## High-level Architecture Design

![Overview](image/AutowareArchitectureOverview.drawio.svg)

This architecture consists of the following six stacks. Each of these design pages contains a more detailed set of requirements and use cases specific to that stack:

- [Sensing Design](Sensing/index.md)
- [Localization Design](Localization/index.md)
- [Perception Design](Perception/index.md)
- [Planning Design](Planning/index.md)
- [Control Design](Control/index.md)
- [VehicleInterface Design](Vehicle/index.md)
- [Map Design](Map/index.md)

## Node Diagram

The Autoware node diagram with the default configuration is described at the page: [node-diagram](node-diagram/index.md).

Note that Autoware configurations are scalable / selectable and will vary depending on the environment and required use cases.

## References

- [The architecture presentation given to the AWF Technical Steering Committee, March 2020](https://discourse.ros.org/uploads/short-url/woUU7TGLPXFCTJLtht11rJ0SqCL.pdf)
