# Architecture overview

This page describes the architecture of Autoware.

## Introduction

The current Autoware is defined to be a layered architecture that clarifies each module's role and simplifies the interface between them. By doing so:

- Autoware's internal processing becomes more transparent.
- Collaborative development is made easier because of the reduced interdependency between modules.
- Users can easily replace an existing module (e.g. localization) with their own software component by simply wrapping their software to fit in with Autoware's interface.

Note that the initial focus of this architecture design was solely on driving capability, and so the following features were left as future work:

- Fail safe
- Human Machine Interface
- Real-time processing
- Redundant system
- State monitoring system

## High-level architecture design

![Overview](image/autoware-architecture-overview.drawio.svg)

Autoware's architecture consists of the following seven stacks. Each linked page contains a more detailed set of requirements and use cases specific to that stack:

- [Sensing design](sensing/index.md)
- [Map design](map/index.md)
- [Localization design](localization/index.md)
- [Perception design](perception/index.md)
- [Planning design](planning/index.md)
- [Control design](control/index.md)
- [Vehicle Interface design](vehicle/index.md)

## Node diagram

A diagram showing Autoware's nodes in the default configuration can be found on the [Node diagram](node-diagram/index.md) page. Detailed documents for each node are available in the [Autoware Universe docs](https://autowarefoundation.github.io/autoware.universe/main/).

Note that Autoware configurations are scalable / selectable and will vary depending on the environment and required use cases.

## References

- [The architecture presentation given to the AWF Technical Steering Committee, March 2020](https://discourse.ros.org/uploads/short-url/woUU7TGLPXFCTJLtht11rJ0SqCL.pdf)
