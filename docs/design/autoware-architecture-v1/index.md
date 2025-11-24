# Architecture overview

This page provides a high-level overview of the **Autoware architecture**, which is built on two complementary principles:

- the **Microautonomy architecture**, defining _how Autoware is constructed_ as a set of small, composable modules
- the **Generator–Selector architecture**, defining _how Autoware makes driving decisions_ using those modules

Together, these principles enable Autoware to support a wide range of autonomous driving approaches—from classical robotics pipelines to modern learning-based planners—within a unified and consistent system.

## Microautonomy architecture (structural foundation)

Autoware follows a **modular and composable [microautonomy architecture](../autoware-concepts/index.md#microautonomy-architecture-conceptual-overview)**.
In this paradigm, the autonomy stack is built from many small, replaceable components, each with clear inputs and outputs.
This ensures that:

- [components](#autoware-components) can be independently developed, swapped, or extended
- system integration remains stable even as modules evolve
- different vehicles and algorithms can share the same architectural framework

The microautonomy architecture defines _what the building blocks are_ and _how they connect_.

!!! note

    These building blocks include the domain-specific [components](#autoware-components) listed later on this page: sensing, perception, localization, map, planning, control, and the vehicle interface.

## Generator–Selector architecture (behavioral execution model)

While microautonomy defines the _components_, the **[Generator–Selector architecture](../autoware-concepts/index.md#generator-selector-architecture-conceptual-overview)** defines the _behavioral loop_ that produces motion commands.

Autoware’s runtime decision-making is organized around two key roles:

- **Generators**, which propose candidate trajectories
- **The Selector**, which validates and chooses the final trajectory

This separation allows diverse planning strategies to coexist safely and consistently.

## High-Level architecture design

<img src="../autoware-concepts/images/generator-selector-simple.svg" alt="Overview" />

Autoware’s autonomy loop operates through **multiple trajectory Generators** feeding into a **Selector** that determines the final motion command.

### Generators: Flexible trajectory producers

Generators are modular components that output possible trajectories for the vehicle.
They may use:

- classical robotics-style pipelines (sensing → perception → localization → planning)
- optimization or sampling-based methods
- end-to-end or neural network approaches
- specialized domain planners for unique environments

Because all Generators follow the same interface pattern, they can be freely added, replaced, or run in parallel.

!!! info

    Generators may reuse Autoware’s domain components such as: sensing, perception, maps, localization, and control—depending on their design.

### Selector: Safety and final decision

The Selector receives all candidate trajectories and performs two primary functions:

#### Safety gate

Each trajectory must pass checks for:

- drivable area compliance
- adherence to traffic rules
- obstacle avoidance
- system-level constraints

This ensures safety even when using black-box or machine-learned Generators.

#### Ranking & selection

Among the valid trajectories, the Selector chooses the one that best satisfies the driving policy using criteria such as:

- safety margin
- comfort
- progress and efficiency
- scenario-specific behavior rules

The selected trajectory is then sent to the control component for execution.

## Autoware components

Autoware’s microautonomy architecture organizes functionality into **domain-specific components**.
Generators rely on these components as needed.

The primary components include:

- [Sensing](components/sensing/index.md)
- [Map](components/map/index.md)
- [Localization](components/localization/index.md)
- [Perception](components/perception/index.md)
- [Planning](components/planning/index.md)
- [Control](components/control/index.md)
- [Vehicle Interface](components/vehicle/index.md)

These components define responsibilities and interfaces that ensure consistent behavior across the system.
Developers can replace or extend individual components without altering the architecture as a whole.

## Node diagram

A diagram of Autoware’s nodes in the default configuration is available on the [Node diagram](node-diagram/index.md) page.
Configurations vary depending on which Generators and components are enabled and on the vehicle and sensor setup.

!!! note

    Detailed component and node-level documentation is available in the [Autoware Universe documentation](https://autowarefoundation.github.io/autoware_universe/main/).
