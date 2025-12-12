# Autoware 2.0 Architecture

!!! warning

    Under Construction

## Generator - Selector architecture: conceptual overview

![generator-selector-simple.svg](images/generator-selector-simple.svg)

Traditional autonomous driving follows a fixed pipeline:

> **Sensing → Perception → Localization → Planning → Trajectory**

This works well for rule-based planners, but newer approaches like E2E or diffusion models don’t fit neatly into that structure.
They may skip or replace parts of the pipeline, making integration difficult.

To support both classical and modern approaches, we **abstracted away the front half of the pipeline**.

### Generators: Flexible trajectory producers

A **Generator** is any module that outputs trajectories. It could be one or more:

- rule-based or optimization planners using perception and maps
- E2E models using raw sensor input
- learned or sampling-based planners

Generators can reuse Autoware’s sensing, perception, localization, and control. Or bypass them.
Multiple generators can run in parallel.

### Selector: Safety + final choice

The **Selector** receives candidate trajectories and:

- **Safety-checks** them (e.g., rule compliance, drivable area)
- **Ranks and selects** the best one based on context or driving policies

!!! tip "This enables:"

    * Seamless integration of both robotics-based and E2E planners
    * Safe use of black-box models through explicit checks
    * Flexible experimentation with new planning methods
    * Robust decision-making by comparing multiple trajectory proposals
