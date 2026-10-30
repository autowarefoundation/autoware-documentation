# Autoware concepts

The concept of Autoware revolves around providing an open and flexible platform to accelerate the development and deployment of autonomous driving systems.
This page explains its two key ideas: the microautonomy architecture and the Core and Universe repository model.

## Microautonomy architecture

**Microautonomy architecture** is the way Autoware breaks down "autonomous driving" into many small, replaceable capabilities rather than one monolithic stack.
Each capability (for example, object detection, behavior planning, or lane-level routing) is a module with clear inputs and outputs, so systems can be composed like building blocks for different vehicles and use cases.

!!! question "What is _microautonomy architecture_?"

    In short, it is a **component-based autonomy design** where driving behaviors are built by _composing many small autonomy modules_ instead of relying on a single, fixed pipeline.
    This makes it easy to mix, match, and upgrade parts without rewriting the whole system.

Autoware's modules are connected through well-defined interfaces, which allows you to **swap or extend individual components** while keeping the rest of the system intact.
For example, you can replace the default object detection with a custom neural network specialized in construction cones, and the downstream tracking, planning, and control modules still work as before.

<div class="aw-steps" markdown>

1. **Start with the default perception pipeline.**
2. **Plug in a dedicated detector** for special targets, for example cones or forklifts.
3. **Keep the same planner and controller.** They see "objects", regardless of how they were detected.

</div>

At a high level, these interfaces come in two flavors:

<div class="grid cards aw-cards aw-cards--small" markdown>

- :fa-cl-s fa-diagram-project: **[Internal component interfaces](../autoware-architecture-v1/interfaces/components/index.md)**

  Connect modules inside Autoware, for example perception to planning to control.

- :fa-cl-s fa-plug: **[External AD APIs](../autoware-architecture-v1/interfaces/ad-api/index.md)**

  Expose Autoware's capabilities to the outside world, for example fleet management, cloud services, or infotainment.

</div>

### Why this matters for developers and partners

<div class="aw-features aw-features--small" markdown>

- :fa-cl-s fa-recycle: **Reuse**

  You can reuse core Autoware modules and only customize what is unique to your product.

- :fa-cl-s fa-arrows-rotate: **Evolve step by step**

  You can evolve your stack incrementally and swap one module at a time.

- :fa-cl-s fa-handshake: **Collaborate**

  Partners can collaborate around shared, stable interfaces and contribute components that plug into a common ecosystem.

</div>

## Core and Universe repository model

Autoware's software ecosystem is organized in layers: **Autoware Core**, **Autoware Universe**, and the community packages in the **Autoware Index**.
Together, they balance **quality assurance** with **community-driven innovation**.

<div class="aw-layers" role="img" aria-label="Three nested layers. Autoware Core is in the center, Autoware Universe is around it, and the Autoware Index is around both.">
  <div class="aw-layers__layer aw-layers__layer--index">
    <p class="aw-layers__name">Autoware Index</p>
    <p class="aw-layers__text">Community packages in their own repositories</p>
    <div class="aw-layers__layer aw-layers__layer--universe">
      <p class="aw-layers__name">Autoware Universe</p>
      <p class="aw-layers__text">Community innovation, maintained by the authors</p>
      <div class="aw-layers__layer aw-layers__layer--core">
        <p class="aw-layers__name">Autoware Core</p>
        <p class="aw-layers__text">Quality-assured base, maintained by the AWF</p>
      </div>
    </div>
  </div>
</div>

Dependencies point inward: Universe depends on Core, and Core never depends on Universe.

<div class="aw-editions" markdown>

<div class="aw-edition aw-edition--featured" markdown>

<span class="aw-edition__badge">Quality-assured base</span>

### Autoware Core

[Autoware Core](https://github.com/autowarefoundation/autoware_core) contains the foundational packages maintained by the Autoware Foundation (AWF).
These packages follow strict development standards: unit tests, a coverage gate, a stricter static analysis profile, and CI on every supported ROS distribution.
Core serves as the **stable platform** that users can rely on for building autonomous driving systems.

- A vetted, reliable foundation
- Consistent APIs and behavior
- Maintained, tested, and versioned releases

</div>

<div class="aw-edition" markdown>

<span class="aw-edition__badge">Community innovation</span>

### Autoware Universe

[Autoware Universe](https://github.com/autowarefoundation/autoware_universe) is a broader collection of open-source packages contributed by individuals, companies, and research groups.
These packages are owned and maintained by their original authors, who set their own quality and development practices.

- Rapid prototyping and experimentation
- A place for sharing specialized modules
- An entry point for contributions from the global community

</div>

</div>

Contributions can take two forms:

- Merged directly into the Universe repository hosted by the AWF.
- Hosted in their own repositories and listed in the [Autoware Index](../../installation/autoware/autoware-index.md).

Universe acts as a **sandbox for experimentation**, allowing new ideas, algorithms, and hardware adaptations to be shared quickly.

### From Universe to Core

Promising packages from Autoware Universe can be **adopted into Autoware Core** when they demonstrate sufficient maturity, stability, and usefulness.
This creates a natural pipeline:

<div class="aw-pipeline" markdown>

- :fa-cl-s fa-lightbulb: <span>**Innovation** in Universe</span>
- :fa-cl-s fa-ruler-combined: <span>**Standardization** to the Core requirements</span>
- :fa-cl-s fa-industry: <span>**Production** use from Core</span>

</div>

## Learn more

<div class="grid cards aw-cards" markdown>

- :fa-cl-s fa-list-check: **[What belongs in Autoware Core](core-package-inclusion-criteria.md)**

  The requirements that a package must meet to move into Core.

- :fa-cl-s fa-sitemap: **[Repository structure](../repository-structure.md)**

  The repositories of Autoware and how they depend on each other.

- :fa-cl-s fa-code-branch: **[Open source philosophy](../../contributing/open-source-philosophy.md)**

  Why Autoware is developed in the open, and how.

- :fa-cl-s fa-car-side: **[System capabilities](../autoware-system-capabilities.md)**

  What an Autoware-based system can do.

</div>
