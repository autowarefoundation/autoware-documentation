# Architecture roadmap

This roadmap describes how the Autoware project plans to evolve from a traditional robotics-based autonomous driving stack to a data-centric, AI-based stack. The target is a stack that can power safe and globally scalable end-to-end (E2E) autonomous driving for many use cases. The roadmap called the robotics stack "Autoware 1.0" and the target stack "Autoware 2.0".

!!! info "Status in Autoware 2.0.0"

    Autoware 2.0.0 contains the generator-selector framework and Step 1 of the plan: you can select the Diffusion Planner, a learned planner, at launch. The rule-based stack is still the default. The other steps are plans. The [Architecture 2.0](../index.md) page describes what 2.0.0 does today.

## Overview

The autonomous driving industry developed quickly because of advances in artificial intelligence (AI).

Neural networks changed self-driving technology. Hand-coded software modules that follow a traditional robotics paradigm became components that machine learning powers. With these data-driven approaches, developers of self-driving technology can scale autonomy and deploy it around the world.

A recent breakthrough is end-to-end autonomous driving. In this approach, a single monolithic neural network maps the sensing data to safe driving trajectories. The network learns the full driving task.

This approach is promising, but safety, explainability, and model training need attention.

These pages give the framework that the Autoware project uses to adopt end-to-end autonomous driving.

<div class="grid cards aw-cards" markdown>

- :fa-cl-s fa-layer-group: **[Autonomous driving stack architecture](autonomous-driving-stack-architecture.md)**

  The steps from the robotics stack to learned planning, learned perception, learned driving, and a learned hybrid stack.

- :fa-cl-s fa-code-branch: **[Detailed architectural interface](detailed-architectural-interface.md)**

  The requirements of the new architecture and the generator-selector framework.

- :fa-cl-s fa-scale-balanced: **[Assessment of safety and benchmarks](assessment-safety-and-benchmarks.md)**

  The tools that check and measure planners and learned models today.

- :fa-cl-s fa-database: **[Data, MLOps, and training](data-mlops-and-training.md)**

  How the AI models of Autoware are trained, versioned, and distributed today.

- :fa-cl-s fa-flag-checkered: **[Development roadmap](development-roadmap.md)**

  The status of the generator-selector migration and where the work is discussed.

- :fa-cl-s fa-cube: **[Neural world simulation](neural-world-simulation.md)**

  The current work on simulation with learned scene models.

</div>
