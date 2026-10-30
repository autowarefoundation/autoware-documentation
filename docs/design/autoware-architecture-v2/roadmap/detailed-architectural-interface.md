# Detailed architectural interface

## Requirements and considerations

To bring end-to-end (E2E) autonomous driving technologies into the Autoware open-source project, we set these high-level software design requirements for the new Autoware E2E architecture and its software interfaces.

### 1. Define a robust, consistent interface across evolutionary steps

The [Autonomous driving stack architecture](autonomous-driving-stack-architecture.md) page describes the evolutionary steps that we plan to take towards end-to-end autonomous driving.

{{ diagram("media/detailed_architecture_figure1.svg", "Figure 1: The stepwise technology evolution.") }}

We aim to avoid disruptive changes to core interfaces at each stage. This ensures users can adopt new End-to-End AI modules incrementally, without requiring major rewrites. Moreover, some users may prefer to retain classical rule-based planners for specific scenarios (for example, deterministic planning). By preserving interface consistency, we allow these users to easily switch between End-to-End AI and traditional approaches as needed.

### 2. Introduce a framework for minimum safety guarantees

A major challenge with End-to-End AI models is their black-box nature, making it difficult to verify whether the generated trajectories are safe and valid. To address this, the architecture must incorporate a post-processing phase that enforces safety constraints on neural network outputs.

These safety requirements may vary across users and applications. Therefore, the framework should support easy customization, allowing developers to define and enforce domain-specific safety policies.

### 3. Support user-defined behavior preferences

In autonomous driving, many decisions are inherently non-deterministic. For example, the timing of a lane change often depends on driver preference and context. Both keeping the current lane and changing lanes can be equally valid choices.

Some E2E planners, such as [DiffusionDrive](https://github.com/hustvl/DiffusionDrive) (the paper ["DiffusionDrive: Truncated Diffusion Model for End-to-End Autonomous Driving"](https://arxiv.org/abs/2411.15139)), can generate multiple feasible trajectories in such scenarios. However, the vehicle must ultimately commit to a single maneuver. Therefore, the architecture must support a mechanism for users to influence decision-making, allowing their preferences to guide behavior selection when multiple valid options exist.

DiffusionDrive is a published research planner. It is not the Diffusion Planner ([`autoware_diffusion_planner`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_diffusion_planner)) of Autoware 2.0.0. The Diffusion Planner makes one candidate trajectory for each element of its batch, and its default `batch_size` is 1.

## Generator-Selector framework

To support these diverse approaches, we proposed a Generator-Selector framework. This framework has two core components:

- Generator: Generates candidate trajectories that the vehicle can follow.
- Selector: Selects the safest and most optimal trajectory from among the candidates.

{{ diagram("media/detailed_architecture_figure2.svg", "Figure 2: The Generator-Selector framework.") }}

This framework enables unified handling of different autonomous driving architectures with varying levels of End-to-End AI technologies, while allowing safety and performance considerations to guide the final decision.

!!! info "In Autoware 2.0.0"

    The messages and the packages of this framework are in Autoware 2.0.0. The `planning_setting` launch argument selects one generator: the rule-based planning stack or the Diffusion Planner. The selector packages are in Autoware Universe, but Autoware 2.0.0 does not launch them yet. The [Architecture 2.0](../index.md#what-autoware-200-does-today) page gives the details.

### Generator

The Generator can be any autonomous driving stack capable of generating candidate trajectories that the vehicle may follow. For example:

- Traditional robotics stack: The existing Autoware stack can be reused as-is.
- Proprietary AI models: Autoware Foundation member companies can use their in-house models.
- Open-source end-to-end AI: Autoware E2E models.

Multiple Generators can be executed in parallel or selectively activated depending on the context.

A generator publishes its candidates as an `autoware_internal_planning_msgs/msg/CandidateTrajectories` message. Each `CandidateTrajectory` in it has a `generator_id`, and the `GeneratorInfo` list maps each `generator_id` to a generator name. In Autoware Universe 0.53.1, these generator packages publish this message:

- `autoware_diffusion_planner`: the Diffusion Planner, a learned planner.
- `autoware_minimum_rule_based_planner`: a minimal rule-based planner that follows the lanelet centerline. The ranker treats it as a backup planner.
- `autoware_tensorrt_vad`: the VAD model, a monolithic learned driving model. It also publishes the best candidate as a `Trajectory`.

`autoware_trajectory_modifier` and `autoware_trajectory_selector` also publish `CandidateTrajectories` messages, with the candidates that they processed.

### Selector

The Selector is responsible for two main functions:

- Safety Gate (Safety Assurance)
  - Validates the output of black-box Generators (for example, neural networks) to ensure a minimum level of safety. For example, it uses an HD map to make sure that the trajectory obeys the traffic signals.
- Ranking (Trajectory Evaluation and Selection)
  - Evaluates and ranks the outputs from multiple Generators, then selects the best one.
    - Examples:
      - Use a robotics-based approach when an HD map is available. Fall back to End-to-End AI models otherwise.
      - Score trajectories based on driving policies such as safety, comfort, or rule compliance.

These Selector functions are implemented as plugins, allowing developers to customize safety requirements and inject preferences to select a suitable trajectory for their own use case.

The node [`autoware_trajectory_selector`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_selector) implements the selector in Autoware Universe 0.53.1:

<div class="aw-steps" markdown>

1. **Concatenate.** [`autoware_trajectory_concatenator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_concatenator) merges the latest candidates of each generator. The node has one input for generative planners and one input for backup planners.
2. **Safety gate.** The plugins of [`autoware_trajectory_validator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_validator) remove the candidates that fail a check. The built-in plugins check the vehicle limits, the uncrossable map boundaries, the traffic lights, and the crosswalks.
3. **Ranking.** [`autoware_trajectory_ranker`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_ranker) gives each candidate a score from its safety risk level, its source (the Diffusion Planner or a backup planner), and optional quality metric plugins.
4. **Adapt.** [`autoware_trajectory_adapter`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_adapter) converts the candidate with the highest score to an `autoware_planning_msgs/msg/Trajectory`.

</div>

### Reference links

- GitHub Discussions:
  - [The difficulty in handling complex scenarios due to the hierarchical architecture of Planning](https://github.com/orgs/autowarefoundation/discussions/5033)
  - [Introducing Generator + Selector Architecture for Modular Trajectory Planning](https://github.com/orgs/autowarefoundation/discussions/6301)
- Proposed architecture interface: [tier4/new_planning_framework wiki](https://github.com/tier4/new_planning_framework/wiki)
- Migration plan for the new generator-selector framework: [Enable generator-selector planning framework](https://github.com/autowarefoundation/autoware/issues/6292)
