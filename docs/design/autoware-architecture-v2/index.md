# Autoware 2.0 architecture

Autoware 2.0 adds AI planning next to the rule-based planning stack. The design is the generator-selector architecture. A generator produces candidate trajectories. A selector checks the candidates and picks one.

## What Autoware 2.0.0 does today

In Autoware 2.0.0, you choose the planner when you launch Autoware. The `planning_setting` launch argument has two values:

- `rule_based` (default): the rule-based scenario planner makes the trajectory, as in Autoware 1.x.
- `diffusion_planner`: the [Diffusion Planner](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_diffusion_planner), a learned planner, makes the trajectory.

Only one generator runs at a time. The selector packages are in `autoware_universe`, but Autoware 2.0.0 does not launch them yet. The launch file has a placeholder where the selector goes. Until then, `autoware_trajectory_modifier` passes the first candidate trajectory to the planning validator.

The rest of this page describes the target architecture. The [Generator-Selector framework](roadmap/detailed-architectural-interface.md#generator-selector-framework) page gives the full design.

### Data flow with the Diffusion Planner

{{ diagram("images/planning-data-flow.svg", "The planning_setting launch argument selects the source of the trajectory. Both sources go through the planning validator.") }}

1. `autoware_diffusion_planner` publishes an `autoware_internal_planning_msgs/msg/CandidateTrajectories` message on `/planning/generator/diffusion_planner/candidate_trajectories`.
2. `autoware_trajectory_modifier` applies its modifier and optimizer plugins to each candidate. It publishes the first candidate on `/planning/trajectory_generator/trajectory`.
3. `autoware_planning_validator` reads this trajectory and publishes `/planning/trajectory` for the control stack.

With `planning_setting:=rule_based`, the planning validator reads `/planning/scenario_planning/velocity_smoother/trajectory` instead.

### Packages

| Role                 | Package                                                                                                                                       | Status in 2.0.0                                                      |
| :------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------- |
| Generator            | [`autoware_diffusion_planner`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_diffusion_planner)       | Runs with `planning_setting:=diffusion_planner`                      |
| Post-processing      | [`autoware_trajectory_modifier`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_modifier)   | Runs with `planning_setting:=diffusion_planner`                      |
| Selector node        | [`autoware_trajectory_selector`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_selector)   | Not launched                                                         |
| Selector safety gate | [`autoware_trajectory_validator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_validator) | Not launched. It is a library inside `autoware_trajectory_selector`. |
| Selector ranking     | [`autoware_trajectory_ranker`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_ranker)       | Not launched. It is a library that a selector node can use.          |
| Messages             | [`autoware_internal_planning_msgs`](https://github.com/autowarefoundation/autoware_internal_msgs/tree/1.16.0/autoware_internal_planning_msgs) | `CandidateTrajectories`, `CandidateTrajectory` and `GeneratorInfo`   |

The launch files are in [`autoware_planning_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_planning_launch/launch) in `autoware_launch`.

### Run the Diffusion Planner in the planning simulator

The Diffusion Planner uses TensorRT, so you need an NVIDIA GPU.

1. Follow the [planning simulation](../../demos/planning-sim/index.md) preparation steps. They download the sample map and the AI models.
2. Make sure that the model folder exists:

   ```bash
   ls ~/autoware_data/ml_models/diffusion_planner/v5.0/
   ```

3. Launch the planning simulator with the Diffusion Planner:

   ```bash
   source ~/autoware/install/setup.bash
   ros2 launch autoware_launch planning_simulator.launch.xml \
     map_path:=$HOME/autoware_data/maps/sample-map-planning \
     vehicle_model:=sample_vehicle \
     sensor_model:=sample_sensor_kit \
     planning_setting:=diffusion_planner
   ```

4. Set the initial pose and the goal pose in RViz, as in the [planning simulation](../../demos/planning-sim/index.md) demo.

At the first launch, the Diffusion Planner builds a TensorRT engine from the ONNX model. As a result, the first start takes longer than the next starts.

### AI models

The Diffusion Planner and the other AI models of Autoware are on [Hugging Face](https://huggingface.co/AutowareFoundation). Each model has version tags. The [`artifacts`](https://github.com/autowarefoundation/autoware/tree/2.0.0/ansible/roles/artifacts) Ansible role downloads each model at a fixed version with the `hf` CLI into `~/autoware_data/ml_models/`. For example, the role downloads the Diffusion Planner versions `v3.0`, `v3.1`, `v4.0` and `v5.0`, and Autoware 2.0.0 uses `v5.0`.

To download the models, run this command from the `autoware` directory:

```bash
ansible-playbook autoware.dev_env.install_dev_env --tags artifacts
```

The [Autoware Index](../../installation/autoware/autoware-index.md) also lists AI inference packages outside the main workspace. Their models are on Hugging Face too.

## Generator - Selector architecture: conceptual overview

{{ diagram("images/generator-selector.svg", "Generators make candidate trajectories. The selector checks them and picks one for the controller.") }}

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
