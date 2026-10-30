# Architecture 2.0

Architecture 2.0 is the architecture of Autoware 2.0. It adds learned planning next to the rule-based planning stack. The design is the generator-selector architecture: a generator makes candidate trajectories, and a selector checks the candidates and picks one.

The rule-based stack (sensing, map, localization, perception, planning, control, and vehicle) is still the default stack in Autoware 2.0.0. Architecture 2.0 does not replace it. The rule-based planning stack becomes one generator among others.

<div class="grid cards aw-cards aw-cards--small" markdown>

- :fa-cl-s fa-cubes: **[Components](../autoware-architecture-v1/components/sensing/index.md)**

  The design of each component of the rule-based stack.

- :fa-cl-s fa-plug: **[Interfaces](../autoware-architecture-v1/interfaces/index.md)**

  The messages between the components, and the AD API for external systems.

- :fa-cl-s fa-diagram-project: **[Node diagram](../autoware-architecture-v1/node-diagram/index.md)**

  All the nodes and topics of Autoware in one diagram.

- :fa-cl-s fa-route: **[Roadmap](roadmap/index.md)**

  The steps from the robotics stack to learned driving.

</div>

## The generator-selector architecture

{{ diagram("images/generator-selector.svg", "Generators make candidate trajectories. The selector checks them and picks one for the controller.") }}

A traditional autonomous driving stack is a fixed pipeline:

<div class="aw-pipeline" markdown>

- :fa-cl-s fa-satellite-dish: <span>Sensing</span>
- :fa-cl-s fa-eye: <span>Perception</span>
- :fa-cl-s fa-location-crosshairs: <span>Localization</span>
- :fa-cl-s fa-route: <span>Planning</span>
- :fa-cl-s fa-chart-line: <span>Trajectory</span>

</div>

This pipeline works well for rule-based planners. Newer approaches, for example end-to-end (E2E) models and diffusion models, do not fit this structure. They can skip or replace parts of the pipeline, so it is difficult to integrate them. To support both classical and newer approaches, Architecture 2.0 abstracts away the front half of the pipeline.

<div class="aw-features" markdown>

- :fa-cl-s fa-wand-magic-sparkles: **Generators make trajectories**

  A generator is any module that outputs candidate trajectories: a rule-based or optimization planner that uses perception and maps, an E2E model that uses raw sensor data, or a learned or sampling-based planner. A generator can use the sensing, perception, localization, and control of Autoware, or it can bypass them. Many generators can run in parallel.

- :fa-cl-s fa-shield-halved: **The selector makes the final choice**

  The selector receives the candidate trajectories. First, it does safety checks on them, for example traffic rules and the drivable area. Then it ranks the safe candidates and selects the best one for the context or the driving policy.

</div>

!!! tip "What this architecture makes possible"

    - Robotics planners and E2E planners work in the same stack.
    - Explicit checks make the use of black-box models safer.
    - You can try a new planning method without changes to the rest of the stack.
    - The selector can compare many trajectory proposals before it decides.

The [Generator-Selector framework](roadmap/detailed-architectural-interface.md#generator-selector-framework) page gives the full design.

## What Autoware 2.0.0 does today

In Autoware 2.0.0, you choose the planner when you launch Autoware. The `planning_setting` launch argument has two values:

- `rule_based` (default): the rule-based scenario planner makes the trajectory, as in Autoware 1.x.
- `diffusion_planner`: the [Diffusion Planner](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_diffusion_planner), a learned planner, makes the trajectory.

Only one generator runs at a time. The selector packages are in `autoware_universe`, but Autoware 2.0.0 does not launch them yet. The launch file has a placeholder where the selector goes. Until then, `autoware_trajectory_modifier` passes the first candidate trajectory to the planning validator.

### Data flow

{{ diagram("images/planning-data-flow.svg", "The planning_setting launch argument selects the source of the trajectory. Both sources go through the planning validator.") }}

With `planning_setting:=diffusion_planner`, the data flows as follows:

1. `autoware_diffusion_planner` publishes an `autoware_internal_planning_msgs/msg/CandidateTrajectories` message on `/planning/generator/diffusion_planner/candidate_trajectories`.
2. `autoware_trajectory_modifier` applies its modifier and optimizer plugins to each candidate. It publishes the first candidate on `/planning/trajectory_generator/trajectory`. It also publishes all the processed candidates on `/planning/generator/candidate_trajectories`.
3. `autoware_planning_validator` reads this trajectory and publishes `/planning/trajectory` for the control stack.

With `planning_setting:=rule_based`, the planning validator reads `/planning/scenario_planning/velocity_smoother/trajectory` instead.

### Packages

| Role             | Package                                                                                                                                                   | Status in 2.0.0                                                                                                                            |
| :--------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------- |
| Generator        | [`autoware_diffusion_planner`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_diffusion_planner)                   | Runs with `planning_setting:=diffusion_planner`                                                                                            |
| Post-processing  | [`autoware_trajectory_modifier`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_modifier)               | Runs with `planning_setting:=diffusion_planner`                                                                                            |
| Backup generator | [`autoware_minimum_rule_based_planner`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_minimum_rule_based_planner) | Not launched. It follows the lanelet centerline and publishes `CandidateTrajectories`.                                                     |
| Selector node    | [`autoware_trajectory_selector`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_selector)               | Not launched. It concatenates, validates, and ranks the candidates, and publishes one `Trajectory`. It uses the four libraries below.      |
| Concatenation    | [`autoware_trajectory_concatenator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_concatenator)       | Not launched. It is a library inside `autoware_trajectory_selector`. It merges the candidates of all generators.                           |
| Safety gate      | [`autoware_trajectory_validator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_validator)             | Not launched. It is a library inside `autoware_trajectory_selector`. Its plugins remove unsafe candidates and candidates that break rules. |
| Ranking          | [`autoware_trajectory_ranker`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_ranker)                   | Not launched. It is a library inside `autoware_trajectory_selector`. It scores each candidate.                                             |
| Adapter          | [`autoware_trajectory_adapter`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_adapter)                 | Not launched. It is a library inside `autoware_trajectory_selector`. It converts the best candidate to a `Trajectory`.                     |
| Messages         | [`autoware_internal_planning_msgs`](https://github.com/autowarefoundation/autoware_internal_msgs/tree/1.16.0/autoware_internal_planning_msgs)             | `CandidateTrajectories`, `CandidateTrajectory`, `GeneratorInfo`, `ScoredCandidateTrajectories`, and `ScoredCandidateTrajectory`            |

The launch files are in [`autoware_planning_launch`](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/autoware_universe_launch/autoware_planning_launch/launch) in `autoware_launch`.

### Run the Diffusion Planner in the planning simulator

The Diffusion Planner uses TensorRT, so you need an NVIDIA GPU.

1. Follow the [planning simulation](../../simulation/planning-simulation/index.md) preparation steps. They download the sample map and make sure that the AI models are on your computer.
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

4. Set the initial pose and the goal pose in RViz, as in the [planning simulation](../../simulation/planning-simulation/index.md) demo.

At the first launch, the Diffusion Planner builds a TensorRT engine from the ONNX model. As a result, the first start takes longer than the next starts.

### AI models

The Diffusion Planner and the other AI models of Autoware are on [Hugging Face](https://huggingface.co/AutowareFoundation). Each model has version tags. The [`artifacts`](https://github.com/autowarefoundation/autoware/tree/2.0.0/ansible/roles/artifacts) Ansible role downloads each model at a fixed version with the `hf` CLI into `~/autoware_data/ml_models/`. For example, the role downloads the Diffusion Planner versions `v3.0`, `v3.1`, `v4.0`, and `v5.0`, and Autoware 2.0.0 uses `v5.0`.

To download the models, run this command from the `autoware` directory:

```bash
ansible-playbook autoware.dev_env.install_dev_env --tags artifacts --ask-become-pass
```

### Other learned driving packages

- [`autoware_tensorrt_vad`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/e2e/autoware_tensorrt_vad) is in the `e2e/` folder of Autoware Universe. It runs the VAD model, one network that maps six camera images to trajectories. The model is trained on CARLA simulation data. The package has its own launch file, and `autoware_launch` 0.53.0 does not include it.
- Vision Pilot, an open-source end-to-end Level 2 ADAS system, is in the [Autoware Index](../../installation/autoware/autoware-index.md). It has its own repository, and the `autoware.repos` file does not include it.

## The rule-based stack

The rule-based stack has seven components. Their design pages describe the default stack of Autoware 2.0.0.

<div class="grid cards aw-cards aw-cards--small" markdown>

- :fa-cl-s fa-satellite-dish: **[Sensing](../autoware-architecture-v1/components/sensing/index.md)**

  Preprocesses the raw sensor data and defines its input formats.

- :fa-cl-s fa-map: **[Map](../autoware-architecture-v1/components/map/index.md)**

  Gives the vector map and the point cloud map to the rest of the stack.

- :fa-cl-s fa-location-crosshairs: **[Localization](../autoware-architecture-v1/components/localization/index.md)**

  Estimates the pose, velocity, and acceleration of the vehicle.

- :fa-cl-s fa-eye: **[Perception](../autoware-architecture-v1/components/perception/index.md)**

  Recognizes the objects, obstacles, and traffic lights around the vehicle.

- :fa-cl-s fa-route: **[Planning](../autoware-architecture-v1/components/planning/index.md)**

  Makes the target trajectory (path and speed) of the vehicle.

- :fa-cl-s fa-gamepad: **[Control](../autoware-architecture-v1/components/control/index.md)**

  Makes the vehicle follow the trajectory.

- :fa-cl-s fa-car: **[Vehicle](../autoware-architecture-v1/components/vehicle/index.md)**

  Converts Autoware commands to the format of the vehicle, and the vehicle status to Autoware messages.

</div>

## Roadmap

The [roadmap](roadmap/index.md) describes the planned steps from the robotics stack to learned driving. Autoware 2.0.0 contains the first step: the Diffusion Planner is a learned planner that you can select at launch.
