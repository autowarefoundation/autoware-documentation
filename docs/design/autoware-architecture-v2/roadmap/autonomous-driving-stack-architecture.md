<!-- cspell:ignore oneplanner alpamayo Alpamayo -->

# Autonomous driving stack architecture

This page describes the planned evolution of the Autoware stack, step by step. Each step has a note that tells what exists in Autoware 2.0.0.

## Traditional robotics stack

{{ diagram("media/architecture_figure01.svg", "Figure 1: The traditional robotics stack for autonomous driving.") }}

Early architectures for autonomous driving followed a traditional robotics stack, with hand-coded algorithms for perception, localization, and planning, as Figure 1 shows. These approaches can achieve robust performance in known scenarios and in constrained Operational Design Domains (ODDs). But this classical architecture has some drawbacks.

These include difficulty in handling previously unseen scenarios, challenges in code upgrades for new features, driving behavior which doesn’t match natural human driving styles, and challenges in addressing negotiated driving maneuvers such as merges, nudges, giving way and taking way.

!!! info "In Autoware 2.0.0"

    The rule-based stack of Autoware is a robotics stack of this type. It is the default stack in Autoware 2.0.0. The [component pages](../../autoware-architecture-v1/components/sensing/index.md) describe it.

## Introduction to Autoware E2E

The roadmap uses the name Autoware E2E (end-to-end) for the next evolution of the Autoware project: from a traditional robotics stack to an AI-first, data-centric stack with end-to-end autonomous driving features.

This evolution is being driven by our core value of developing ‘cutting edge’ technologies, as well as our desire to offer the autonomous driving community with a new open-source technology paradigm, powered by data-centric AI.

The Autoware E2E stack aims to achieve global scalability across ODDs with data-centric end-to-end AI. In this plan, neural networks replace the hand-coded modules in a modular and incremental way. Neural networks show great promise in handling diverse and complex driving scenarios across domains.

{{ diagram("media/architecture_figure02.svg", "Figure 2: The roadmap definition of Level 4+ autonomous driving, compared with SAE Levels 3, 4, and 5.") }}

In particular, we aim to achieve Level 4+ autonomous driving functionality through Autoware E2E. Level 4+ is not an SAE level. [SAE J3016](https://www.sae.org/standards/content/j3016_202104/) defines six levels of driving automation, from Level 0 to Level 5. The roadmap uses Level 4+ for its own definition: a system that meets SAE Level 4 and can also dynamically adapt to new scenarios that it did not encounter before. Like SAE Level 4, it drives only under limited conditions and does not require the human to take over driving. In Figure 2, the first three rows summarize the SAE J3016 descriptions of Levels 3, 4, and 5. The Level 4+ column and the last row are the roadmap's own definitions.

## Autoware E2E architecture description

The plan implements the Autoware E2E architecture in an evolutionary fashion, in steps. Each step builds on the technology of the earlier steps. This stepwise approach gives a smooth transition without breaking changes. It also allows thorough evaluation and testing of the learned AI-based modules when they are introduced.

### Step 1: Learned planner

Step 1 aims to introduce a learned planning module which is able to ingest a world state consisting of the ego-vehicle’s localized position with respect to an HD map, alongside key perception information including elements such as 3D bounding boxes of other foreground objects, traffic light state etc.

{{ diagram("media/architecture_figure03.svg", "Figure 3: Step 1, the learned planning architecture.") }}

The learned planning module is able to take this ‘local world representation’ from upstream perception and localization processes and devises an optimal trajectory which aligns with the overall navigation goal as determined by the mission plan.

The advantages of a learned local planner are that complex negotiated traffic scenarios such as merges, nudges and giving way can be learned from human driving data and do not need to be hand-coded. Additionally, the model can be trained on data from different cities to better learn local driving styles which match the temperament of drivers - for example, driving in London is very different from driving in Mumbai.

!!! info "In Autoware 2.0.0"

    The [Diffusion Planner](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_diffusion_planner) is a learned planner of this type. It reads the ego state from localization, the vector map, the route, the tracked objects, and the traffic light states, and it makes candidate trajectories. To use it, launch Autoware with `planning_setting:=diffusion_planner`. The [Architecture 2.0](../index.md#what-autoware-200-does-today) page gives the details.

### Step 2: Component-based end-to-end AI with learned perception and learned planning

Step 2 builds on Step 1 and adds learned perception, so that the ‘local world representation’ is also learned instead of hand-coded. At this stage, the vehicle can use an HD map and a localizer (option A). Or the vehicle can create a ‘local map’ implicitly with learned perception, which estimates the possible driving corridors and localizes key scene elements with respect to the ego vehicle. This option usually uses a 2D map (option B).

Option A:

{{ diagram("media/architecture_figure04.svg", "Figure 4: Step 2, option A. Learned perception and learned planning with an HD map.") }}

Option B:

{{ diagram("media/architecture_figure05.svg", "Figure 5: Step 2, option B. Learned perception and learned planning with a standard 2D map.") }}

In Step 2, the entire autonomous driving stack is learned and trainable end-to-end. But there is an interface between the learned perception module and the learned planning module: the outputs of learned perception must satisfy the requirements of learned planning. This approach decouples perception and planning and allows for introspection of the full end-to-end stack. As a result, debugging and testing are easier.

The advantage of option B is that the system is available wherever a 2D map is present. This gives a larger geographic coverage from the start, because you do not have to make and maintain an HD map. But option A is more robust because of the HD map. The HD map gives the vehicle more information about the shape of the road, key infrastructure elements, and the possible driving corridors of other road actors.

!!! info "In Autoware 2.0.0"

    Autoware 2.0.0 has no Step 2 stack. Many perception modules of the rule-based stack already use learned models, for example the lidar object detectors. The [`artifacts`](https://github.com/autowarefoundation/autoware/tree/2.0.0/ansible/roles/artifacts) role lists their models.

### Step 3: Learned driving

Step 3 builds on top of the developments in Step 2 to create a single Learned Driving, monolithic neural network, which internally learns a ‘local world representation’ without any explicit learned perception or planning modules. There is no traffic light detection, semantic segmentation, or lane detection: the model represents all these factors implicitly in its feature space. A decoder takes these features and estimates a single ideal driving trajectory, which a closed-loop controller tracks. The motivation of such monolithic approaches is that they do not constrain the perceptual elements that should be extracted for optimal local planning. The stack can learn unique scene-level features and their relationships for full end-to-end driving.

Similar to Step 2, there are two options to implement Step 3. In option A, an HD map gives the vehicle detailed scene information. The HD map acts almost as a non-line-of-sight sensor for the static environment: it encodes the positions of traffic lights, the road geometry, the lane topologies, the rights of way, and more. In option B, the vehicle uses a sat-nav style 2D or ADAS map. As a result, the learned driving model must build an internal, feature-level representation of the same information that the HD map encodes.

Option A:

{{ diagram("media/architecture_figure06.svg", "Figure 6: Step 3, option A. A monolithic learned driving model with an HD map.") }}

Option B:

{{ diagram("media/architecture_figure07.svg", "Figure 7: Step 3, option B. A monolithic learned driving model with a standard 2D map.") }}

Many neural network architectures can work as learned driving models. Broadly, they belong to two types: convolutional neural network models and vision-language-action models.

#### Convolutional neural network models

Convolutional Neural Network models usually rely upon two stages within the model. In the first stage, sensor data is passed through a ‘Feature Backbone’ which is responsible for extracting and capturing general-purpose image features which represent the content of the scene. These features are then fed into a second stage, often referred to as a ‘Policy Model’, which usually consists of fully-connected neural network layers which are responsible for predicting the future trajectory of the vehicle.

The advantage of convolutional neural network models is that, with a careful selection of the feature backbone and design of the policy model, you can deploy the model on embedded edge hardware. You can also train it on fewer data samples than larger, more complex models. But a significant limitation of these models is that they can fail to capture the overall scene context and the relationships between scene elements. Instead, they capture higher-level features, for example the presence of shapes or objects. This can decrease the performance and the generalizability of the model.

#### Vision-language-action models

An alternative architecture for Learned Driving is based on Vision-Language-Action models. These models utilize a combination of Vision Transformers and Large Language Models to jointly learn image features, scene context and reason about driving decisions.

The model can be conditioned on learned queries or text inputs, for example ‘continue straight along the road’ and the model can execute a driving policy to follow these commands. A diffusion transformer model decodes the network features to output the driving action in the form of a trajectory and can also optionally output text, which aims to explain the reasoning through which the model predicted the trajectory - helping with model introspection and explainability.

#### Open challenges with learned driving

Although both Convolutional Neural Network architectures and Vision-Language-Action models present viable options at executing Learned Driving, and demonstrate great promise and potential, there still remain challenges with these approaches. One of the most important challenges is that the entire autonomous driving stack is learned and executed in a single forward pass of a large monolithic neural network, which can be difficult to train and debug, since there are no explicit scene level representations - only representations inside the neural network’s feature space. Such Learned Driving networks are also difficult to validate, since it is challenging to truly understand ‘what the network has learned’ in feature space. Such approaches are also limited by the quality of training data - for example, a naive approach which simply applies Imitation Learning may learn to follow human drivers which disobey traffic rules, such as not stopping at a stop sign - making data cleanliness of utmost importance.

!!! info "In Autoware 2.0.0 and current work"

    - [`autoware_tensorrt_vad`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/e2e/autoware_tensorrt_vad) in the `e2e/` folder of Autoware Universe runs VAD, a monolithic model that maps six camera images to trajectories. The model is trained on CARLA simulation data. The default launch files of Autoware 2.0.0 do not launch it.
    - [AutoE2E](https://github.com/autowarefoundation/auto_e2e) is an open-source, camera-first E2E driving model of the Autoware Foundation. Its weights are on [Hugging Face](https://huggingface.co/AutowareFoundation/auto_e2e).
    - [`autoware_oneplanner`](https://github.com/autowarefoundation/autoware_oneplanner) is a ROS 2 runtime for OnePlanner E2E planning models. They read the lidar point cloud, the vector map, the route, and the traffic light states.
    - [`autoware_latent_drive`](https://github.com/autowarefoundation/autoware_latent_drive) is a TensorRT inference framework for the LatentDrive camera-based E2E planner.
    - [`alpamayo-autoware`](https://github.com/autowarefoundation/alpamayo-autoware) is a ROS 2 node for Alpamayo 1.5, a reasoning model from NVIDIA. It makes a trajectory and a text that explains its reasoning.

    These repositories are outside the Autoware 2.0.0 release.

### Step 4: Learned hybrid approach (under development by industry)

To get the best of both worlds, Step 4 uses a learned hybrid approach. It combines a learned driving model with complementary safety perception modules. This gives redundancy, more built-in safety and introspection, and diversity in sensing. As in the earlier steps, you can use an HD map or a 2D map.

#### Safety perception

Safety Perception comprises dedicated perception modules which play a key role in helping to ensure that rare edge case scenarios which may not be appropriately handled by a Learned Driving model, do not result in collisions. While the Learned Driving model can rely upon camera-based sensing, Safety Perception modules can leverage physics-based 3D sensors such as LIDAR and RADAR. To improve introspectability, Safety Perception modules can utilize rule-based implementations allowing for complete transparency, or alternatively, they can also be Learned Perception models.

The motivation of this is to help autonomous vehicles achieve a super-human capability by leveraging sensing modalities not available to human drivers, who rely upon vision based driving alone. LIDAR and RADAR provide excellent perceptual redundancy and robustness in adverse weather conditions such as fog, rain, heavy snow, and dust storms as well as challenging lighting conditions such as pitch-black, or sun glare.

#### Safety guardian

A Safety Guardian module is responsible for cross-referencing the Safety Perception outputs with the Learned Driving predictions to avoid safety-critical edge-case scenarios where, for example, the Learned Driving model may not be able to see an obstacle due to adverse weather or occlusion, and the Safety Perception modules can fill this safety gap. Additionally, the Safety Guardian can utilize infrastructure-based scene perception data or other connected vehicle data via V2X (vehicle to infrastructure communication) if it is available to further enhance safety of the self-driving system. As in Steps 2 and 3, Step 4 can use either an HD map or a standard 2D or ADAS map.

Option A:

{{ diagram("media/architecture_figure08.svg", "Figure 8: Step 4, option A. A learned hybrid model with an HD map.") }}

Option B:

{{ diagram("media/architecture_figure09.svg", "Figure 9: Step 4, option B. A learned hybrid model with a standard 2D map.") }}

!!! info "In Autoware 2.0.0"

    Autoware 2.0.0 has no learned hybrid stack. A related mechanism is the safety gate of the selector: the plugins of [`autoware_trajectory_validator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_validator) reject candidate trajectories that break vehicle limits, cross uncrossable map boundaries, or break traffic light and crosswalk rules. This library is in Autoware Universe, but Autoware 2.0.0 does not launch it.
