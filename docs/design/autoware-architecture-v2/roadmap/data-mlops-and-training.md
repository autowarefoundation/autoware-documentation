<!-- cspell:ignore oneplanner Kueue -->

# Data, MLOps, and training

This page describes how the AI models of Autoware are trained, versioned, and distributed today. It describes only work that exists, with links to the source. Machine learning operations (MLOps) are the tools and processes that train, track, and deliver models.

The Autoware 2.0.0 release does not contain training code. Each model has its own training repository, and the release downloads the trained models.

<div class="aw-pipeline" markdown>

- :fa-cl-s fa-database: <span>Data</span>
- :fa-cl-s fa-microchip: <span>Training</span>
- :fa-cl-s fa-tag: <span>Versioned model on Hugging Face</span>
- :fa-cl-s fa-download: <span>`artifacts` role</span>
- :fa-cl-s fa-car: <span>Autoware node</span>

</div>

## Model distribution

The Autoware Foundation hosts the AI models of Autoware on [Hugging Face](https://huggingface.co/AutowareFoundation). Each model is a Hugging Face repository, and each model version is a tag on that repository.

The [`artifacts`](https://github.com/autowarefoundation/autoware/tree/2.0.0/ansible/roles/artifacts) Ansible role of Autoware 2.0.0 downloads each model at a fixed tag with the `hf` CLI. The default destination is `~/autoware_data/ml_models/`. The role downloads the models of these packages and more:

- Perception, for example `lidar_centerpoint`, `lidar_transfusion`, `bevfusion`, `tensorrt_yolox`, the traffic light classifiers, and `simpl_prediction`.
- Planning: `diffusion_planner` (tags `v3.0`, `v3.1`, `v4.0`, and `v5.0`).
- End-to-end driving: `tensorrt_vad` (tag `v0.1`).

A launch file selects one version through a parameter path. For example, the Diffusion Planner of Autoware 2.0.0 reads `$(var data_path)/diffusion_planner/v5.0`.

The Hugging Face organization also holds models that are not in the release, for example `auto_e2e`, `meteor`, `oneplanner`, and `latent_drive`. It also holds datasets, for example `meteor-demo-scenes`, CARLA maps, and the `autoware-ai-archive` recordings.

## Training of the models in Autoware Universe

| Package                                                                                                                                 | Model and training                                                                                                                                                                                           |
| :-------------------------------------------------------------------------------------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`autoware_diffusion_planner`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_diffusion_planner) | Diffusion Planner, from the paper "Diffusion-Based Planning for Autonomous Driving with Flexible Guidance". The [tier4/Diffusion-Planner](https://github.com/tier4/Diffusion-Planner) fork trains the model. |
| [`autoware_tensorrt_vad`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/e2e/autoware_tensorrt_vad)                | VAD (Vectorized Autonomous Driving). The model is trained on the Bench2Drive dataset, which is CARLA simulation data. NVIDIA DL4AGX optimizes it for TensorRT.                                               |

The README of each package gives the model versions and the links to the original papers.

## Training projects of the Autoware Foundation

### AutoE2E

[AutoE2E](https://github.com/autowarefoundation/auto_e2e) is an open-source, camera-first E2E driving model. The repository holds the full training stack:

- The `Model` folder holds the model, the data processing, the training, the distributed training, and the evaluation code.
- The `Platform` folder holds an MLOps platform on Amazon EKS. It uses MLflow to track experiments, Flyte to run the pipelines, and Kueue to queue the GPU jobs. The datasets are WebDataset shards in Amazon S3.
- The `Tools` folder holds the DataModelConsole, a dashboard that replays camera scenes with the ground-truth and the predicted trajectories.

The released checkpoints are trained on nuPlan, and one checkpoint is fine-tuned on KITScenes-Multimodal. The checkpoints are on [Hugging Face](https://huggingface.co/AutowareFoundation/auto_e2e).

### METEOR

[METEOR](https://github.com/tier4/METEOR) is a reference E2E model for surround-view driving from TIER IV. Its repository holds the source, the training recipe, an autolabel factory, and the runtimes. The autolabel factory makes all the training labels from raw recordings, without human labels. The weights are on [Hugging Face](https://huggingface.co/AutowareFoundation/meteor). The announcement is the discussion [METEOR: an open reference E2E model for surround-view driving](https://github.com/orgs/autowarefoundation/discussions/7312).

## Data tools

- [`autoware_rosbag2_anonymizer`](https://github.com/autowarefoundation/autoware_rosbag2_anonymizer) anonymizes the images in ROS 2 bags, so that you can share recorded data.
- [`autoware_planning_data_analyzer`](https://github.com/autowarefoundation/autoware_tools/tree/main/planning/autoware_planning_data_analyzer) evaluates trajectories from rosbag data. The [Assessment of safety and benchmarks](assessment-safety-and-benchmarks.md) page describes it.
