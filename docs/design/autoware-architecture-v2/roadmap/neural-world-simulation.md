# Neural world simulation

Neural world simulation uses learned models to make realistic simulated worlds and sensor data. For example, a learned scene model of a real street can render new camera images when the simulated vehicle drives a different path. With such data, you can test and train learned driving models in scenes that look like the real world.

This page describes only work that exists today, with links to the source. Autoware 2.0.0 does not contain a neural simulator.

## 3D Gaussian Splatting

3D Gaussian Splatting (3DGS) represents a real scene as many small 3D Gaussian shapes that are learned from recorded data. A renderer can draw new views of the scene in real time.

<div class="grid cards aw-cards" markdown>

- :fa-cl-s fa-camera: **[SplatSim](https://github.com/orgs/autowarefoundation/discussions/7133)**

  A photo-realistic simulation tool based on 3DGS. It renders camera images and publishes them through CycloneDDS, so that ROS 2 and Autoware can read them. It can co-simulate with CARLA: CARLA simulates the traffic participants, and SplatSim renders the camera images. In May 2026, the authors announced that they prepare to open-source it, and that the integration into Autoware is in progress.

- :fa-cl-s fa-file-code: **[3dgs_io](https://github.com/autowarefoundation/3dgs_io)**

  A Python library of the Autoware Foundation that reads and writes 3DGS data in glTF, 3D Tiles, SPZ, and PLY formats. It defines scene bundles for simulation, with sensor rigs, trajectories, and tracks of dynamic objects. A scene bundle can also embed an Autoware map directory. The [3dgs_io documentation](https://autowarefoundation.github.io/3dgs_io/) describes the formats.

</div>

The [Simulation working group meeting of 2026-01-14](https://github.com/orgs/autowarefoundation/discussions/6723) had the digital twin creation activities of 2026 with Gaussian Splatting on its agenda. The AutoE2E project keeps a [3DGS literature review](https://github.com/autowarefoundation/auto_e2e/tree/main/KnowledgeBase/3D_Gaussian_Review).

## Generated training data

The training of [METEOR](https://github.com/orgs/autowarefoundation/discussions/7312), a reference E2E model from TIER IV, uses NVIDIA Cosmos Transfer. This generative model renders recorded scenes again in adverse conditions, which adds training data for these conditions.

## Related simulation work

The design discussion [CARLA × Autoware integration architecture for closed-loop evaluation](https://github.com/orgs/autowarefoundation/discussions/7277) proposes a sensor simulation layer, in which a renderer makes camera and lidar data from the CARLA scene. The [Assessment of safety and benchmarks](assessment-safety-and-benchmarks.md) page lists the simulators that test Autoware today.
