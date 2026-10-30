# AWSIM simulator

[AWSIM](https://github.com/autowarefoundation/AWSIM) is a simulator for Autoware development and testing. TIER IV developed it first. It is now maintained in the Autoware Foundation GitHub organization.

To get started with AWSIM, follow the [Quick start demo](https://autowarefoundation.github.io/AWSIM/GettingStarted/QuickStartDemo/) in the AWSIM documentation. It lists the PC requirements, the network settings, and the steps to run the demo.

!!! tip

    There are 2 versions of the AWSIM demo:

    - 🐇 URP (Universal Render Pipeline) version: ⭐ **AWSIM-Demo-Lightweight.zip**
      - This is the recommended version.
    - 🐢 HDRP (High Definition Render Pipeline) version: **AWSIM-Demo.zip**
      - This version has higher graphical fidelity but it is very heavy.

    Find more details in [AWSIM/Downloads](https://autowarefoundation.github.io/AWSIM/Downloads/).

## Launch Autoware for the AWSIM demo

The demo drives in Shinjuku, Tokyo. Download `Shinjuku-Map.zip` from [AWSIM/Downloads](https://autowarefoundation.github.io/AWSIM/Downloads/) and extract it. It holds the Lanelet2 map and the point cloud map that Autoware uses.

Start AWSIM first. Then launch Autoware with the `e2e_simulator.launch.xml` launch file and the AWSIM sensor kit:

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=<path-to-Shinjuku-Map>/map
```

AWSIM publishes Autoware messages directly, so the launch file starts no simulator interface for it (`simulator_type` is `awsim` by default).

!!! tip "Run Autoware in Docker"

    The Autoware repository has a Docker Compose example for AWSIM in [`docker/examples/demos/awsim`](https://github.com/autowarefoundation/autoware/tree/2.0.0/docker/examples/demos/awsim). It runs the same launch command in a container. AWSIM runs on the host.

!!! warning

    If you are looking for [AWSIM Labs](https://github.com/autowarefoundation/AWSIM-Labs), it is archived, see the [announcement post](https://github.com/orgs/autowarefoundation/discussions/6706).
