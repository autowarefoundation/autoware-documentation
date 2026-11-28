# CARLA 1.0 with autoware_carla_interface_v2

This page runs Autoware on [CARLA 1.0](https://github.com/carla-simulator/carla/releases/tag/1.0) (Unreal Engine 5.8) with [`autoware_carla_interface_v2`](https://github.com/autowarefoundation/autoware_carla_interface_v2), and then runs a sample scenario of [`autoware_carla_scenario`](https://github.com/autowarefoundation/autoware_carla_scenario) against it.

CARLA 1.0 publishes the sensor data, the vehicle status, and `/clock` on Autoware's topics itself, and applies Autoware's control commands to the ego vehicle. `autoware_carla_interface_v2` is the client node that is left: it attaches to the ego, spawns the sensor kit on it, and publishes the traffic-light states. The package is registered in the [Autoware Index](https://github.com/autowarefoundation/autoware-index), so you add it to the workspace with [`aw-index-cli`](https://github.com/autowarefoundation/aw-index-cli).

| Component                     | Runs on             | Role                                                                                      |
| ----------------------------- | ------------------- | ----------------------------------------------------------------------------------------- |
| CARLA 1.0 server              | Host                | Simulates the world. Started with `--ros2`, it talks to Autoware over ROS 2               |
| Autoware                      | Jazzy dev container | `e2e_simulator.launch.xml`, started by `autoware_carla_interface_v2`'s launch file        |
| `autoware_carla_interface_v2` | Jazzy dev container | Attaches to the ego, spawns the sensor kit, publishes the traffic-light states            |
| `autoware_carla_scenario`     | Host (Python, `uv`) | Spawns the ego, serves the mission, and starts Autoware for every scenario in a container |

!!! note "Tested versions"

    This procedure was tested on 2026-11-19, and again from a fresh workspace on 2026-11-28, with:

    - Autoware: the `main` branch of every repository (`repositories/autoware.repos` with `repositories/autoware-nightly.repos` on top)
    - Dev container image: `ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy` (built 2026-11-10)
    - `autoware_carla_interface_v2`: the `main` branch from the Autoware Index (`jazzy` distribution)
    - CARLA: the 1.0 nightly build of `ue58-dev` (commit `d93dfef63`), with the CARLA changes in [CARLA server requirements](#carla-server-requirements)
    - `autoware_carla_scenario`: 28.1.1

## Requirements

- Ubuntu 24.04 on x86_64, with an NVIDIA GPU (CARLA recommends an RTX GPU and driver 550 or later)
- Docker with the NVIDIA Container Toolkit
- About 30 GiB of free disk space for CARLA, 20 GiB for the dev container image, and 10 GiB for the build
- [`vcs2l`](https://github.com/ros-infrastructure/vcs2l) (`vcs`), [`pipx`](https://pipx.pypa.io/), the [Hugging Face CLI](https://huggingface.co/docs/huggingface_hub/guides/cli) (`hf`), and [`uv`](https://docs.astral.sh/uv/)

### CARLA server requirements

Two changes to CARLA's ROS 2 interface are needed with current Autoware:

- [carla-simulator/carla#10206](https://github.com/carla-simulator/carla/pull/10206) (open as of 2026-11-19): without it, the LiDAR's `azimuth` field runs against the scan direction, and the ring outlier filter of Autoware Universe 0.53 or later drops every point. Localization then never initializes, and the pose initializer reports `align server failed`.
- [carla-simulator/carla#10216](https://github.com/carla-simulator/carla/pull/10216) (merged on 2026-11-12, so nightly builds after that date have it): without it, CARLA's native vehicle interface does not receive the commands of `control_command_gate`, and the ego does not move. Without it, use the client-side vehicle interface instead (see [Run a sample scenario](#run-a-sample-scenario)).

Until #10206 is merged, build CARLA's ROS 2 library (`libcarla-ros2-native.so`) from that pull request and replace the one in the package at `CarlaUnreal/Plugins/Carla/Binaries/Linux/`.

## Set up the workspace

1. Clone the Autoware repository and import the sources. `autoware-nightly.repos` moves Autoware Core, Autoware Universe, and `autoware_launch` to their `main` branches.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git ~/autoware
   cd ~/autoware
   mkdir -p src
   vcs import src < repositories/autoware.repos
   vcs import src < repositories/autoware-nightly.repos
   ```

2. Compose a `.repos` file for `autoware_carla_interface_v2` from the Autoware Index and import it. `aw-index-cli` also adds the packages it depends on from the Index (`tamagawa_imu_driver`).

   ```bash
   pipx install aw-index-cli
   aw-index-cli compose --rosdistro jazzy --packages autoware_carla_interface_v2 --stdout > autoware-index.repos
   mkdir -p src/index
   vcs import src/index < autoware-index.repos
   ```

## Build in the dev container

1. Start the `universe-devel-cuda` dev container. Open the workspace in VS Code and select **Dev Containers: Reopen in Container** with `universe-devel-cuda`, or start it with Docker Compose:

   ```bash
   mkdir -p ~/autoware_data
   cd ~/autoware
   HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose -p autoware \
     -f docker/devcontainer/universe-devel-cuda.compose.yaml up -d
   docker compose -p autoware -f docker/devcontainer/universe-devel-cuda.compose.yaml \
     exec -u aw autoware-devel bash
   ```

   The rest of this section runs in the container, in `/home/aw/autoware`.

   With Docker Compose, source ROS 2 in the container's `~/.bashrc` once, as VS Code does with the dev container's `postCreateCommand`. A shell started with `docker compose exec` does not run the image's entrypoint, so without this step ROS 2 is not sourced, and `colcon build` fails with `Could not find a package configuration file provided by "ament_cmake_core"`.

   ```bash
   echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

2. Remove the root-owned acados lock file that the image ships. Without this step, `autoware_trajectory_modifier` fails to build with `PermissionError: [Errno 13] Permission denied: '/tmp/acados_codegen_temporal.lock'`.

   ```bash
   sudo rm -f /tmp/acados_codegen_*.lock
   ```

3. Install the dependencies. `autoware_carla_interface_v2` depends on the CARLA client [typesafe_carla](https://github.com/hakuturu583/typesafe_carla) through the pip key `python3-typesafe-carla-pip`. The image activates the Python virtual environment `/opt/autoware-venv`, and `--as-root pip:false` makes rosdep run that environment's `pip` as the `aw` user instead of the system `pip` with `sudo`. The environment is owned by root, so `pip` installs into the user site (`~/.local`, with `Defaulting to user installation because normal site-packages is not writeable`), which the environment uses because it includes the system site packages.

   ```bash
   sudo apt-get update
   rosdep update --rosdistro jazzy
   rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --as-root pip:false
   ```

4. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   On a machine with less than 32 GiB of memory, limit the parallel jobs, for example with `MAKEFLAGS=-j3` and `--parallel-workers 4`.

## Download the map

Download the Autoware map of CARLA's `Town10HD_Opt` from the [`carla-ue5-maps`](https://huggingface.co/datasets/AutowareFoundation/carla-ue5-maps) dataset:

```bash
hf download AutowareFoundation/carla-ue5-maps --repo-type dataset \
  --revision c834d5cbcefee0bdf6d7cb3694a2cbb3cab512f6 \
  --include "autoware_maps/Town10HD_Opt/*" \
  --local-dir ~/autoware_data/maps
```

The map is placed in `~/autoware_data/maps/autoware_maps/Town10HD_Opt`. The dev container mounts `~/autoware_data` at `/home/aw/autoware_data`, and `autoware_carla_scenario` finds the map at the same place.

!!! warning

    Use this revision or a later one. The `demo_artifacts` Ansible role downloads revision `682ddd4`, which has no traffic lights or stop lines. With it, `autoware_carla_interface_v2` cannot match CARLA's traffic lights to the map, and logs `Traffic lights: 0 of 15 assigned to Autoware groups`.

## Start CARLA

1. Install CARLA. The [`carla` Ansible role](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/carla) downloads CARLA 1.0 or the nightly build to `~/autoware_simulation/carla/<version>`:

   ```bash
   cd ~/autoware
   ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
   ansible-playbook autoware.dev_env.install_dev_env --tags carla -e carla__version=nightly --ask-become-pass
   ```

   Apply the changes in [CARLA server requirements](#carla-server-requirements) to the installed package.

2. Raise the UDP socket buffers to 64 MB. Without this, the LiDAR stream overruns them.

   ```bash
   sudo sysctl -w net.core.rmem_max=67108864 net.core.wmem_max=67108864
   ```

3. Start the server with ROS 2. Use the same Cyclone DDS configuration as the dev container (`~/cyclonedds.xml` in the container, which uses the loopback interface only), so that `/clock` and the sensor data reach Autoware. Copy it from the container to the host, then start CARLA:

   ```bash
   docker cp autoware-autoware-devel-1:/home/aw/cyclonedds.xml ~/cyclonedds.xml
   cd ~/autoware_simulation/carla/nightly
   CYCLONEDDS_URI=file://$HOME/cyclonedds.xml ./CarlaUnreal.sh --ros2 -rmw=cyclonedds -RenderOffScreen -nosound
   ```

   Use two dashes in `--ros2`. With `-ros2`, the server starts with ROS 2 turned off, and prints no error.

## Run a sample scenario

`autoware_carla_scenario` runs a scenario and starts a fresh Autoware for it: its Docker launcher runs `autoware_carla_interface_v2`'s `autoware_carla_scenario.launch.xml` in a container made from the dev container image, with the workspace mounted. The `scenario_bridge` node in that container initializes localization, sets the route, and engages Autoware through the AD API.

!!! note

    The Docker launcher installs the workspace's dependencies into an image of its own. With the dev container images that have the Python virtual environment, this needs `autoware_carla_scenario` 28.1.1 or later ([autowarefoundation/autoware_carla_scenario#2197](https://github.com/autowarefoundation/autoware_carla_scenario/pull/2197)).

1. Set up `autoware_carla_scenario` on the host. CARLA 1.0 needs typesafe-carla 0.8.7 or later.

   ```bash
   git clone -b v28.1.1 https://github.com/autowarefoundation/autoware_carla_scenario.git ~/autoware_carla_scenario
   cd ~/autoware_carla_scenario
   uv sync
   uv pip install "typesafe-carla>=0.8.7,<0.9"
   ```

2. List the right turns of `Town10HD_Opt`. The example scenarios name lanelets of another map by default, so a run on `Town10HD_Opt` gives the lanelets of one of these cases.

   ```bash
   TYPESAFE_CARLA_CARLA_REF=1.0 uv run --no-sync scenario-expand \
     scenario=intersection_passing/right_turn map=town10hd_opt
   ```

3. Run the first case, a right turn from lanelet 3235 into lanelet 1281. Replace `~/autoware` with your workspace.

   ```bash
   TYPESAFE_CARLA_CARLA_REF=1.0 uv run --no-sync scenario \
     scenario=intersection_passing/right_turn map=town10hd_opt \
     ego.spawn_lanelet_id=3235 ego.spawn_s=7.0 \
     'scenario.expected_route_lanelet_ids=[4766,1281]' \
     ego.goal_lanelet_id=1281 ego.goal_s=70.0 \
     scenario.timeout_seconds=120 \
     ego.vehicle_type=vehicle.lincoln.mkz \
     ego.entity=autoware ego.native_autoware_interface=true \
     autoware.launcher.type=docker \
     autoware.launcher.workspace=$HOME/autoware \
     autoware.launcher.build=never \
     server.max_tick_rate_hz=20 autoware.ready_timeout_ticks=6000 \
     'autoware.launcher.launch=[autoware_carla_interface_v2,autoware_carla_scenario.launch.xml,"map_path:=/home/aw/autoware_data/maps/autoware_maps/Town10HD_Opt"]'
   ```

   The settings that this setup needs:

   | Setting                                | Why                                                                                                                                                                                                    |
   | -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
   | `TYPESAFE_CARLA_CARLA_REF=1.0`         | Selects the LibCarla of CARLA 1.0 in typesafe_carla                                                                                                                                                    |
   | `ego.goal_lanelet_id`, `ego.goal_s`    | Puts the goal 70 m into lanelet 1281. With the scenario's default goal, right after the turn, Autoware's goal planner finds no pull-over path and keeps the ego stopped before the junction            |
   | `ego.vehicle_type=vehicle.lincoln.mkz` | The vehicle that CARLA tuned the steering of its Autoware interface for                                                                                                                                |
   | `ego.native_autoware_interface=true`   | CARLA applies Autoware's control and publishes the vehicle status. Set `false` to let `autoware_carla_interface_v2` do it from the client, for example on a server without carla-simulator/carla#10216 |
   | `server.max_tick_rate_hz=20`           | Runs the simulation at about real time, as Autoware expects                                                                                                                                            |
   | `autoware.launcher.build=never`        | Uses the workspace that you built in the dev container. Leave it out to let the launcher build a workspace without `install/`                                                                          |

   The first run builds the dependency image (`autoware-carla-scenario-deps:<digest>`), which takes a few minutes. Autoware then takes about 50 seconds to start and localize, and the scenario starts:

   ```text
   The Autoware stack took the mission after 47.0 s; waiting, still at about real time, for it to localize
   Autoware reported ready via ReportReadiness
   [IntersectionPassingScenario] === Tick loop start ===
   [IntersectionPassingScenario] Result: PASSED — Entity 'Ego' is on road '5' (any lane) (s=0.20, t=-1.51) at 22.00s (22.00s)
   ```

   The results and the logs, including the output of Autoware (`autoware-000-intersection_passing.log`), are written under `outputs/<date>/<time>/`.

   This video shows the run, with CARLA's chase camera on the left and Autoware's RViz on the right:

   [![CARLA 1.0 with autoware_carla_interface_v2: a right turn on Town10HD_Opt](https://img.youtube.com/vi/3MP5VW7aoIE/maxresdefault.jpg)](https://youtu.be/3MP5VW7aoIE)

## Troubleshooting

| Symptom                                                                                                                     | Cause and fix                                                                                                                                                |
| --------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `took the mission but did not report in`, and `align server failed` in the Autoware log                                     | CARLA without carla-simulator/carla#10206. See [CARLA server requirements](#carla-server-requirements)                                                       |
| The ego does not move after Autoware engages                                                                                | CARLA without carla-simulator/carla#10216 with `ego.native_autoware_interface=true`. Use a newer nightly build, or set `ego.native_autoware_interface=false` |
| `Installing the dependencies of ... failed`, and `Could not find an activated virtualenv (required)` in `autoware-deps.log` | `autoware_carla_scenario` older than 28.1.1. See the note in [Run a sample scenario](#run-a-sample-scenario)                                                 |
| `Traffic lights: 0 of 15 assigned to Autoware groups` in the Autoware log                                                   | The map has no traffic lights. See [Download the map](#download-the-map)                                                                                     |
| `colcon build` fails with `Could not find a package configuration file provided by "ament_cmake_core"`                      | ROS 2 is not sourced in a shell started with `docker compose exec`. See [Build in the dev container](#build-in-the-dev-container)                            |
| No `/clock` or sensor topics in the container                                                                               | CARLA started with `-ros2` instead of `--ros2`, or without the Cyclone DDS configuration of the dev container                                                |
| The ego stops before the goal, and the goal planner logs `generated 0 pull over path candidates`                            | The goal is too close to the junction. Put it farther along the lane with `ego.goal_lanelet_id` and `ego.goal_s`                                             |
