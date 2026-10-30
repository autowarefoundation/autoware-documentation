# Driving Log Replayer

[Driving Log Replayer v2](https://github.com/tier4/driving_log_replayer_v2) (`driving_log_replayer_v2`) is an evaluation tool for Autoware, developed by TIER IV. It plays back a recorded rosbag into Autoware, runs `logging_simulator.launch.xml` or `planning_simulator.launch.xml`, and evaluates the output topics of Autoware against the conditions of a scenario file. Use it to measure the performance of a component and for regression tests.

!!! note

    The first version, [`driving_log_replayer`](https://github.com/tier4/driving_log_replayer), is archived. Use `driving_log_replayer_v2`.

## How it works

1. You write a scenario file (`scenario.yaml`). It names the use case, the vehicle and sensor models, the dataset, and the pass conditions.
2. Driving Log Replayer launches Autoware and plays back the rosbag of the dataset.
3. An evaluation node reads the output topics of Autoware and checks them against the conditions.
4. The result goes to a `result.jsonl` file (one JSON object per line) and a result bag.

Each evaluation type is a use case. For example, `localization`, `perception`, `obstacle_segmentation`, `traffic_light`, `planning_control`, and `diagnostics` are use cases. For the full list, see [Evaluation use cases](https://tier4.github.io/driving_log_replayer_v2/use_case/).

## Requirements

- An Autoware workspace that you built from source. See [Source installation](../../installation/autoware/source-installation.md).
- Ubuntu 22.04 with ROS 2 Humble and Python 3.10, or Ubuntu 24.04 with ROS 2 Jazzy and Python 3.12.
- An NVIDIA GPU, if you evaluate perception.
- The `zstd` tool, to extract the sample data: `sudo apt install zstd`.
- The `gdown` tool, to download the sample data from Google Drive.

## Build Driving Log Replayer

1. Go to the Autoware workspace:

   ```bash
   cd ~/autoware
   ```

2. Create a file `driving_log_replayer.repos` in the workspace with the dependencies of Driving Log Replayer:

   ```yaml
   repositories:
     simulator/perception_eval:
       type: git
       url: https://github.com/tier4/autoware_perception_evaluation.git
       version: main
     simulator/driving_log_replayer_v2:
       type: git
       url: https://github.com/tier4/driving_log_replayer_v2.git
       version: main
     simulator/vendor/ros2_numpy:
       type: git
       url: https://github.com/Box-Robotics/ros2_numpy.git
       version: humble # or jazzy
     simulator/vendor/ros2bag_extensions:
       type: git
       url: https://github.com/tier4/ros2bag_extensions.git
       version: main
   ```

3. Import the repositories. Driving Log Replayer also needs `autoware_tools`, which `repositories/tools.repos` imports:

   ```bash
   vcs import src < driving_log_replayer.repos
   vcs import src < repositories/tools.repos
   ```

4. Install the dependent ROS packages. On Ubuntu 24.04, replace `humble` with `jazzy`:

   ```bash
   source /opt/ros/humble/setup.bash
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

5. Build the workspace:

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

!!! warning

    The `main` branch of `driving_log_replayer_v2` follows the latest Autoware. If it does not build with Autoware 2.0.0, use a [release tag](https://github.com/tier4/driving_log_replayer_v2/tags) of `driving_log_replayer_v2` from the time of the Autoware release.

## Run the sample localization evaluation

The sample dataset and the sample map are copyright 2022 and 2020 TIER IV, Inc.

1. Download and extract the sample dataset and the sample map:

   ```bash
   mkdir -p ~/driving_log_replayer_v2
   gdown -O ~/driving_log_replayer_v2/sample_dataset_v2.tar.zst 'https://docs.google.com/uc?export=download&id=1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC'
   tar -I zstd -xvf ~/driving_log_replayer_v2/sample_dataset_v2.tar.zst -C ~/driving_log_replayer_v2/
   gdown -O ~/driving_log_replayer_v2/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/driving_log_replayer_v2/ ~/driving_log_replayer_v2/sample-map-planning.zip
   mv ~/driving_log_replayer_v2/sample-map-planning ~/driving_log_replayer_v2/sample_dataset/map
   ```

2. Copy the sample localization scenario next to the dataset:

   ```bash
   cp ~/autoware/src/simulator/driving_log_replayer_v2/sample/localization/scenario.yaml ~/driving_log_replayer_v2/localization.yaml
   ```

3. Run the evaluation:

   ```bash
   source ~/autoware/install/setup.bash
   ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=$HOME/driving_log_replayer_v2/localization.yaml
   ```

The localization use case launches the logging simulator without perception, planning, and control. When the rosbag ends, the results are in `~/driving_log_replayer_v2/out/<date-time>/`. The `out/latest` link points to the newest run.

For the other use cases and their sample data, see the [setup guide](https://tier4.github.io/driving_log_replayer_v2/quick_start/setup/) of Driving Log Replayer v2.

## Run many scenarios

The `dlr2` command-line tool runs all scenarios in a folder, one after the other. Install it with `pipx`:

```bash
pipx install git+https://github.com/tier4/driving_log_replayer_v2.git
dlr2 simulation run <scenario-root>
```

Each subfolder of `<scenario-root>` must hold one `scenario.yaml` and its datasets. For the details, see [Command](https://tier4.github.io/driving_log_replayer_v2/overview/command/).

## Learn more

- [Driving Log Replayer v2 documentation](https://tier4.github.io/driving_log_replayer_v2/)
- [Scenario format](https://tier4.github.io/driving_log_replayer_v2/scenario_format/)
- [Result format](https://tier4.github.io/driving_log_replayer_v2/result_format/)
