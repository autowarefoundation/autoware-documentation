# CARLA simulator support

[CARLA](https://carla.org) is an open-source simulator for autonomous driving research. Autoware connects to CARLA through the [`autoware_carla_interface`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/simulator/autoware_carla_interface) package of Autoware Universe. This page shows what Autoware 2.0.0 supports, how the support grew, and what is still missing.

{{ diagram("images/carla-bridge.svg", "The bridge sends CARLA sensor data, vehicle status, and the clock to Autoware, and sends the control commands of Autoware back to CARLA.") }}

## Status in Autoware 2.0.0

| Part             | Where                                                                                                                                      | What it does                                                                                                                                                                                                                                               |
| :--------------- | :----------------------------------------------------------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Bridge           | `autoware_carla_interface` in `autoware_universe` 0.53.1                                                                                   | Connects to the CARLA server, spawns the ego vehicle, publishes the sensor data and the vehicle status, and sends the control commands to CARLA. It runs `autoware_raw_vehicle_cmd_converter` with a calibration for the `vehicle.toyota.prius` blueprint. |
| Sensor kit       | `carla_sensor_kit_launch` and `carla_sensor_kit_description` in `autoware_launch` 0.53.0                                                   | Six cameras for 360-degree coverage, a top LiDAR, a GNSS receiver, and an IMU.                                                                                                                                                                             |
| Launch file      | `e2e_simulator.launch.xml` in `autoware_launch` 0.53.0                                                                                     | `simulator_type:=carla` includes the bridge. `launch_simulator_interface:=false` leaves it out, so that it can run in another process or container.                                                                                                        |
| Docker demo      | [`docker/examples/demos/carla`](https://github.com/autowarefoundation/autoware/tree/2.0.0/docker/examples/demos/carla) in `autoware` 2.0.0 | Runs CARLA 0.9.16 and Autoware in four Docker Compose services: `carla-simulator`, `carla-interface`, `spectator-follow`, and `autoware`. The Autoware services use the `universe-cuda-jazzy` image.                                                       |
| Maps             | The `demo_artifacts` Ansible role in `autoware` 2.0.0                                                                                      | Downloads the `carla-kashiwanoha` map for the Docker demo, and the `Town10HD_Opt` map for CARLA 0.10. The Docker demo of Autoware 2.0.0 still loads `Town01`, which you download yourself.                                                                 |
| End-to-end model | `autoware_tensorrt_vad` in `autoware_universe` 0.53.1                                                                                      | Runs the VAD model, which is trained on CARLA simulation data. See [Autoware Architecture 2.0](../../../design/autoware-architecture-v2/index.md).                                                                                                         |

The README of `autoware_carla_interface` lists Ubuntu 22.04, ROS 2 Humble, and CARLA 0.9.15 as the supported environment. The Docker demo of Autoware 2.0.0 runs ROS 2 Jazzy with CARLA 0.9.16.

## Run Autoware with CARLA

Two ways are available:

- **Docker demo:** Follow the [README of the CARLA demo](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/examples/demos/carla/README.md). You need an NVIDIA GPU with the NVIDIA Container Toolkit, Docker Compose v2, a CARLA Lanelet2 map such as `Town01` in `~/autoware_data/maps/Town01`, and the perception models in `~/autoware_data/ml_models`. Then run these commands from the root of the `autoware` repository:

  ```bash
  xhost +local:docker
  cd docker/examples/demos/carla
  HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up
  ```

- **Source workspace:** Install CARLA, the CARLA Python API, and the maps as the [README of `autoware_carla_interface`](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/simulator/autoware_carla_interface/README.md) explains. Start the CARLA server, then launch Autoware:

  ```bash
  ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/autoware_data/maps/Town01 \
    vehicle_model:=sample_vehicle \
    sensor_model:=carla_sensor_kit \
    simulator_type:=carla
  ```

In RViz, set the initial pose with **Init by GNSS**, set a goal, wait for the route, and engage.

The [CARLA simulator demo page](../../../demos/digital-twin-demos/carla-tutorial.md) also lists community projects that connect Autoware and CARLA.

## Timeline

1. **2024 May:** `e2e_simulator.launch.xml` gets an argument to launch the CARLA interface ([autoware_launch#924](https://github.com/autowarefoundation/autoware_launch/pull/924)).
2. **2024 July:** The `autoware_carla_interface` package is added to Autoware Universe ([autoware_universe#6859](https://github.com/autowarefoundation/autoware_universe/pull/6859)).
3. **2025 October:** The CARLA sensor kit packages are added to `autoware_launch` ([autoware_launch#1655](https://github.com/autowarefoundation/autoware_launch/pull/1655)).
4. **2026 May:** The `launch_simulator_interface` argument makes the bridge optional in `e2e_simulator.launch.xml` ([autoware_launch#1836](https://github.com/autowarefoundation/autoware_launch/pull/1836)).
5. **2026 August:** The Docker Compose demo for CARLA is added ([autoware#7264](https://github.com/autowarefoundation/autoware/pull/7264)).
6. **2026 September:** The `demo_artifacts` role downloads the Kashiwanoha map ([autoware#7301](https://github.com/autowarefoundation/autoware/pull/7301)) and the CARLA 0.10 `Town10HD_Opt` map ([autoware#7308](https://github.com/autowarefoundation/autoware/pull/7308)).
7. **2026 October:** Autoware 2.0.0 is released with all of the above.

## Next steps

- **CARLA 0.10:** CARLA 0.10 uses Unreal Engine 5. Autoware 2.0.0 does not run it yet. PyPI has no CARLA Python client after 0.9.16. The pedal map and the vehicle list of the bridge are for CARLA 0.9, and the default ego vehicle `vehicle.toyota.prius` does not exist in CARLA 0.10. The 0.10 maps of `demo_artifacts` wait for this support. For details, see [Maps for CARLA 0.10](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/examples/demos/carla/README.md#maps-for-carla-010).
- **Traffic lights:** The CARLA Lanelet2 maps do not have traffic light elements for Autoware. Add them to the map, as the [README of the bridge](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/simulator/autoware_carla_interface/README.md) explains. Then you can set `traffic_light.publish:=true`, so that the bridge publishes the traffic light states of CARLA instead of camera recognition. The bridge matches each CARLA light to a traffic light element of the map, for example by position with `traffic_light.map_path`.
- **Closed-loop evaluation:** [`autoware_carla_scenario`](https://github.com/autowarefoundation/autoware_carla_scenario) is a scenario test framework for Autoware on CARLA. The design discussion [CARLA × Autoware integration architecture for closed-loop evaluation](https://github.com/orgs/autowarefoundation/discussions/7277) proposes how the CARLA tools fit together. See also [Assessment of safety and benchmarks](../../../design/autoware-architecture-v2/roadmap/assessment-safety-and-benchmarks.md).
