# Planning simulation

## Preparation

### Download the sample map

```bash
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
```

- You can also download [the map](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view?usp=sharing) manually.

!!! info

    Sample map: Copyright 2020 TIER IV, Inc.

### Make sure the artifacts are downloaded

Check if you have `~/autoware_data` folder and files in it.

```bash
$ cd ~/autoware_data
$ ls -C -w 30
image_projection_based_fusion
lidar_apollo_instance_segmentation
lidar_centerpoint
tensorrt_yolo
tensorrt_yolox
traffic_light_classifier
traffic_light_fine_detector
traffic_light_ssd_fine_detector
yabloc_pose_initializer
```

If not, please, follow [Manual downloading of artifacts](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts).

## Launch Autoware Planning Simulator

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

!!! warning

    Note that you cannot use `~` instead of `$HOME` here.

    If `~` is used, the map will fail to load.

---

!!! info "[Using Autoware Launch GUI](using-launch-gui.md)"

    If you prefer a graphical user interface (GUI) over the command line for launching and managing your simulations, refer to the [Using Autoware Launch GUI](using-launch-gui.md) section at the end of this document for a step-by-step guide.

---

<div style="text-align: center;" markdown="1">

[:fa-cl-s fa-film: Reference Video Tutorials](https://drive.google.com/file/d/1bs_dX1JJ76qHk-SGvS6YF9gmekkN8fz7/view?usp=sharing){ .md-button style="margin: 5px" }

</div>

---

## Basic simulations

<div style="text-align: center;" markdown="1">

[Lane Driving](lane-driving.md){ .md-button style="margin: 5px" }
[Parking](parking.md){ .md-button style="margin: 5px" }
[Pull out and pull over](pull-over-out.md){ .md-button style="margin: 5px" }
[Lane Change](lane-change.md){ .md-button style="margin: 5px" }

</div>

## Advanced simulations

<div style="text-align: center;" markdown="1">

[:fa-cl-s fa-cube: Placing Dummy Objects](placing-objects.md){ .md-button style="margin: 5px" }
[Avoidance](avoidance.md){ .md-button style="margin: 5px" }
[Traffic Light Recognition Simulation](traffic-light.md){ .md-button style="margin: 5px" }
[Driving Through a Crosswalk](crosswalk.md){ .md-button style="margin: 5px" }

</div>

## Increase the maximum velocity

The original Autoware is designed to operate at a wide speed range. But for safety reasons, the default maximum velocity has been limited to **15 km/h**.
Because of this, even if you drag the slider to a higher speed in the RViz panel, the system will not allow it.

To run Autoware at a higher speed, you can modify the `max_vel` parameter in the config file [autoware_launch/config/planning/scenario_planning/common/common.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/common.param.yaml) located in the `autoware_launch` repository.

!!! example

    Set `max_vel` to `20.0` (20 m/s = 72 km/h).
    Then launch the planning simulator, place the vehicle and set the velocity limit with the slider.

![increase-max-velocity](images/others/increase-max-velocity.png)

## Create your own map

The content above takes place in the planning simulator using a sample map. If you are interested in running Autoware with maps of your own environment, please visit the [How to Create Vector Map](../../tutorials/integrating-autoware/creating-maps/index.md) section for guidance.
