# Overview

## Requirement: prepare your real vehicle hardware

Prerequisites for the vehicle:

- An onboard computer that satisfies the [Autoware installation prerequisites](../../installation/autoware/source-installation.md#prerequisites)
- The following devices attached
  - Drive-by-wire interface
  - LiDAR
  - Optional: Inertial measurement unit
  - Optional: Camera
  - Optional: GNSS

## 1. Creating your Autoware meta-repository

Create your Autoware meta-repository.
One easy way is to fork [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware) and clone it.
For how to fork a repository, refer to [GitHub Docs](https://docs.github.com/en/get-started/quickstart/fork-a-repo).

```bash
git clone https://github.com/YOUR_NAME/autoware.git
```

If you set up multiple types of vehicles, adding a suffix like "autoware.vehicle_A" or "autoware.vehicle_B" is recommended.

## 2. Creating your vehicle and sensor description

Next, you need to create description packages that define the vehicle and sensor configuration of your vehicle.

In Autoware 2.0.0, the sample vehicle and sensor kit packages are in the [autoware_launch](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0) repository. The separate `sample_vehicle_launch` and `sample_sensor_kit_launch` repositories are archived. Fork `autoware_launch` and add these two packages to your fork:

- YOUR_VEHICLE_launch in `vehicle/` (copy [sample_vehicle_launch](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/vehicle/sample_vehicle_launch) as an example)
- YOUR_SENSOR_KIT_launch in `sensor_kit/` (copy [sample_sensor_kit_launch](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/sensor_kit/sample_sensor_kit_launch) as an example)

Autoware finds these packages by the `vehicle_model` and `sensor_model` launch arguments. For example, `vehicle_model:=YOUR_VEHICLE` loads `YOUR_VEHICLE_description` and `YOUR_VEHICLE_launch`, and `sensor_model:=YOUR_SENSOR_KIT` loads `YOUR_SENSOR_KIT_description` and `YOUR_SENSOR_KIT_launch`.

Once created, you need to update the `repositories/autoware.repos` file of your cloned Autoware repository to refer to your fork of `autoware_launch`.

```diff
  launcher/autoware_launch:
    type: git
-   url: https://github.com/autowarefoundation/autoware_launch.git
-   version: 0.53.0
+   url: https://github.com/YOUR_NAME/autoware_launch.git
+   version: main
```

You can also keep your vehicle and sensor kit packages in separate repositories. In this case, add them to `repositories/autoware.repos` too. For more information, see [Creating Autoware repositories](creating-your-autoware-repositories/creating-autoware-repositories.md).

### Adapt YOUR_VEHICLE_launch for autoware launching system

#### At YOUR_VEHICLE_description

Define URDF and parameters in the vehicle description package (refer to the [sample vehicle description package](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/vehicle/sample_vehicle_launch/sample_vehicle_description) for an example).

#### At YOUR_VEHICLE_launch

Create `launch/vehicle_interface.launch.xml` that launches your vehicle interface (refer to the [sample vehicle launch package](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/vehicle/sample_vehicle_launch/sample_vehicle_launch) for example).
If you have multiple vehicles with the same hardware setup, you can specify `vehicle_id` to distinguish them.

### Adapt YOUR_SENSOR_KIT_launch for autoware launching system

#### At YOUR_SENSOR_KIT_description

Define URDF and extrinsic parameters for all the sensors here (refer to the [sample sensor kit description package](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description) for example).
Note that you need to calibrate extrinsic parameters for all the sensors beforehand.

#### At YOUR_SENSOR_KIT_launch

Create `launch/sensing.launch.xml` that launches the interfaces of all the sensors on the vehicle. (refer to the [sample sensor kit launch package](https://github.com/autowarefoundation/autoware_launch/tree/0.53.0/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch) for example).

!!! note

    At this point, you are now able to run Autoware's Planning Simulator to do a basic test of your vehicle and sensing packages.
    To do so, you need to build and install Autoware using your cloned repository. Follow the [steps for either Docker or source installation](../../installation/index.md) (starting from the dependency installation step) and then run the following command:

    ```bash
    ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=YOUR_VEHICLE sensor_model:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
    ```

## 3. Create a `vehicle_interface` package

You need to create an interface package for your vehicle.
The package is expected to provide the following two functions.

1. Receive command messages from `vehicle_cmd_gate` and drive the vehicle accordingly
2. Send vehicle status information to Autoware

You can find detailed information about the requirements of the `vehicle_interface` package in the [Vehicle Interface design documentation](../../design/autoware-architecture-v1/interfaces/components/vehicle-interface.md).
For a step-by-step guide, see the [Vehicle interface](creating-vehicle-interface-package/vehicle-interface.md) tutorial.
You can also refer to TIER IV's [pacmod_interface repository](https://github.com/tier4/pacmod_interface) as an example of a vehicle interface package.

## 4. Create maps

You need both a pointcloud map and a vector map in order to use Autoware.
For more information, see the [map design documentation](../../design/autoware-architecture-v1/components/map/index.md).

### Create a pointcloud map

Use third-party tools such as a LiDAR-based SLAM (Simultaneous Localization And Mapping) package to create a pointcloud map in the `.pcd` format.
For more information, see [Creating maps](creating-maps/index.md).

### Create vector map

Use third-party tools such as [TIER IV's Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) to create a Lanelet2 format `.osm` file.

## 5. Launch Autoware

This section briefly explains how to run your vehicle with Autoware.

### Install Autoware

Follow the [installation steps of Autoware](../../installation/index.md).

### Launch Autoware

Launch Autoware with the following command:

```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_model:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
```

### Set initial pose

If GNSS is available, Autoware automatically initializes the vehicle's pose.

If not, you need to set the initial pose using the RViz GUI.

1. Click the 2D Pose Estimate button in the toolbar, or hit the P key
2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the initial pose.

### Set goal pose

Set a goal pose for the ego vehicle.

1. Click the 2D Goal Pose button in the toolbar, or hit the G key
2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the goal pose.
   If successful, you will see the calculated planning path on RViz.

### Engage

In your terminal, execute the following command. It calls the [AD API](../../design/autoware-architecture-v1/interfaces/ad-api/index.md) service that changes the operation mode to autonomous.

```bash
source ~/autoware.YOURS/install/setup.bash
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
```

You can also engage via RViz with "AutowareStatePanel". When the route is calculated, press the **AUTO** button in the panel.
The Autoware RViz configuration includes the panel. You can also add it from `Panels > Add New Panel > tier4_state_rviz_plugin > AutowareStatePanel`.

![Autoware State Panel](launch-autoware/images/autoware_state_panel_before.png)

Now the vehicle should drive along the calculated path!

## 6. Tune parameters for your vehicle & environment

You may need to tune your parameters depending on the domain in which you will operate your vehicle.

The maximum velocity is the `max_vel` parameter in [`common.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/0.53.0/autoware_universe_launch/autoware_planning_config/config/scenario_planning/common/common.param.yaml#L16) of `autoware_planning_config`. It is 4.17 m/s (15 km/h) by default.
