# Speed bump

Behavior velocity planner's [speed bump module](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/) plans velocity
to slow down before speed bump for comfortable and safety driving.
In order to operate that, we will add speed bumps to our lanelet2 map.

## Creating a speed bump element

In order to create a speed bump on your pointcloud map, please follow these steps:

1. Select `Linestring` from Lanelet2Maps section.
2. Click and draw polygon for speed bump.
3. Then please disable `Linestring` from Lanelet2Maps section.
4. CLick `Change to Polygon` from the `Action` panel.
5. Please select this Polygon and enter `speed_bump` as the type.
6. Then, please click lanelet which speed bump to be added.
7. Select `Create General Regulatory ELement`.
8. Go to this element, and please enter `speed_bump` as subtype.
9. Click `Add refers` and type your created speed bump polygon ID.

You can see these steps in the speed bump creating demonstration video:

![type:video](https://youtube.com/embed/EenccStyZVg)

### Testing created the speed bump element with planning simulator

After the completing of creating the map, we need to save it.
To that please click `File` --> `Export Lanelet2Maps` then download.

After the download is finished,
we need to put lanelet2 map and pointcloud map on the same location.
The directory structure should be like this:

```text
<YOUR-MAP-DIRECTORY>/
 ├─ pointcloud_map.pcd
 ├─ lanelet2_map.osm
 └─ map_projector_info.yaml
```

The `map_projector_info.yaml` file tells Autoware the projection of the map. Write the MGRS grid that you entered when you created the Lanelet2 map in Vector Map Builder:

```yaml
projector_type: MGRS
vertical_datum: WGS84
mgrs_grid: <YOUR-MGRS-GRID> # for example, 35TPF
```

Use the vertical datum of your point cloud map. For a point cloud map from the [UTM to MGRS converter](../../converting-utm-to-mgrs-map/index.md), use `EGM2008`. If the file does not exist, Autoware reads the MGRS grid from the Lanelet2 map. This fallback is deprecated.

If your .osm or .pcd map file's name is different from these names,
add the `lanelet2_map_file` and `pointcloud_map_file` arguments to the launch command:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<YOUR-MAP-FOLDER-DIR> vehicle_model:=<YOUR-VEHICLE-MODEL> sensor_model:=<YOUR-SENSOR-KIT> lanelet2_map_file:=<YOUR-LANELET-MAP-NAME>.osm pointcloud_map_file:=<YOUR-POINTCLOUD-MAP-NAME>.pcd
```

!!! note

    The speed bump module is not enabled by default. To enable it, set the default value of `launch_speed_bump_module` to `"true"` in [default_preset.yaml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/preset/default_preset.yaml). If you did not build Autoware with `--symlink-install`, build the `autoware_planning_config` package again. The module parameters are in [speed_bump.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_universe_launch/autoware_planning_config/config/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/speed_bump.param.yaml).

Now we are ready to launch the planning simulator:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<YOUR-MAP-FOLDER-DIR> vehicle_model:=<YOUR-VEHICLE-MODEL> sensor_model:=<YOUR-SENSOR-KIT>
```

Example for tutorial_vehicle:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_data/maps/tutorial_map/ vehicle_model:=tutorial_vehicle sensor_model:=tutorial_vehicle_sensor_kit vehicle_id:=tutorial_vehicle
```

1. Click `2D Pose Estimate` button on rviz or press `P` and give a pose for initialization.
2. Click `2D Goal Pose` button on rviz or press `G` and give a pose for goal point.
3. You can see the speed bump marker on the rviz screen.

Speed bump markers on rviz:

<figure markdown>
  ![speed-bump-test](images/speed-bump-test.png){ align=center }
  <figcaption>
    Speed bump test on the created map.
  </figcaption>
</figure>

You can check your speed bump elements in the planning simulator as this demonstration video:

![type:video](https://youtube.com/embed/rg_a-ipdNAY)
