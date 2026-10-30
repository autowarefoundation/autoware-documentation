# Detection area element

Behavior velocity planner's [detection area](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/) plans velocity
for a detection area on the map. If the module finds an obstacle (point cloud or predicted objects) in the detection area, it stops the vehicle before the stop line of the detection area.
In order to operate that, we will add a detection area element to our lanelet2 map.

## Creating a detection area element

In order to create a detection area on your map, please follow these steps:

1. Click `Lanelet2Maps` button on top panel.
2. Select `Detection Area` from the panel.
3. Please select lanelet which stop line to be added.
4. Click and insert `Detection Area` on your pointcloud map.
5. You can change the dimensions of the detection area with clicking points on the corners of the detection area. For more information, you can check the demonstration video.

You can see these steps in the detection area creating demonstration video:

![type:video](https://youtube.com/embed/RUJvXok-ncQ)

### Testing created detection area with planning simulator

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
3. We need to add pedestrians to detection area, so activate interactive pedestrians from `Tool Properties` panel on rviz.
4. After that, please press `Shift`, then click right click button for inserting pedestrians.
5. You can control inserted pedestrian via dragging right click. So, you should put pedestrian on the detection area for testing.

For more information about interactive dummy objects, see [placing interactive dummy objects](../../../../../demos/planning-sim/placing-objects.md#placing-interactive-dummy-objects).

Stop detection area on rviz:

<figure markdown>
  ![detection-area-test](images/detection-area-test.png){ align=center }
  <figcaption>
    Detection area test on the created map.
  </figcaption>
</figure>

You can check your detection area elements in the planning simulator as this demonstration video:

![type:video](https://youtube.com/embed/zjfPnRIz8Xk)
