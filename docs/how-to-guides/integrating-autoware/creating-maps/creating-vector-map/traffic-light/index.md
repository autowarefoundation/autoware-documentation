# Traffic light

Behavior velocity planner's [traffic light module](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_velocity_traffic_light_module/) plans velocity
according to the traffic light status.
In order to operate that, we will add traffic light attribute to our lanelet2 map.

## Creating a traffic light regulatory element

In order to create a traffic light on your pointcloud map, please follow these steps:

1. Please select lanelet which traffic light to be added.
2. Click `Abstraction` button on top panel.
3. Select `Traffic Light` from the panel.
4. Click on the desired area for inserting traffic light.

You can see these steps in the traffic-light creating demonstration video:

![type:video](https://youtube.com/embed/P3xcayPkTOg)

### Testing created the traffic light element with planning simulator

After the completing of creating the map, we need to save it.
To that please click `File` --> `Export Lanelet2Maps` then download.

After the download is finished,
we need to put lanelet2 map and pointcloud map on the same location.
The directory structure should be like this:

```diff
+ <YOUR-MAP-DIRECTORY>/
+  ├─ pointcloud_map.pcd
+  └─ lanelet2_map.osm
```

If your .osm or .pcd map file's name is different from these names,
you need to update autoware.launch.xml:

```diff
  <!-- Map -->
-  <arg name="lanelet2_map_file" default="lanelet2_map.osm" description="lanelet2 map file name"/>
+  <arg name="lanelet2_map_file" default="<YOUR-LANELET-MAP-NAME>.osm" description="lanelet2 map file name"/>
-  <arg name="pointcloud_map_file" default="pointcloud_map.pcd" description="pointcloud map file name"/>
+  <arg name="pointcloud_map_file" default="<YOUR-POINTCLOUD-MAP-NAME>.pcd" description="pointcloud map file name"/>
```

Now we are ready to launch the planning simulator:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<YOUR-MAP-FOLDER-DIR> vehicle_model:=<YOUR-VEHICLE-MODEL> sensor_model:=<YOUR-SENSOR-KIT>
```

Example for tutorial_vehicle:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Files/autoware_map/tutorial_map/ vehicle_model:=tutorial_vehicle sensor_model:=tutorial_vehicle_sensor_kit vehicle_id:=tutorial_vehicle
```

1. Click `2D Pose Estimate` button on rviz or press `P` and give a pose for initialization.
2. Click `Panels` -> `Add new panel`, select `TrafficLightPublishPanel`, and then press `OK`.
3. In TrafficLightPublishPanel, set the ID and color of the traffic light.
4. Then, Click `SET` and `PUBLISH` button.
5. Click `2D Goal Pose` button on rviz or press `G` and give a pose for goal point.
6. You can see the traffic light marker on the rviz screen if you set the traffic light color as `RED`.

Traffic Light markers on rviz:

<figure markdown>
  ![traffic-light-test](images/traffic-light-test.png){ align=center }
  <figcaption>
    Traffic light test on the created map.
  </figcaption>
</figure>

You can check your traffic light elements in the planning simulator as this demonstration video:

![type:video](https://youtube.com/embed/AaFT24uqbJk)
