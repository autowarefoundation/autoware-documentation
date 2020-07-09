SVL simulator {#lgsvl}
========

@tableofcontents

# SVL simulator: running the SVL simulator alongside Autoware.Auto

SVL is a Unity-based multi-robot simulator for autonomous vehicle developers. It provides a simulated world to

- create sensor inputs to Autoware.Auto,
- allow the user to manually steer the ego vehicle similar to a computer game,
- place other moving traffic participants in a scene.

For more information about the simulator, see [https://www.svlsimulator.com/docs/getting-started/getting-started/](https://www.svlsimulator.com/docs/getting-started/getting-started/).

# Requirements

The following guide assumes that the SVL simulator will be run from inside an ADE container, although it is not strictly required.

- ADE 4.2.0 or later. Follow the
[ADE installation instructions](https://ade-cli.readthedocs.io/en/latest/install.html) to install it
- NVidia graphics card
- If using Docker engine version 19.03 or later, [install Native GPU Support](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support)).
- If using Docker engine with a version less than 19.03, either upgrade Docker or [install nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))
- Cyclone DDS is the DDS vendor; see @ref choosing-a-dds-vendor

# Using the simulator

Using the simulator involves the following steps:

-# Launch it
-# Configure cluster (one time step)
-# Choose or create a simulation
-# Start the simulation

This section outlines these steps. You also need an SVL account to use the simulator, if you do not have one, create it now on `https://wise.svlsimulator.com/`.

## Launching the simulator

Install ADE as described in the [installation section](@ref installation-ade):

Start ADE with the SVL volume:

```{bash}
$ cd ~/adehome/AutowareAuto
$ ade --rc .aderc-amd64-foxy-lgsvl start --update --enter
```

Pick a different `.aderc-*-lgsvl` file to manually choose a ROS version.

To start the SVL simulator, in the same terminal window:

```{bash}
ade$ /opt/lgsvl/simulator
```

### Troubleshooting

In case the simulator window opens up with a black screen and the application immediately terminates, have a look at the log file at

```{bash}
~/.config/unity3d/LGElectronics/SVLSimulator/Player.log
```

One possible fix is to remove conflicting graphics drivers from ADE with

```{bash}
ade$ sudo apt remove mesa-vulkan-drivers
```
and launch the simulator again.


If point cloud data or image data is not being visualized in rviz but other data such as bounding box is visible run the following command inside ade,
```{bash}
ade$ sudo apt update ; sudo apt dist-upgrade
```

## Configure the cluster

You need to make your ADE environment a valid SVL cluster in order to launch any simulations. This is a one time configuration step. 

On your first simulator run there should be a window with only one button: `LINK TO CLOUD`. Click it and a web browser with `https://wise.svlsimulator.com/` should open. You can create a new cluster there by providing a cluster name and clicking `Create cluster` button.

More about linking to cloud: [documentation](https://www.svlsimulator.com/docs/installation-guide/installing-simulator/#linktocloud).

## Creating a simulation

Creating a simulation takes only a few clicks in the browser. The following steps assume that the launch was successful and illustrate the configuration process with the setup for the @ref avpdemo.

Start your browser and go to [SVL simulator web interface](https://wise.svlsimulator.com/). You should have your account set up already, so sign in using the button on the top right of the screen.

### Choosing a map

The goal is to add `AutonomouStuff` parking lot map to your library. If that map is already in your library then nothing needs to be done.

Adding a map to your library:
- Go to `Store` -> `Maps`.
- Click `+` button next to `AutonomouStuff` map (you can use search bar to filter by name).

@image html images/svl-map.png "Choosing a map" width=50%

### Configuring a vehicle {#lgsvl-configuring-vehicle}

The goal is to add `AWFLexus2016RXHybrid` vehicle to your library. If this vehicle is already in your library then nothing needs to be done.

Adding a vehicle to your library:
- Go to `Store` -> `Vehicles`.
- Click `+` button next to `AWFLexus2016RXHybrid` vehicle (you can use search bar to filter by name).

@image html images/svl-vehicle.png `Adding a vehicle` width=50%

### Adding ROS2ForUnitySVLBridge

The goal is to add native ROS2 bridge to your library. If this bridge is already in your library then nothing needs to be done:
- Go to [ROS2ForUnitySVLBridge](https://wise.svlsimulator.com/plugins/profile/d490cb21-2a44-447f-a289-7d869f23aabf) plugin page.
- Click `Add to Library` button.

You can also search for `ROS2ForUnitySVLBridge` using the search bar.

### Configure vehicle sensors

Once you added vehicle to your library:
- Go to `Library` -> `Vehicles`.
- Click on the `AWFLexus2016RXHybrid` portrait. You will be forwarded to a vehicle edit page.
- Click a button near `Sensor Configurations` section to modify sensor configurations.

@image html images/svl-sensors.png "Configuring sensors"

Notice that there is already an `Autoware.Auto` configuration. To make sure that we are running the newest version possible, it is better to create a new one and use the most recent version of sensor configuration file:
- Click `+ Create New Configuration` button at the bottom of the page. 
- Set a configuration name and pick `ROS2ForUnitySVLBridge` as a bridge. This is a native implementation of ROS2 bridge.
- Confirm.

In the configuration edit view:
- Click `{...}` symbol near Visual Editor (preview) window.
- Paste contents of [avp-sensors.json](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/launch/autoware_demos/config/svl/avp-sensors.json) file inside edit window. `Configurator` window should populate with bunch of sensors now. 
- Click `Save` to save configuration.

@image html images/svl-sensors-json.png "Json configuration file" width=40%

That’s it. Now you have a vehicle with a valid configuration.

### Choosing/creating a simulation

The SVL simulator lets you store and reuse multiple simulation configurations. To use an existing simulation, navigate to `Simulations` tab and press the "Run Simulation" button for desired instance. The simulator should now start in the SVL window.

To create a new simulation, follow the below steps:

- Switch to the `Simulations` tab and click the `Add new` button.
- Enter a name, description and select a cluster. Click `Next`.
- Select the `Random Traffic` runtime template from the drop-down menu.
- Select `AutonomouStuff` map and `AWFLexus2016RXHybrid` vehicle with your sensor configuration. Click `Next`.
- Select `Other ROS 2 Autopilot` autopilot and leave `Bridge Connection` with default value. 
- Click `Next` and then `Publish`.

You can visit [SVL documentation](https://www.svlsimulator.com/docs/user-interface/web/simulations/) for more in-depth description.

## Starting the simulation {#lgsvl-start-simulation}

Once the simulation has been created, you can run it by clicking the `Run Simulation` button next to the simulation configuration widget in `Simulations` view.

@image html images/svl-simulation-start.png "Starting the simulation" width=40%

The Lexus should appear in the `SVL Simulator` window (not in the browser).

The next step is to control the Lexus and to drive around. Press `F1` to see a list of shortcuts and press the cookie button in bottom left corner for more UI controls.

The essential commands are to use the arrow keys to steer and accelerate, and the `Page Up` and `Page Down` keys to switch between forward and reverse driving.

Congratulations if everything is working up to this point. The setup of SVL is completed.

@image html images/lgsvl-controls.png "Controlling the Lexus" width=60%

@todo #850 Uncomment joystick session when tested again

<!-- ### Controlling LGSVL with a joystick -->

<!-- It is possible to control the simulation with a gamepad or joystick instead of a keyboard. Assuming just one joystick is plugged into the system, just map it into the Docker container when starting ADE by appending the proper `--device` flag: -->

<!-- ``` -->
<!-- $ ade start <ade arguments> -- --device /dev/input/js0 -->
<!-- ``` -->

<!-- @note The instructions in this section were tested with a Logitech Gamepad F310 -->

<!-- #### Troubleshooting -->

<!-- ### The brake/throttle/steering does not work -->

<!-- The joystick control mapping is not deterministic. It is occasionally necessary to modify the axis -->
<!-- mapping. -->

<!-- First, with the joystick controller running, verify that you can see the raw messages by running -->
<!-- the following: -->

<!-- ``` -->
<!-- $ ade enter -->
<!-- ade$ source /opt/AutowareAuto/setup.bash -->
<!-- ade$ ros2 topic echo /joy -->
<!-- ``` -->

<!-- Next, actuate the appropriate axis on the vehicle controllers to determine which buttons and joy -->
<!-- sticks correspond to which indices in the `Joy` message. -->

<!-- Update the `src/tools/joystick_vehicle_interface/param/logitech_f310.defaults.param.yaml` appropriately, or -->
<!-- make a copy. -->

<!-- ### There are no data on the /joy topic -->

<!-- Ensure that `/dev/input/js0` is available from within ADE. -->

<!-- @todo Specific instructions. What should a user do exactly? -->

<!-- If it is not available, restart `ade`, ensuring that the device is appropriately mounted. Alternatively, restart `ade` and run it with the `--privileged` flag, e.g.: -->

<!-- ``` -->
<!-- $ ade start <ade arguments> -- --privileged -->
<!-- ``` -->

<!-- ### The vehicle still does not move -->

<!-- First, ensure the whole stack is running properly, and is appropriately configured. See the section -->
<!-- above titled "No data are being sent through to ROS." -->

<!-- Next, ensure there are data on the `/joy` topic. If this is not the case, refer to the appropriate -->
<!-- question. -->

# Bridging with Autoware.Auto

@todo update check section

SVL uses conventions which are not directly aligned with ROS 2 conventions. The full list of behaviors the `lgsvl_interface` implements is:
-# Converts control inputs with CCW positive rotations to the CCW negative inputs the SVL
simulator expects
-# Provides a mapping from `VehicleControlCommand` to the `RawControlCommand` SVL expects via
parametrizable 1D lookup tables

To run the `lgsvl_interface` manually, enter the following in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run lgsvl_interface lgsvl_interface_exe --ros-args --params-file /opt/AutowareAuto/share/lgsvl_interface/param/lgsvl.param.yaml
```

Autoware.Auto uses PointCloud2 messages with `x,y,z,intensity` rather than `x,y,z,intensity,timestamp` fields.

This node will convert `points_xyzi`

Run `point_type_adapter` to convert the messages.

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch point_type_adapter point_type_adapter.launch.py
```

Launch scripts are also provided for convenience. For example for a joystick control demo, run the following in a new terminal window:

```{bash}
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch joystick_vehicle_interface_nodes lgsvl_joystick.launch.py
```

For an example of using `VehicleControlCommand` with SVL, run the following demo in a new terminal window:

```{bash}
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_vehicle_control_command.launch.py
```
