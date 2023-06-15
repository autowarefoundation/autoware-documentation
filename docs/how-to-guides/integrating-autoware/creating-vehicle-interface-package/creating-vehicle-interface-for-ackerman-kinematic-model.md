# Creating vehicle interface for ackerman kinematic model

!!! warning

    Under Construction

## What is vehicle interface
`vehicle interface` is an interface that connects the control commands and your hardward.  
Autoware publishes control commands such as

- Longitudinal control
- Steering control
- Car light commands 

Then, `vehicle interface` converts these commands into actuations such like

- Motors and breaks
- The steering wheel
- Lighting control

This page shows you a brief explanation how to implement your `vehicle interface`, [but you can see further information of `vehicle interface` in the "design" page.](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-interface/)

**Note that there is no package named `vehicle interface` prepared in Autoware. It is a necessary package to actuate your vehilce, but you have to create one by yourself since it is very specific to your hardware.**

For example, if you are using a by-wire kit [PACMod](https://autonomoustuff.com/platform/pacmod), a `vehicle interface` named [`pacmod_interface` published by TIER IV, Inc.](https://github.com/tier4/pacmod_interface/tree/main) is available. 

If you have found a ROS2 package or support for your hardware, you can implement your own `vehicle interface` from it. Of course you can make your own from scratch.

1. Create a directory for your `vehicle interface`. From this example, the interface is named as `my_vehicle_interface`. It is recommended to create this directory at **TBD**
```
cd <your-autoware-dir>/src/vehicle/external
mkdir my_vehicle_interface
```

2. Install the package if it is already applicable to Autoware. If not, you have to implement some packages to pass commands obtained from Autoware to your hardware in an appropriate form. Note that the Control component of Autoware gives us control commands for Ackermann kinematic model vehicles. Therefore, you have to modified the control commands if your vehicle does not suit this model. [Another document gives you an example how to convert your Ackermann kinematic model control inputs into a differential drive model.](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/customizing-for-differential-drive-model/)

3. Declare `my_vehicle_interface` to a launch file in order to launch it immediately when Autoware has been launched.

## Ackermann kinematic model
Autoware now supports control inputs for vehicles based on an Ackermann kinematic model.

### Model description

