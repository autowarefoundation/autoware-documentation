# Vehicle monitoring

AD API provides the current vehicle status for remote monitoring, visualization for passengers, and similar applications.
Use the API below depending on the data you want to monitor.

## Vehicle status

The [vehicle status](../features/vehicle-status.md) provides basic information such as kinematics, indicators, steering, energy, and dimensions.
This allows a remote operator to know the position and velocity of the vehicle.
For applications such as FMS, it can help find vehicles that need assistance, such as vehicles that are stuck or brake suddenly.
It is also possible to determine the actual distance to an object from the vehicle dimensions.

## Planning factors

The [planning factors](../features/planning-factors.md) provide the planning status of the vehicle.
HMI can use this to warn of sudden movements of the vehicle, and to share the stop reason with passengers for comfortable driving.

## Motion state

The [motion state](../features/motion.md) shows whether the vehicle is stopped, is about to start, or is moving.
HMI can use it to make an announcement before the vehicle starts.

## Detected objects

The [perception](../features/perception.md) API provides the objects detected by Autoware.
HMI can use this to visualize objects around the vehicle.
