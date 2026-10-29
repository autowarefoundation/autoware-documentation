# Vehicle dimensions

Autoware describes the size of the vehicle with the parameters in `vehicle_info.param.yaml`. Each vehicle description package has this file, for example [`autoware_sample_vehicle_description`](https://github.com/autowarefoundation/autoware_core/blob/main/description/autoware_sample_vehicle_description/config/vehicle_info.param.yaml). The [`autoware_vehicle_info_utils`](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_vehicle_info_utils) package reads the parameters and calculates the values that depend on them.

The current format supports only the Ackermann model. It is the model that planning and control use. It does not have to be the exact physical model of the vehicle.

## Vehicle axes and base_link

{{ diagram("images/vehicle-axes.svg", "The axes of the base_link frame.") }}

The `base_link` frame is used very frequently throughout the Autoware stack. It is the projection of the rear-axle center onto the ground surface.

- The Localization component outputs the `map` to `base_link` transformation.
- The Planning component plans the poses for where the `base_link` frame should be in the future.
- The Control component tries to fit `base_link` to incoming poses.

## Vehicle dimensions

{{ diagram("images/vehicle-dimensions.svg", "The dimensions of the vehicle in the top view. The front wheels show a positive steering angle.") }}

All lengths are in meters.

### wheel_base

The distance between the front and rear axles.

### wheel_tread

The distance between the centers of the left and right wheels.

### Overhangs

Overhangs are part of the minimum safety box calculation.

When measuring overhangs, side mirrors, protruding sensors and wheels should be taken into consideration.

#### left_overhang

The distance between the axis centers of the left wheels and the left-most point of the vehicle.

#### right_overhang

The distance between the axis centers of the right wheels and the right-most point of the vehicle.

#### front_overhang

The distance between the front axle and the foremost point of the vehicle.

#### rear_overhang

The distance between the rear axle and the rear-most point of the vehicle.

### vehicle_height

The height of the vehicle from the ground.

### max_steer_angle

The maximum steering angle of the tires, in radians.

### vehicle_length

Total length of the vehicle. Calculated by `front_overhang + wheel_base + rear_overhang`

### vehicle_width

Total width of the vehicle. Calculated by `left_overhang + wheel_tread + right_overhang`

### Wheel parameters

{{ diagram("images/wheel-dimensions.svg", "The width and the radius of a wheel.") }}

#### wheel_width

The lateral width of a wheel tire, primarily used for dead reckoning.

#### wheel_radius

The radius of the wheel, primarily used for dead reckoning.

### Footprint

{{ diagram("images/vehicle-footprint.svg", "The footprint polygon that Autoware makes from the vehicle dimensions.") }}

The footprint is the minimum collision area of the vehicle. It is not a parameter. The `createFootprint()` function of `autoware_vehicle_info_utils` makes it from the dimensions above. The points are in clockwise order, with the origin on `base_link`. The center points are at half of `wheel_base`. A planning or control module can add a lateral or longitudinal margin to the footprint.

The AD API reports the dimensions and a footprint polygon of the vehicle through `/api/vehicle/dimensions`.

## Wheel orientations

If the vehicle is going forward, a positive wheel angle will result in the vehicle turning left.

Autoware assumes the rear wheels don't turn on `z` axis.
