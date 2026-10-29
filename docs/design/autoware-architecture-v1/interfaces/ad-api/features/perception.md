# Perception

## Related API

- {{ link_ad_api('/api/perception/objects') }}

## Description

This API provides the objects that Autoware detects around the vehicle.
Applications can use it to show the objects to the operator and the passengers, for example in an HMI or a remote monitoring system.

Each object in the array has these fields:

| Field                   | Description                                                                                        |
| ----------------------- | -------------------------------------------------------------------------------------------------- |
| `id`                    | The UUID of the object. It stays the same while Autoware tracks the object.                        |
| `existence_probability` | The probability that the object exists.                                                            |
| `classification`        | The classes of the object with their probabilities, for example `CAR`, `PEDESTRIAN`, or `BICYCLE`. |
| `kinematics`            | The pose, twist, and acceleration of the object, and its predicted paths with their confidence.    |
| `shape`                 | The shape of the object as a box, a cylinder, or a prism (a polygon with a height).                |

The pose uses the frame of the message header.

## Default implementation

The `perception` node of `autoware_default_adapi_universe` converts the predicted objects of the perception component (`/perception/object_recognition/objects`) to this message.
It converts the bounding box shape to a box, the cylinder shape to a cylinder, and the polygon shape to a prism.
