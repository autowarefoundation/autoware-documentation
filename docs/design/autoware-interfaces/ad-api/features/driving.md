# Driving API

- [/api/driving/state](../list/api/driving/state.md)
- [/api/driving/engage](../list/api/driving/engage.md)

## Description

This API manages whether the vehicle is driving or can start driving.
The user can use this API to check if the vehicle is ready to drive and instruct it to depart.

## States

The state transition for driving state is illustrated in the diagram below.
The vehicle holds a stop when the state is not DRIVING.
During normal operation, the flow of driving state transitions is as follows:

1. Driving state is initialized to NOT_READY.
2. The state transitions to READY when the autonomous system is ready to start driving.
3. The state becomes DRIVING if the engage API is called and the vehicle starts driving.
4. The state returns to NOT_READY when the vehicle reaches its destination.
5. The state can be manually returned to NOT_READY by calling the disengage API.

![driving-state](./driving-state.drawio.svg)

| State     | Description                                     |
| --------- | ----------------------------------------------- |
| NOT_READY | The vehicle is not ready to start driving.      |
| READY     | The vehicle is ready to start driving           |
| DRIVING   | The vehicle is driving towards the destination. |
