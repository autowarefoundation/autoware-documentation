# Driving

## Description

The driving feature manages whether the vehicle is driving or can start driving.
The vehicle holds a stop when the state is not DRIVING.
This state is mainly used to show the users whether the vehicle can start driving and to implement an HMI for departure.

The state transition for driving state is illustrated in the diagram below.
In a normal operation, this state will transition as follows:
1. Driving state is initialized as NOT_READY
2. State transitions to READY when Autoware components are launched sucesffuly and is ready to start driving.
3. The state becomes DRIVING if the engage API is called and the vehicle starts driving.
4. When the vehicle reaches to the destination, the state returns to NOT_READY.
5. The state can be manually returned to NOT_READY whenever disengage API is called
First, the driving state is NOT_READY and changes depending on other states of the vehicle.
The state becomes READY when it is ready to start driving.
Next, the state becomes DRIVING if the engage API is called, then the vehicle starts driving.
Finally, the state returns to NOT_READY when the vehicle arrives at the destination.
The state can be manually returned to NOT_READY by calling disengage API.

![driving-state](./driving-state.drawio.svg)

| State     | Description                                       |
| --------- | ------------------------------------------------- |
| NOT_READY | The vehicle cannot start driving for some reason. |
| READY     | The vehicle is ready to start driving             |
| DRIVING   | The vehicle is driving toward the destination.    |

## Related API

- /api/driving/state
- /api/driving/engage
