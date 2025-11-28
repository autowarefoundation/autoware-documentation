# Quick start demos

## Digital twin demos (End to end simulation)

- [🔗 AWSIM Full Demo](digital-twin-demos/awsim-tutorial.md)
- [🔗 AWSIM with Autoware Core Demo](digital-twin-demos/autoware-core-awsim/index.md)

???+ abstract "Summary"

    **Tests:**

    - All components _(in the demo)_
    - Can test specific components _(based on the configuration)_

    **Simulates:**

    - Sensors (Lidar, camera, GNSS/INS, etc.)
    - Vehicle dynamics
    - NPCs (Non playable characters / other road users & obstacles)

## Planning simulation demo

[🔗 Planning Simulation Demo](planning-simulation.md)

???+ abstract "Summary"

    **Tests:**

    - Planning component
    - Control component

    **Simulates:**

    - Perception output (bounding boxes)
      - Allows you to place dummy objects and simulate their simple movement
      - Traffic light output
    - Localization output
      - Allows you to place the ego vehicle anywhere on the map
    - Map output
      - Lets you test the validity of Lanelet2 maps

## Rosbag replay simulation demo

[🔗 Rosbag Replay Simulation Demo](rosbag-replay-simulation.md)

???+ abstract "Summary"

    **Tests:**

    - Sensing component _(in the demo)_
    - Perception component _(in the demo)_
    - Localization component _(in the demo)_
    - Anything else _(based on recorded data)_

    **Plays back:**

    - Left, right and top lidar output _(in the demo)_
    - GNSS/INS data _(in the demo)_
    - Vehicle status _(in the demo)_
    - Anything else _(based on recorded data)_

## Scenario simulator v2 demo

[🔗 Scenario Simulator v2 Demo](scenario-simulation/scenario-simulator/installation.md)

???+ abstract "Summary"

    **Tests:**

    - Planning component
    - Control component

    **Simulates:**

    - Perception output (bounding boxes)
      - Allows you to place dummy objects and simulate their simple movement
      - Traffic light output
    - Localization output
      - Allows you to place the ego vehicle anywhere on the map
    - Map output
      - Lets you test the validity of Lanelet2 maps
