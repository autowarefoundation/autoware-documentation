# Quick start demos

Each demo runs Autoware in a different kind of simulation. Choose a demo by the components that you want to test.

<div class="aw-demo-table" markdown>

| Demo                                                       |      Sensing       |     Perception     |    Localization    |      Planning      |      Control       |
| :--------------------------------------------------------- | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: |
| [Digital twin](#digital-twin-demos-end-to-end-simulation)  | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: |
| [Planning simulation](#planning-simulation-demo)           |                    |                    |                    | :fa-cl-s fa-check: | :fa-cl-s fa-check: |
| [Rosbag replay simulation](#rosbag-replay-simulation-demo) | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: |                    |                    |
| [Scenario simulator v2](#scenario-simulator-v2-demo)       |                    |                    |                    | :fa-cl-s fa-check: | :fa-cl-s fa-check: |

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![AWSIM drives the ego vehicle through a city](digital-twin-demos/autoware-core-awsim/images/awsim.png){ loading=lazy }

<span class="aw-demo__kicker">End-to-end</span>

</div>

<div class="aw-demo__body" markdown>

## Digital twin demos (End to end simulation)

<div class="aw-demo__facts" markdown>

<div markdown>

<p class="aw-demo__label">Tests</p>

<span class="aw-tag aw-tag--test">All components</span>

All components in the demo. With another configuration, you can test specific components.

</div>

<div markdown>

<p class="aw-demo__label">Simulates</p>

- Sensors (Lidar, camera, GNSS/INS, etc.)
- Vehicle dynamics
- NPCs (Non playable characters / other road users & obstacles)

</div>

</div>

<div class="aw-demo__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: AWSIM Full Demo](digital-twin-demos/awsim-tutorial.md){ .md-button .md-button--primary }
[:fa-cl-s fa-circle-arrow-right: AWSIM with Autoware Core Demo](digital-twin-demos/autoware-core-awsim/index.md){ .md-button }

</div>

Other simulators: [MORAI Sim: Drive](digital-twin-demos/MORAI_Sim-tutorial.md), [CARLA](digital-twin-demos/carla-tutorial.md), and [PanoSim](digital-twin-demos/panosim-demo.md).
{ .aw-demo__more }

</div>

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![The ego vehicle drives on its planned path in RViz](planning-sim/images/lane-following/start-driving.png){ loading=lazy style="object-position: 75% 50%" }

<span class="aw-demo__kicker">Planning and control</span>

</div>

<div class="aw-demo__body" markdown>

## Planning simulation demo

<div class="aw-demo__facts" markdown>

<div markdown>

<p class="aw-demo__label">Tests</p>

<span class="aw-tag aw-tag--test">Planning</span> <span class="aw-tag aw-tag--test">Control</span>

</div>

<div markdown>

<p class="aw-demo__label">Simulates</p>

- Perception output (bounding boxes)
  - Allows you to place dummy objects and simulate their simple movement
  - Traffic light output
- Localization output
  - Allows you to place the ego vehicle anywhere on the map
- Map output
  - Lets you test the validity of Lanelet2 maps

</div>

</div>

<div class="aw-demo__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Planning Simulation Demo](planning-sim/index.md){ .md-button .md-button--primary }

</div>

</div>

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![Recorded lidar data replays around the ego vehicle in RViz](images/rosbag-replay/third-person-follower.png){ loading=lazy style="object-position: 55% 40%" }

<span class="aw-demo__kicker">Recorded data</span>

</div>

<div class="aw-demo__body" markdown>

## Rosbag replay simulation demo

<div class="aw-demo__facts" markdown>

<div markdown>

<p class="aw-demo__label">Tests</p>

<span class="aw-tag aw-tag--test">Sensing</span> <span class="aw-tag aw-tag--test">Perception</span> <span class="aw-tag aw-tag--test">Localization</span>

These components in the demo. With your own recorded data, you can test any other component.

</div>

<div markdown>

<p class="aw-demo__label">Plays back</p>

- Left, right and top lidar output _(in the demo)_
- GNSS/INS data _(in the demo)_
- Vehicle status _(in the demo)_
- Anything else _(based on recorded data)_

</div>

</div>

<div class="aw-demo__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Rosbag Replay Simulation Demo](rosbag-replay-simulation.md){ .md-button .md-button--primary }

</div>

</div>

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![A scenario runs on a map in RViz](scenario-simulation/scenario-simulator/images/scenario_test_runner.png){ loading=lazy style="object-position: 80% 50%" }

<span class="aw-demo__kicker">Scenarios</span>

</div>

<div class="aw-demo__body" markdown>

## Scenario simulator v2 demo

<div class="aw-demo__facts" markdown>

<div markdown>

<p class="aw-demo__label">Tests</p>

<span class="aw-tag aw-tag--test">Planning</span> <span class="aw-tag aw-tag--test">Control</span>

</div>

<div markdown>

<p class="aw-demo__label">Simulates</p>

- Perception output (bounding boxes)
  - Allows you to place dummy objects and simulate their simple movement
  - Traffic light output
- Localization output
  - Allows you to place the ego vehicle anywhere on the map
- Map output
  - Lets you test the validity of Lanelet2 maps

</div>

</div>

<div class="aw-demo__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Scenario Simulator v2 Demo](scenario-simulation/scenario-simulator/installation.md){ .md-button .md-button--primary }

</div>

</div>

</div>
