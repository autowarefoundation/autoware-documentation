# Simulation

You can run Autoware without a vehicle in four kinds of simulation. Each kind simulates different parts of the vehicle and its world, so it tests different components of Autoware. Choose a simulation by the components that you want to test.

Each page shows how to set up the simulation and how to run Autoware in it. Before you start, [install Autoware](../installation/index.md).

<div class="aw-demo-table" markdown>

| Simulation                                            |      Sensing       |     Perception     |    Localization    |      Planning      |      Control       |
| :---------------------------------------------------- | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: |
| [Digital twin simulation](#digital-twin-simulation)   | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: |
| [Planning simulation](#planning-simulation)           |                    |                    |                    | :fa-cl-s fa-check: | :fa-cl-s fa-check: |
| [Rosbag replay simulation](#rosbag-replay-simulation) | :fa-cl-s fa-check: | :fa-cl-s fa-check: | :fa-cl-s fa-check: |                    |                    |
| [Scenario simulation](#scenario-simulation)           |                    |                    |                    | :fa-cl-s fa-check: | :fa-cl-s fa-check: |

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![AWSIM drives the ego vehicle through a city](digital-twin-simulation/autoware-core-awsim/images/awsim.png){ loading=lazy }

<span class="aw-demo__kicker">End-to-end</span>

</div>

<div class="aw-demo__body" markdown>

## Digital twin simulation

<div class="aw-demo__facts" markdown>

<div markdown>

<p class="aw-demo__label">Tests</p>

<span class="aw-tag aw-tag--test">All components</span>

All components of Autoware. With another configuration, you can test specific components.

</div>

<div markdown>

<p class="aw-demo__label">Simulates</p>

- Sensors (Lidar, camera, GNSS/INS, etc.)
- Vehicle dynamics
- NPCs (Non playable characters / other road users & obstacles)

</div>

</div>

<div class="aw-demo__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: AWSIM](digital-twin-simulation/awsim-tutorial.md){ .md-button .md-button--primary }
[:fa-cl-s fa-circle-arrow-right: AWSIM with Autoware Core](digital-twin-simulation/autoware-core-awsim/index.md){ .md-button }

</div>

Other simulators: [MORAI Sim: Drive](digital-twin-simulation/MORAI_Sim-tutorial.md), [CARLA](digital-twin-simulation/carla-tutorial.md), and [PanoSim](digital-twin-simulation/panosim-demo.md). To check that a simulator works with Autoware, run the [compatibility test](digital-twin-simulation/compatibility-test.md).
{ .aw-demo__more }

</div>

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![The ego vehicle drives on its planned path in RViz](planning-simulation/images/lane-following/start-driving.png){ loading=lazy style="object-position: 75% 50%" }

<span class="aw-demo__kicker">Planning and control</span>

</div>

<div class="aw-demo__body" markdown>

## Planning simulation

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

[:fa-cl-s fa-circle-arrow-right: Planning simulation](planning-simulation/index.md){ .md-button .md-button--primary }

</div>

</div>

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![Recorded lidar data replays around the ego vehicle in RViz](images/rosbag-replay/third-person-follower.png){ loading=lazy style="object-position: 55% 40%" }

<span class="aw-demo__kicker">Recorded data</span>

</div>

<div class="aw-demo__body" markdown>

## Rosbag replay simulation

<div class="aw-demo__facts" markdown>

<div markdown>

<p class="aw-demo__label">Tests</p>

<span class="aw-tag aw-tag--test">Sensing</span> <span class="aw-tag aw-tag--test">Perception</span> <span class="aw-tag aw-tag--test">Localization</span>

These components with the sample rosbag. With your own recorded data, you can test any other component.

</div>

<div markdown>

<p class="aw-demo__label">Plays back</p>

- Left, right and top lidar output _(in the sample rosbag)_
- GNSS/INS data _(in the sample rosbag)_
- Vehicle status _(in the sample rosbag)_
- Anything else _(based on recorded data)_

</div>

</div>

<div class="aw-demo__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Rosbag replay simulation](rosbag-replay-simulation.md){ .md-button .md-button--primary }

</div>

</div>

</div>

<div class="aw-demo" markdown>

<div class="aw-demo__media" markdown>

![A scenario runs on a map in RViz](scenario-simulation/scenario-simulator/images/scenario_test_runner.png){ loading=lazy style="object-position: 80% 50%" }

<span class="aw-demo__kicker">Scenarios</span>

</div>

<div class="aw-demo__body" markdown>

## Scenario simulation

<div class="aw-demo__facts" markdown>

<div markdown>

<p class="aw-demo__label">Tests</p>

<span class="aw-tag aw-tag--test">Planning</span> <span class="aw-tag aw-tag--test">Control</span>

</div>

<div markdown>

<p class="aw-demo__label">Simulates</p>

- Other road users and traffic lights, as an OpenSCENARIO scenario describes them
- Perception output (bounding boxes) and the localization output
- The success and failure conditions of the scenario

</div>

</div>

<div class="aw-demo__actions" markdown>

[:fa-cl-s fa-circle-arrow-right: Scenario simulation](scenario-simulation/scenario-simulator/installation.md){ .md-button .md-button--primary }

</div>

To evaluate Autoware with recorded data and pass or fail criteria, use the [Driving Log Replayer](scenario-simulation/driving-log-replayer.md).
{ .aw-demo__more }

</div>

</div>

## Evaluation

The [evaluation pages](evaluation/index.md) describe how to measure the quality of Autoware in these simulations: the evaluator packages that calculate metrics while Autoware runs, and an evaluation of the localization in an urban environment.
