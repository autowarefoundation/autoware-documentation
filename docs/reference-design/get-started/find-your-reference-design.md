# Find your reference design

Use the decision tree to identify which reference configuration matches your deployment scenario. Then explore the real-world examples that demonstrate each configuration.

## Decision tree

{{ diagram("images/reference-design-decision-tree.svg", "The decision tree leads from the technology maturity, the environment, the location, and the vehicle size to a reference design.") }}

The decision tree asks up to four questions:

1. Technology maturity: a system at Technology Readiness Level (TRL) 7 or higher is close to production. See the [example production platforms](#example-production-platforms). A system at TRL 6 or lower starts from a design guideline.
2. Environment: an indoor-only system uses the Low Speed Autonomy (LSA) guideline, a racing robot, or the Go-Kart. A mixed indoor and outdoor system uses the LSA guideline.
3. Location: an outdoor system on a closed campus uses the LSA guideline. A highway system uses the Privately Owned Vehicles (PoV) guideline. An off-road system uses the off-road vehicle guideline.
4. Size: a city system that is large is a bus. A city system of sedan size is a robotaxi.

---

## Example production platforms

| Configuration                                                                                                                                                          | Best for                           | Environment          | Localization | Key feature               |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- | -------------------- | ------------ | ------------------------- |
| [KingWayTek Micro LSV](https://github.com/autowarefoundation/LSA-reference-design-docs/blob/main/docs/get-started/KWT_LSV/Kingway_LSV_introduction_ENG_0729AWF_v1.pdf) | Mixed traffic flows and low budget | Paved, GPS available | RTK GNSS     | Balanced cost/capability  |
| [Bus](../itri-bus/index.md)                                                                                                                                            | Shuttle services for fixed route   | Paved, GPS available | RTK GNSS     | Comfortable and safe ride |
| [RoboTaxi](https://chishengshih.github.io/autoware-documentation/latest/reference-design/LeoTaxi/ReferenceDesignforLeoTaxi/)                                           | Point-to-point on-demand services  | Paved, GPS available | RTK GNSS     | Comfortable and safe ride |

## Reference conceptual design guidelines

These are conceptual configurations optimized for different deployment scenarios. Each defines recommended components, performance targets, and ODD coverage.

| Configuration                                                                             | Best for                           | Environment            | Localization | Key feature                          |
| ----------------------------------------------------------------------------------------- | ---------------------------------- | ---------------------- | ------------ | ------------------------------------ |
| [LSA](https://autowarefoundation.github.io/LSA-reference-design-docs/main/)               | Mixed traffic flows and low budget | Paved, GPS available   | RTK GNSS     | Balanced cost/capability             |
| [PoV](https://autowarefoundation.github.io/autoware.pov-reference-design-docs/main/)      | Highway                            | Paved, GPS available   | RTK GNSS     | End-to-End Machine Learning Pipeline |
| [Off-road](https://autowarefoundation.github.io/autoware.orv-reference-design-docs/main/) | Racing, education, and research    | Unpaved, GPS available | Single GNSS  | No map and AI-enabled                |
| [RoboRacer](RoboRacer/index.md)                                                           | Racing and education               | Indoor tracks          | No           | Racing robots using Autoware         |
| [Go-Kart](GoKart/index.md)                                                                | Racing and development             | Outdoor                | RTK GNSS     | EV Go-Kart using Autoware            |
| [Delivery vehicle](Delivery/index.md)                                                     | Cargo delivery on private campus   | Private campus         | GNSS/IMU     | Symmetric vehicle with crab steering |

For the components, performance targets, and ODD coverage of each LSA configuration, see [Design choices by example](build-examples.md).

## Real-world examples

These deployments demonstrate the Campus configuration of the LSA guideline. The [LSA guideline](https://autowarefoundation.github.io/LSA-reference-design-docs/main/get-started/find-your-reference-design/) describes them in more detail.

### TalTech iseAuto

iseAuto is the autonomous shuttle of the Autonomous Vehicles research group at Tallinn University of Technology (TalTech) in Estonia. On its campus test track, researchers and partners test teleoperation, V2X, cybersecurity, and connectivity over its private 5G network.

| Aspect       | Details                                 |
| ------------ | --------------------------------------- |
| Platform     | Custom electric shuttle                 |
| Sensors      | 3D LiDARs, automotive cameras, GNSS/IMU |
| Connectivity | Private 5G network                      |

Learn more on the [TalTech page of the Autoware Foundation](https://autoware.org/autowareio/taltech/).

### NC A&T Aggie Auto

Aggie Auto is the autonomous shuttle program of North Carolina A&T State University. A one-month public pilot program connected the campus with downtown Greensboro. The two areas are about one mile apart, and the vehicles drove at up to 25 mph.

| Aspect        | Details                                                   |
| ------------- | --------------------------------------------------------- |
| Platform      | GEM e6 electric vehicles                                  |
| Sensors       | Multi-sensor LiDAR suite, NovAtel GNSS with dual antennas |
| Drive-by-wire | AutonomouStuff PACMod                                     |

Learn more on the [Aggie Autonomous Shuttles website](https://www.aggieauto.com/about).
