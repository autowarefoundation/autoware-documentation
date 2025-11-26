# AWSIM simulator

AWSIM is a simulator for Autoware development and testing, initially developed by TIER IV and still actively maintained.

AWSIM Labs is a fork of AWSIM, developed under the Autoware Foundation, providing additional features and lighter resource usage.

## Feature differences from the AWSIM and AWSIM Labs

### Simulator Features

| Simulator Features                                             | AWSIM 1.2.3 | AWSIM Labs 1.4.2 |
| -------------------------------------------------------------- | ----------- | ---------------- |
| Rendering Pipeline                                             | HDRP        | URP              |
| Resource usage                                                 | Heavy 🐢    | Light 🐇         |
| Can toggle vehicle keyboard control from GUI                   | ✅          | ✅               |
| Radar sensor support                                           | ✅          | ✅               |
| Lidar sensor multiple returns support                          | ✅          | ❌               |
| Raw radar output                                               | ✅          | ❌               |
| Lidar snow energy loss feature                                 | ✅          | ❌               |
| Physically based vehicle dynamics simulation (VPP integration) | ❌          | ✅               |
| Can reset vehicle position on runtime                          | ❌          | ✅               |
| Select maps and vehicles at startup                            | ❌          | ✅               |
| Scenario simulator integrated into the same binary             | ❌          | ✅               |
| Multi-lidars are enabled by default                            | ❌          | ✅               |
| Set vehicle pose and spawn objects from RViz2                  | ❌          | ✅               |
| Visualize multiple cameras and move them dynamically           | ❌          | ✅               |
| Turn sensors on/off during runtime                             | ❌          | ✅               |
| Graphics quality settings (Low/Medium/Ultra)                   | ❌          | ✅               |
| Bird’s eye view camera option                                  | ❌          | ✅               |
| Works with the latest Autoware main branch                     | ❌          | ✅               |

### Development Features

| Development Features                              | AWSIM            | AWSIM Labs                |
| ------------------------------------------------- | ---------------- | ------------------------- |
| Unity Version                                     | Unity 2021.1.7f1 | Unity LTS 2022.3.36f1     |
| CI for build                                      | ✅               | ❌ (disabled temporarily) |
| Various regression unit tests                     | ✅               | ❌                        |
| CI for documentation generation within PR         | ❌               | ✅                        |
| Main branch is protected with linear history      | ❌               | ✅                        |
| Pre-commit for code formatting                    | ❌               | ✅                        |
| Documentation page shows PR branches before merge | ❌               | ✅                        |

## AWSIM Labs

[AWSIM Labs](https://github.com/autowarefoundation/AWSIM-Labs) supports Unity LTS 2022.3.36f1 and uses the Universal Render Pipeline (URP), optimized for lighter resource usage. It introduces several enhancements such as the ability to reset vehicle positions at runtime, support for multiple scenes and vehicle setups on runtime, and multi-lidars enabled by default.

To get started with AWSIM Labs, please follow the [instructions](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/QuickStartDemo/).

## AWSIM

[AWSIM](https://github.com/tier4/AWSIM) runs on Unity 2021.1.7f1 using the High Definition Render Pipeline (HDRP), which requires more system resources.

To get started with AWSIM, please follow the [instructions](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/) provided by TIER IV.
