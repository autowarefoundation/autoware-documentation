# AWSIM simulator

AWSIM is a simulator for Autoware development and testing, initially developed by TIER IV and still actively maintained.

AWSIM Labs is a fork of AWSIM, developed under the Autoware Foundation, providing additional features and lighter resource usage.

## Feature differences from the AWSIM and AWSIM Labs

| Simulator Features                           | AWSIM 1.2.1 | AWSIM Labs 1.0.0 |
| -------------------------------------------- | ----------- | ---------------- |
| Rendering Pipeline                           | HDRP        | URP              |
| Resource usage                               | Heavy       | Light            |
| Can reset vehicle position on runtime        | ❌          | ✅               |
| Multiple scene and vehicle setup             | ❌          | ✅               |
| Multi-lidars are enabled by default          | ❌          | ✅               |
| Radar sensor support                         | ✅          | ❌               |
| Can toggle vehicle keyboard control from GUI | ✅          | ❌               |

| Development Features                         | AWSIM            | AWSIM Labs            |
| -------------------------------------------- | ---------------- | --------------------- |
| Unity Version                                | Unity 2021.1.7f1 | Unity LTS 2022.3.21f1 |
| CI for build                                 | ❌               | ✅                    |
| CI for documentation generation within PR    | ❌               | ✅                    |
| Main branch is protected with linear history | ❌               | ✅                    |

## AWSIM Labs

[AWSIM Labs](https://github.com/autowarefoundation/AWSIM-Labs) supports Unity LTS 2022.3.21f1 and uses the Universal Render Pipeline (URP), optimized for lighter resource usage. It introduces several enhancements such as the ability to reset vehicle positions at runtime, support for multiple scenes and vehicle setups on runtime, and multi-lidars enabled by default.

To get started with AWSIM Labs, please follow the [instructions](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/QuickStartDemo/).

## AWSIM

[AWSIM](https://github.com/tier4/AWSIM) runs on Unity 2021.1.7f1 using the High Definition Render Pipeline (HDRP), which requires more system resources.

To get started with AWSIM, please follow the [instructions](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/) provided by TIER IV.
