# Generator-Selector Architecture

To flexibly accommodate these diverse approaches, we propose a **Generator-Selector Framework**.
This framework consists of two core components:

- **Generator**: Generates candidate trajectories that the vehicle can follow.
- **Selector**: Selects the safest and most optimal trajectory from among the candidates.

![Generator-Selector Architecture Diagram](media/generator-selector.png)

## Generator

The **Generator** can be any module capable of generating candidate trajectories that the vehicle may follow.
Examples include:

- **Rule-based**: Existing Autoware planners can be reused as-is.
- **Optimization-based**: Methods such as _Model Predictive Control (MPC)_.
- **Machine learning-based**: Component-based E2E modular approach or Monolithic E2E.

Multiple Generators can be executed **in parallel** or **selectively activated depending on the context**.

## Selector

The **Selector** is responsible for two main functions:

### 1. Safety Gate (Safety Assurance)

Validates the output of black-box Generators (e.g., neural networks) to ensure a **minimum level of safety**.
Examples:

- Check using an HD map to ensure that traffic signals are obeyed.

### 2. Ranking (Trajectory Evaluation and Selection)

Evaluates and ranks the outputs from multiple Generators, then selects the best one.
Examples:

- Use a robotics-based approach when an HD map is available; fall back to E2E models otherwise.
- Score trajectories based on driving policies such as **safety, comfort, or rule compliance**.

These Selector functions are implemented as **plugins**, allowing developers to:

- Customize safety requirements.
- Inject preference to select suitable trajectories for their own use case.

---

## Reference Links

- GitHub Discussions:
  - [Discussion #5033](https://github.com/orgs/autowarefoundation/discussions/5033)
  - [Discussion #6301](https://github.com/orgs/autowarefoundation/discussions/6301)
- Proposed Architecture: [New Planning Framework Wiki](https://github.com/tier4/new_planning_framework/wiki)
- GitHub Issue: [#6292](https://github.com/autowarefoundation/autoware/issues/6292)
- Migration Plan: [Enable generator-selector planning framework (#6292)](https://github.com/autowarefoundation/autoware/issues/6292)
