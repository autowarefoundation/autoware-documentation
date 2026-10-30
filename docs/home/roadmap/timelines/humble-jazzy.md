# ROS 2 Humble to Jazzy transition

Tracked in [Support ROS 2 Jazzy Jalisco #6695](https://github.com/autowarefoundation/autoware/issues/6695). The issue is closed. Autoware 2.0.0 supports both ROS 2 distributions.

- ROS 2 Humble reaches its end of life in May 2027.
- ROS 2 Jazzy was released in May 2024 and has support until May 2029.

This timeline is for Autoware's transition:

- **From:** Ubuntu 22.04 | ROS 2 **Humble**
- **To:** Ubuntu 24.04 | ROS 2 **Jazzy**

{{ diagram("images/humble-jazzy-timeline.svg", "The transition timeline: Humble and Jazzy have full support side by side until January 2027, and from May 2027 Autoware supports only Jazzy.") }}

## Status in Autoware 2.0.0

The milestones of February and April 2026 are complete. In Autoware 2.0.0:

- The `autoware.dev_env.install_dev_env` Ansible playbook installs ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu 24.04. It stops on other Ubuntu versions. See [`install_dev_env.yaml`](https://github.com/autowarefoundation/autoware/blob/2.0.0/ansible/playbooks/install_dev_env.yaml).
- The Docker images are built for Humble and Jazzy, for amd64 and arm64, for example `ghcr.io/autowarefoundation/autoware:universe-cuda-jazzy-2.0.0`. Jazzy is the default `ROS_DISTRO` in [`docker-bake.hcl`](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/docker-bake.hcl).
- A pull request to `autoware`, `autoware_core`, or `autoware_universe` must pass the required build checks for both Humble and Jazzy before it can merge.

## Next steps

The milestones after Autoware 2.0.0 are the plan from the tracking issue:

1. **2027 January:** Humble soft-freeze. The standard support for Humble ends. After this date, Humble gets only critical fixes.
2. **2027 May:** Jazzy exclusive mode. The CI checks only Jazzy, and the support for Humble ends.
