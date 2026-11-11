# ROS 2 distribution transitions

Autoware moves to each new ROS 2 long-term support (LTS) distribution. This page shows the planned transitions on one timeline.

| Distribution  | Ubuntu | Released | End of life | Tracking issue                                                                                           |
| ------------- | ------ | -------- | ----------- | -------------------------------------------------------------------------------------------------------- |
| ROS 2 Humble  | 22.04  | May 2022 | May 2027    |                                                                                                          |
| ROS 2 Jazzy   | 24.04  | May 2024 | May 2029    | [Support ROS 2 Jazzy Jalisco #6695](https://github.com/autowarefoundation/autoware/issues/6695) (closed) |
| ROS 2 Lyrical | 26.04  | May 2026 | May 2031    | [Support ROS 2 Lyrical Luth #7404](https://github.com/autowarefoundation/autoware/issues/7404)           |

{{ diagram("images/ros-2-distributions-timeline.svg", "The transition timeline: Humble support ends in May 2027, Jazzy and Lyrical have full support side by side from January 2027 until January 2029, and from May 2029 Autoware supports only Lyrical.") }}

## Status in Autoware 2.0.0

Autoware 2.0.0 supports ROS 2 Humble and Jazzy. It does not support ROS 2 Lyrical.

- The `autoware.dev_env.install_dev_env` Ansible playbook installs ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu 24.04. It stops on other Ubuntu versions, for example Ubuntu 26.04. See [`install_dev_env.yaml`](https://github.com/autowarefoundation/autoware/blob/2.0.0/ansible/playbooks/install_dev_env.yaml).
- The Docker images are built for Humble and Jazzy, for amd64 and arm64, for example `ghcr.io/autowarefoundation/autoware:universe-cuda-jazzy-2.0.0`. Jazzy is the default `ROS_DISTRO` in [`docker-bake.hcl`](https://github.com/autowarefoundation/autoware/blob/2.0.0/docker/docker-bake.hcl). No Docker image uses Lyrical.
- A pull request to `autoware`, `autoware_core`, or `autoware_universe` must pass the required build checks for both Humble and Jazzy before it can merge. No CI check uses Lyrical.
- The ROS 2 build farm has Lyrical releases of the Autoware message packages, `autoware_cmake`, `autoware_utils`, `autoware_lanelet2_extension`, and `ros2_socketcan`. `autoware_core` has no Lyrical release.

## Next steps

From December 2026 until the support for Humble ends in May 2027, the CI checks Humble, Jazzy, and Lyrical.

1. **December 2026:** Lyrical Docker beta. The CI adds the Lyrical checks as required checks. Source fixes for the Ubuntu 26.04 toolchain come before this date. Each fix must also keep the Jazzy build working.
2. **January 2027:** Lyrical full support, on January 1, 2027, at the latest. It can come earlier. In the same month, the Humble soft-freeze starts. The full support for Humble ends, and Humble gets only critical fixes.
3. **May 2027:** Humble end of support. The CI stops the Humble checks and checks Jazzy and Lyrical.
4. **January 2029:** Jazzy soft-freeze. The full support for Jazzy ends. After this date, Jazzy gets only critical fixes.
5. **May 2029:** Lyrical exclusive mode. The CI checks only Lyrical, and the support for Jazzy ends.
