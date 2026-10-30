# Open source philosophy

Autoware is an open source software framework for autonomous driving, licensed under the Apache 2.0 license. This permissive licensing model allows users to freely use, modify, and distribute the software, even for commercial purposes, while ensuring proper attribution. This provides the following benefits to the users and developers:

- **Transparency & Validation:** Open access to the source code allows researchers, developers, and industry experts to inspect, validate, and improve the software, ensuring robustness and reliability.
- **Collaboration & Innovation:** A shared development model encourages contributions from a global community, accelerating advancements in autonomous technology.
- **Interoperability & Standardization:** Autoware adheres to common standards and widely adopted environments, ensuring seamless integration with diverse platforms, sensors, and vehicle architectures.
- **Cost & Accessibility:** By eliminating proprietary software costs, Autoware lowers the barriers to entry for startups, researchers, and commercial developers, enabling broader adoption and experimentation.

The sections below explain how these principles shape the way the Autoware project works.

## Develop in the open

Autoware is developed in public repositories on GitHub under the [Autoware Foundation organization](https://github.com/autowarefoundation). The work happens in public too:

- Questions, feature requests, and design proposals start in [GitHub Discussions](https://github.com/orgs/autowarefoundation/discussions).
- Confirmed bugs and tasks are tracked in the GitHub issues of each repository.
- Changes go through public pull requests, with their review comments and CI results.
- The Technical Steering Committee (TSC) and the working groups publish their meeting minutes in the [TSC meetings](https://github.com/orgs/autowarefoundation/discussions/categories/tsc-meetings) and [Working group meetings](https://github.com/orgs/autowarefoundation/discussions/categories/working-group-meetings) discussion categories.
- The working groups are open to everyone. The meeting schedule is in the [Autoware Foundation events calendar](https://calendar.google.com/calendar/u/0/embed?src=autoware.org_6lol0ho5ft0217h8c60pi1fm30@group.calendar.google.com).

Because the discussion and the decisions are public, anyone can find out why the code is the way it is.

## One set of rules for every contributor

Individuals, companies, and research groups contribute to Autoware under the same rules:

- **License:** Each contribution is licensed under the Apache License 2.0, as clause 5 of the license states. Refer to [License](license.md).
- **Developer Certificate of Origin:** Each commit carries a `Signed-off-by` line. With it, the author certifies that they have the right to submit the change. Refer to the [commit guidelines](pull-request-guidelines/commit-guidelines.md).
- **Review:** Pull requests are reviewed before they are merged, and the code owners of the affected packages are asked to review. Refer to the [pull request guidelines](pull-request-guidelines/index.md).
- **Accountability:** The contributor is the author and is responsible for every line, whatever tools they used. Refer to the [AI contribution policy](ai-contribution-policy.md).
- **Conduct:** Everyone follows the [Code of Conduct](https://github.com/autowarefoundation/autoware/blob/main/CODE_OF_CONDUCT.md).

## Quality and openness in layers

Autoware separates the packages that need strict quality control from the packages that need room for new ideas:

- [Autoware Core](https://github.com/autowarefoundation/autoware_core) holds the foundational packages that the Autoware Foundation maintains, with strict development standards.
- [Autoware Universe](https://github.com/autowarefoundation/autoware_universe) holds packages that individuals, companies, and research groups contribute. Their authors maintain them.
- The [Autoware Index](https://index.autoware.org) lists community packages that live in their own repositories. Its CI builds and tests each registered package against the latest Autoware release.

Mature packages from Universe can move into Core. For details, refer to [Autoware concepts](../design/autoware-concepts/index.md#core-and-universe-repository-model) and [What belongs in Autoware Core](../design/autoware-concepts/core-package-inclusion-criteria.md).

## Build on open standards

Autoware is built on [ROS 2](https://docs.ros.org/), and its components talk to each other through ROS 2 topics, services, and messages. The interfaces between the components and the [Autoware AD API](../design/autoware-architecture-v1/interfaces/ad-api/index.md) are documented, so that you can replace one component or connect your own system without changes to the rest. Maps use the open [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format with the [Autoware extension](https://github.com/autowarefoundation/autoware_lanelet2_extension).

## Contribute back

Autoware improves when its users share their work. If you fix a bug, adapt Autoware to new hardware, or build a new feature, consider contributing it:

- Make a pull request to the related repository. Refer to the [contribution guidelines](index.md).
- Register your own packages in the [Autoware Index](../installation/autoware/autoware-index.md), so that other users can find them.
- Share your experience in [GitHub Discussions](https://github.com/orgs/autowarefoundation/discussions), for example in the Show and tell category.
