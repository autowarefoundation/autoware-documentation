# How is Autoware Core/Universe different from Autoware.AI and Autoware.Auto?

Autoware is the world's first "all-in-one" open-source software for self-driving vehicles.
Since it was first released in 2015, there have been multiple releases made with differing underlying concepts, each one aimed at improving the software.

## Autoware.AI

[Autoware.AI](https://github.com/Autoware-AI/autoware.ai) is the first distribution of Autoware that was released based on ROS 1. The repository contains a variety of packages covering different aspects of autonomous driving technologies - sensing, actuation, localization, mapping, perception and planning.

While it was successful in attracting many developers and contributions, it was difficult to improve Autoware.AI's capabilities for a number of reasons:

- A lack of concrete architecture design leading to a lot of built-up technical debt, such as tight coupling between modules and unclear module responsibility.
- Differing coding standards for each package, with very low test coverage.

Furthermore, there was no clear definition of the conditions under which an Autoware-enabled autonomous vehicle could operate, nor of the use cases or situations supported (eg: the ability to overtake a stationary vehicle).

From the lessons learned from Autoware.AI development, a different development process was taken for Autoware.Auto to develop a ROS 2 version of Autoware.

!!! warning

    Autoware.AI is currently in maintenance mode and will reach end-of-life at the end of 2022.

## Autoware.Auto

[Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto) is the second distribution of Autoware that was released based on ROS 2. As part of the transition to ROS 2, it was decided to avoid simply porting Autoware.AI from ROS 1 to ROS 2. Instead, the codebase was rewritten from scratch with proper engineering practices, including defining target use cases and ODDs (eg: Autonomous Valet Parking [AVP], Cargo Delivery, etc.), designing a proper architecture, writing design documents and test code.

Autoware.Auto development seemed to work fine initially, but after completing the AVP and and Cargo Delivery ODD projects, we started to see the following issues:

- The barrier to new engineers was too high.
  - A lot of work was required to merge new features into Autoware.Auto, and so it was difficult for researchers and students to contribute to development.
  - As a consequence, most Autoware.Auto developers were from companies in the Autoware Foundation and so there were very few people who were able to add state-of-the-art features from research papers.
- Making large-scale architecture changes was too difficult.
  - To try out experimental architecture, there was a very large overhead involved in keeping the main branch stable whilst also making sure that every change satisfied the continuous integration requirements.

## Autoware Core/Universe

In order to address the issues with Autoware.Auto development, the Autoware Foundation decided to create a new architecture called Autoware Core/Universe.

Autoware Core carries over the original policy of Autoware.Auto to be a stable and well-tested codebase. Alongside Autoware Core is a new concept called Autoware Universe, which acts as an extension of Autoware Core with the following benefits:

- Users can easily replace a Core component with a Universe equivalent in order to use more advanced features, such as a new Localization or Perception algorithm.
- Code quality requirements for Universe are more relaxed to make it easier for new developers, students and researchers to contribute, but will still be stricter than the requirements for Autoware.AI.
- Any advanced features added to Universe that are useful to the wider Autoware community will be reviewed and considered for potential inclusion in the main Autoware Core codebase.

This way, the primary requirement of having a stable and safe autonomous driving system can be achieved, whilst simultaneously enabling access to state-of-the-art features created by third-party contributors. For more details about the design of Autoware Core/Universe, refer to the [Autoware concepts documentation page](../autoware-concepts/).
